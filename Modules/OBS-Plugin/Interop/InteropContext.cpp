//    *************************** LiveVisionKit ****************************
//    Copyright (C) 2022  Sebastian Di Marco (crowsinc.dev@gmail.com)
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.
// 	  **********************************************************************

#include "InteropContext.hpp"

#include <LiveVisionKit.hpp>
#include "Utility/Logging.hpp"
#include "Utility/ScopedProfiler.hpp"

#ifdef _WIN32
#include <dxgi.h>
#include <d3d11.h>
#include <opencv2/core/directx.hpp>
#else
#include <opencv2/core/opengl.hpp>
#endif

namespace lvk::ocl
{

//---------------------------------------------------------------------------------------------------------------------

	cv::ocl::OpenCLExecutionContext InteropContext::s_OCLContext;
	std::optional<bool> InteropContext::s_TestPassed = std::nullopt;
	std::optional<bool> InteropContext::s_Supported = std::nullopt;
	graphics_t* InteropContext::s_GraphicsContext = nullptr;
	std::thread::id InteropContext::s_BoundThread;

//---------------------------------------------------------------------------------------------------------------------

	bool InteropContext::TryAttach()
	{
		LVK_ASSERT(gs_get_context() != nullptr);

		// Don't attempt attachment if not supported, or our last attachment failed to pass tests
		if(!Supported() || !s_TestPassed.value_or(true))
			return false;

		// Create the OCL interop context if it does not yet exist
		if(s_OCLContext.empty())
		{
			s_TestPassed = true;

			// Create the interop context.
			// NOTE: This may fail on some (Linux) systems where driver support is a little
			// iffy so we must be ready to catch an exception and deal with it correctly.
			try
			{
#ifdef _WIN32
				// DirectX11 Context
				auto device = static_cast<ID3D11Device*>(gs_get_device_obj());
				cv::directx::ocl::initializeContextFromD3D11Device(device);
#else
				// OpenGL Context
				cv::ogl::ocl::initializeContextFromGL();
#endif
			}
			catch(...)
			{
				s_TestPassed = false;
			}

			if(!s_TestPassed.value())
			{
				lvk::log::error("The OpenCL interop context failed to initialize and has been disabled! (bad drivers?)");
				return false;
			}
			else lvk::log::print("The OpenCL interop context was successfully created!");

			s_OCLContext = cv::ocl::OpenCLExecutionContext::getCurrent();
			s_BoundThread = std::this_thread::get_id();
			s_GraphicsContext = gs_get_context();

			// Test the context as some (Linux) systems crash when using interop,
			// despite correctly supporting and initializing the interop context.

			const int test_size = 64;

			gs_texture_t* obs_texture = gs_texture_create(
				test_size,
				test_size,
				GS_RGBA_UNORM,
				1,
				nullptr,
				GS_SHARED_TEX
			);

			cv::UMat cv_texture(
				test_size,
				test_size,
				CV_8UC4,
				cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY
			);

			try
			{
				Export(cv_texture, obs_texture);
				Import(obs_texture, cv_texture);
			}
			catch(...)
			{
				s_TestPassed = false;
			}

			gs_texture_destroy(obs_texture);

			if(!s_TestPassed.value())
			{
				Release();
				lvk::log::error("The OpenCL interop context failed to pass validation tests and has been disabled!");
				return false;
			}
			else lvk::log::print("The OpenCL interop context passed all validation tests!");
		}

		// NOTE: We are making the assumption that 
		// OBS only ever has one graphics context. 
		LVK_ASSERT(gs_get_context() == s_GraphicsContext);
		if(!Attached())
		{
			// If the context is not attached to the current thread, then bind it
			s_OCLContext.bind();
			s_BoundThread = std::this_thread::get_id();
			log::warn("The OpenCL interop context was bound to a new graphics thread");
		}

		return true;
	}

//---------------------------------------------------------------------------------------------------------------------

	// Destroys the context
	void InteropContext::Release()
	{
		if(!s_OCLContext.empty())
		{
			s_OCLContext.getContext().release();
			s_OCLContext.release();

			s_BoundThread = {};
			s_GraphicsContext = nullptr;
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	bool InteropContext::Supported()
	{
		if(!s_Supported.has_value())
		{
			// NOTE: this check is actually relatively slow for what it does (~0.1ms)
			if(cv::ocl::haveOpenCL())
			{
				auto& device = cv::ocl::Device::getDefault();
				try 
				{
					// For the interop context to be supported, we must meet two conditions:
					// 
					// * The respective OpenCV DirectX and OpenGL APIs must be included in 
					//   the libary compilation. This can be tested by calling the API and 
					//   testing that no exception is generated. Volatile is used to the 
					//   compiler from optimizing out the test calls. 
					// 					
					// * The OpenCL device must support the necessary interop extensions. 
					//   This can be tested directly from the device's extension list. 

#ifdef _WIN32       // DirectX11 
					
					volatile auto test = cv::directx::getTypeFromDXGI_FORMAT(DXGI_FORMAT_UNKNOWN);
					
					s_Supported = device.isExtensionSupported("cl_nv_d3d11_sharing")
						       || device.isExtensionSupported("cl_khr_d3d11_sharing");
					
#else               // OpenGL
					
					// NOTE: this constructor does not invoke OpenGL texture creation.
					volatile cv::ogl::Texture2D test;

					s_Supported = device.isExtensionSupported("cl_khr_gl_sharing");
#endif
				}
				catch (const std::exception& e) 
				{ 
					s_Supported = false; 
				}
			}
			else s_Supported = false;
		}

		return s_Supported.value();
	}

//---------------------------------------------------------------------------------------------------------------------

	bool InteropContext::Attached()
	{
		return !s_OCLContext.empty() && s_BoundThread == std::this_thread::get_id();
	}

//---------------------------------------------------------------------------------------------------------------------

	bool InteropContext::Available()
	{
		return !s_OCLContext.empty() && s_TestPassed.value_or(false);
	}

//---------------------------------------------------------------------------------------------------------------------

	void InteropContext::Import(gs_texture_t* src, cv::UMat& dst)
	{
		LVK_ASSERT(Available());
		LVK_ASSERT(src != nullptr);
        LVK_PROFILE;

        // Attach the context if it is detached from the thread.
        if(Available() && !Attached())
            TryAttach();

#ifdef _WIN32 // DirectX11 Interop
		
		auto texture = static_cast<ID3D11Texture2D*>(gs_texture_get_obj(src));

		// Pre-validate texture format
		D3D11_TEXTURE2D_DESC desc = { 0 };
		texture->GetDesc(&desc);
		LVK_ASSERT(cv::directx::getTypeFromDXGI_FORMAT(desc.Format) >= 0);

		cv::directx::convertFromD3D11Texture2D(texture, dst);

#else   // OpenGL Interop
		
		// Pre-validate texture format
		const auto format = gs_texture_get_color_format(src);
		LVK_ASSERT(format == GS_RGBA || format == GS_RGBA_UNORM);

		cv::ogl::Texture2D texture(
			gs_texture_get_height(src),
			gs_texture_get_width(src),
			cv::ogl::Texture2D::RGBA,
			*static_cast<uint32_t*>(gs_texture_get_obj(src)),
			false
		);

		cv::ogl::convertFromGLTexture2D(texture, dst);
#endif
	}

//---------------------------------------------------------------------------------------------------------------------
	
	void InteropContext::Export(cv::UMat& src, gs_texture_t* dst)
	{
		LVK_ASSERT(Available());
		LVK_ASSERT(dst != nullptr);
		LVK_ASSERT(src.cols == static_cast<int>(gs_texture_get_width(dst)));
		LVK_ASSERT(src.rows == static_cast<int>(gs_texture_get_height(dst)));
        LVK_PROFILE;

        // Attach the context if it is detached from the thread.
        if(Available() && !Attached())
            TryAttach();

#ifdef _WIN32 // DirectX11 Interop
		
		auto texture = static_cast<ID3D11Texture2D*>(gs_texture_get_obj(dst));

		// Pre-validate texture format
		D3D11_TEXTURE2D_DESC desc = {0};
		texture->GetDesc(&desc);
		LVK_ASSERT(src.type() == cv::directx::getTypeFromDXGI_FORMAT(desc.Format));

		cv::directx::convertToD3D11Texture2D(src, texture);

#else	// OpenGL Interop
		
		// Pre-validate texture format
		const auto format = gs_texture_get_color_format(dst);
		LVK_ASSERT(format == GS_RGBA || format == GS_RGBA_UNORM);

		cv::ogl::Texture2D texture(
			gs_texture_get_height(dst),
			gs_texture_get_width(dst),
			cv::ogl::Texture2D::RGBA,
			*static_cast<uint32_t*>(gs_texture_get_obj(dst)),
			false
		);

		cv::ogl::convertToGLTexture2D(src, texture);
#endif
	}

//---------------------------------------------------------------------------------------------------------------------

}
