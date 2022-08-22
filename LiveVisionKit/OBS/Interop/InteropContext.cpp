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

#include "Diagnostics/Directives.hpp"
#include "OBS/Utility/Logging.hpp"

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
	graphics_t* InteropContext::s_GraphicsContext = nullptr;
	std::thread::id InteropContext::s_BoundThread;
	bool InteropContext::s_TestPassed = false;

//---------------------------------------------------------------------------------------------------------------------

	bool InteropContext::TryAttach()
	{
		LVK_ASSERT(gs_get_context() != nullptr);

		if(!Supported())
			return false;

		// Create the OCL interop context if it does not yet exist
		if(s_OCLContext.empty())
		{
#ifdef _WIN32 
			// DirectX11 Context
			auto device = static_cast<ID3D11Device*>(gs_get_device_obj());
			cv::directx::ocl::initializeContextFromD3D11Device(device);
#else       
			// OpenGL Context
			cv::ogl::ocl::initializeContextFromGL();
#endif

			s_OCLContext = cv::ocl::OpenCLExecutionContext::getCurrent();
			s_BoundThread = std::this_thread::get_id();
			s_GraphicsContext = gs_get_context();

			// Test the context as some (Linux) systems crash when using interop,
			// despite having support for creating the interop context.
			if (s_TestPassed = TestContext(); s_TestPassed)
				lvk::log::print("Interop support passed all validation tests");
			else
				lvk::log::error("Interop support failed to pass validation tests, and was disabled");
		}

		// NOTE: We are making the assumption that 
		// OBS only ever has one graphics context. 
		LVK_ASSERT(gs_get_context() == s_GraphicsContext);
		if (!Attached())
		{
			// If the context is not attached to the current thread, then bind it
			s_OCLContext.bind();
			s_BoundThread = std::this_thread::get_id();
			log::warn("The interop context was bound to a new graphics thead");
		}

		return true;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool InteropContext::TestContext()
	{
		LVK_ASSERT(gs_get_context() != nullptr);
		LVK_ASSERT(Attached());

		const float test_size = 64;

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

		bool exception = false;
		try
		{
			Export(cv_texture, obs_texture);
			Import(obs_texture, cv_texture);
		}
		catch (std::exception e)
		{
			exception = true;
		}
		
		gs_texture_destroy(obs_texture);
		
		return !exception;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool InteropContext::Supported()
	{
		if (!cv::ocl::haveOpenCL())
			return false;

		auto& device = cv::ocl::Device::getDefault();

#ifdef _WIN32
		// DirectX11 
		return device.isExtensionSupported("cl_nv_d3d11_sharing") 
			|| device.isExtensionSupported("cl_khr_d3d11_sharing");
#else
		// OpenGL
		return device.isExtensionSupported("cl_khr_gl_sharing");
#endif
	}

//---------------------------------------------------------------------------------------------------------------------

	bool InteropContext::Attached()
	{
		return !s_OCLContext.empty() && s_BoundThread == std::this_thread::get_id();
	}

//---------------------------------------------------------------------------------------------------------------------

	bool InteropContext::Available()
	{
		return !s_OCLContext.empty() && s_TestPassed;
	}

//---------------------------------------------------------------------------------------------------------------------

	void InteropContext::Import(gs_texture_t* src, cv::UMat& dst)
	{
		LVK_ASSERT(Attached());
		LVK_ASSERT(src != nullptr);

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
		LVK_ASSERT(Attached());
		LVK_ASSERT(dst != nullptr);
		LVK_ASSERT(src.cols == gs_texture_get_width(dst));
		LVK_ASSERT(src.rows == gs_texture_get_height(dst));

#ifdef _WIN32 // DirectX11 Interop
		
		auto texture = static_cast<ID3D11Texture2D*>(gs_texture_get_obj(dst));

		// Pre-validate texture format
		D3D11_TEXTURE2D_DESC desc = { 0 };
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
