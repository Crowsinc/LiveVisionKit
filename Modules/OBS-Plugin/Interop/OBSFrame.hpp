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

#pragma once

#include <LiveVisionKit.hpp>
#include <opencv2/core.hpp>
#include <obs-module.h>

#include "FrameIngest.hpp"
#include "Utility/Graphics.hpp"

namespace lvk
{

	struct OBSFrame : public Frame
	{
        OBSFrame();

        OBSFrame(const OBSFrame& frame);

        OBSFrame(OBSFrame&& frame) noexcept;

        ~OBSFrame() override;


        OBSFrame& operator=(const OBSFrame& frame);

        OBSFrame& operator=(OBSFrame&& frame) noexcept;


        bool to_obs_frame(obs_source_frame* frame) const;

        bool from_obs_frame(const obs_source_frame* frame);


        void to_obs_texture(gs_texture_t* texture) const;

        void from_obs_texture(gs_texture_t* texture);

	private:

		void prepare_interop_texture(const int width, const int height) const;

	private:

		// Frame upload/download
		mutable std::unique_ptr<FrameIngest> m_FrameIngest;

		// Texture import/export
        mutable VideoFrame m_ExportBuffer;
        mutable gs_texture_t* m_ExportTexture = nullptr;
        mutable lvk::RGBTextureReadBuffer m_ReadBuffer;
        mutable lvk::RGBTextureWriteBuffer m_WriteBuffer;
	};

}
