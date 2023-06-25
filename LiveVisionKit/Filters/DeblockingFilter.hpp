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

#include "VideoFilter.hpp"
#include "Utility/Configurable.hpp"

namespace lvk
{

	struct DeblockingFilterSettings
	{
		uint32_t detection_levels = 3; // Must be greater than 0
		uint32_t block_size = 16; // Must be greater than 0
		uint32_t filter_size = 5; // Must be odd
		float filter_scaling = 4; // Smaller is stronger (1/x)
	};

	class DeblockingFilter final : public VideoFilter, public Configurable<DeblockingFilterSettings>
	{
	public:

		explicit DeblockingFilter(DeblockingFilterSettings settings = {});
		
		void configure(const DeblockingFilterSettings& settings) override;

	private:

        void filter(VideoFrame&& input, VideoFrame& output) override;

		VideoFrame m_SmoothFrame, m_DetectionFrame, m_ReferenceFrame;
		cv::UMat m_BlockMask{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
		cv::UMat m_KeepBlendMap{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
		cv::UMat m_DeblockBlendMap{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
		cv::UMat m_BlockGrid{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
		cv::UMat m_DeblockBuffer{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
		cv::UMat m_FloatBuffer{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
	};

}