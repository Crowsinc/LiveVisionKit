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

namespace lvk
{

	//TODO: deal with mat format changes

	class DeblockingFilter : public VideoFilter
	{
	public:

		struct Settings
		{
			uint32_t detection_levels = 3; // Must be greater than 0
			uint32_t block_size = 16; // Must be greater than 0
			uint32_t filter_size = 5; // Must be odd
			float filter_scaling = 4; // Smaller is stronger (1/x)
		};

	public:

		DeblockingFilter(DeblockingFilter::Settings settings = {}, const uint32_t timing_samples = 60);

		void configure(const DeblockingFilter::Settings& new_settings);

		const DeblockingFilter::Settings& settings() const;

	private:

		virtual void filter(cv::UMat& frame, const bool debug) override;

		virtual void write_log(Logger& logger) override;

	private:

		DeblockingFilter::Settings m_Settings;

		cv::UMat m_SmoothFrame{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
		cv::UMat m_DetectionFrame{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
		cv::UMat m_ReferenceFrame{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
		cv::UMat m_BlockMask{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
		cv::UMat m_KeepBlendMap{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
		cv::UMat m_DeblockBlendMap{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
		cv::UMat m_BlockGrid{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
		cv::UMat m_DeblockBuffer{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY}; 
		cv::UMat m_FloatBuffer{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
	};

}