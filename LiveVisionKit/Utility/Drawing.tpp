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

namespace lvk::draw
{

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	void plot_markers(
		cv::UMat& dst,
		const std::vector<cv::Point_<T>>& markers,
		const cv::Scalar& color,
		const cv::MarkerTypes type,
		const int size,
		const int thickness
	)
	{
		thread_local cv::UMat gpu_draw_mask(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// NOTE: Individually drawing lots of points on a UMat is very inefficient.
		// Instead, draw the points to a mask and apply them in bulk to the UMat.

		thread_local cv::Mat draw_mask;
		draw_mask.create(dst.size(), CV_8UC1);
		draw_mask.setTo(cv::Scalar(0));

		for(const auto& point : markers)
			cv::drawMarker(draw_mask, point, color, type, size, thickness);

		draw_mask.copyTo(gpu_draw_mask);
		dst.setTo(color, gpu_draw_mask);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	void text(
		cv::UMat& dst,
		const std::string& text,
		const cv::Point_<T>& position,
		const cv::Scalar& color,
		const double font_scale,
		const int font_thickness,
		const cv::HersheyFonts font
	)
	{
		cv::putText(
			dst,
			text,
			position,
			font,
			font_scale,
			color,
			font_thickness
		);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	void rect(
		cv::UMat& dst,
		const cv::Rect_<T>& rect,
		const cv::Scalar& color,
		const int thickness
	)
	{
		cv::rectangle(dst, rect, color, thickness);
	}

//---------------------------------------------------------------------------------------------------------------------
}
