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

#include <opencv2/opencv.hpp>

#include "../Math/Transform.hpp"

namespace lvk
{

	class BoundingBox
	{
	public:

		BoundingBox(const cv::Size2d& size, const Transform& transform = Transform::Identity());

		void transform(const Transform& transform);

		bool encloses(const cv::Rect2d& rect) const;

		bool encloses(const BoundingBox& box) const;

	private:

		cv::Rect2d m_Rect;
		cv::Rect2d m_LocalExtent;
		cv::Point2d m_TopLeft, m_TopRight;
		cv::Point2d m_BottomLeft, m_BottomRight;
		cv::Point2d m_XUnitNormal, m_YUnitNormal;


		cv::Point2d to_local_space(cv::Point2d point) const;

	};

}
