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

#include "Homography.hpp"

namespace lvk
{

	class BoundingQuad
	{
	public:

		BoundingQuad(const cv::Size2d& size, const Homography& homography = Homography::Identity());

		void transform(const Homography& homography);

		bool encloses(const cv::Rect2d& rect) const;

		bool encloses(const BoundingQuad& quad) const;

		bool encloses(const cv::Point2d& point) const;

	private:
		std::vector<cv::Point2d> m_LocalVertices, m_Vertices;
	};

}
