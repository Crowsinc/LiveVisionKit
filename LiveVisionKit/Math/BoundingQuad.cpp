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

#include "BoundingQuad.hpp"

#include "Functions/Math.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	BoundingQuad::BoundingQuad(const cv::Size2d& size, const Homography& homography)
	{
		// NOTE: Must follow counter-clockwise ordering
		m_LocalVertices.emplace_back(0, 0);
		m_LocalVertices.emplace_back(size.width, 0);
		m_LocalVertices.emplace_back(size.width, size.height);
		m_LocalVertices.emplace_back(0, size.height);

		this->transform(homography);
	}

//---------------------------------------------------------------------------------------------------------------------

	void BoundingQuad::transform(const Homography& homography)
	{
		homography.transform(m_LocalVertices, m_Vertices);
	}

//---------------------------------------------------------------------------------------------------------------------

	bool BoundingQuad::encloses(const cv::Rect2d& rect) const
	{
		const cv::Point2d tl = rect.tl();
		const cv::Point2d br = rect.br();
		const cv::Point2d tr(br.x, tl.y);
		const cv::Point2d bl(tl.x, br.y);

		return encloses(tl)
			&& encloses(br)
			&& encloses(tr)
			&& encloses(bl);
	}

//---------------------------------------------------------------------------------------------------------------------

	bool BoundingQuad::encloses(const BoundingQuad& quad) const
	{
		return encloses(quad.m_Vertices[0])
			&& encloses(quad.m_Vertices[1])
			&& encloses(quad.m_Vertices[2])
			&& encloses(quad.m_Vertices[3]);
	}

//---------------------------------------------------------------------------------------------------------------------

	bool BoundingQuad::encloses(const cv::Point2d& point) const
	{
		// A point is enclosed within the quad if it is left of
		// every quad edge, going in a counter-clockwise order.
		return sign_2d(point, m_Vertices[0], m_Vertices[1]) <= 0
			&& sign_2d(point, m_Vertices[1], m_Vertices[2]) <= 0
			&& sign_2d(point, m_Vertices[2], m_Vertices[3]) <= 0
			&& sign_2d(point, m_Vertices[3], m_Vertices[0]) <= 0;
	}

//---------------------------------------------------------------------------------------------------------------------

}
