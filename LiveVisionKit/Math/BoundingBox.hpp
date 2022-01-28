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
