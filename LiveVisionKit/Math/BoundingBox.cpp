#include "../Math/BoundingBox.hpp"

namespace lvk
{

	//-------------------------------------------------------------------------------------

	BoundingBox::BoundingBox(const cv::Size2d& size, const Transform& transform)
		: m_Rect({0.0, 0.0}, size),
		  m_LocalExtent({0.0, 0.0}, size)
	{
		this->transform(transform);
	}

	//-------------------------------------------------------------------------------------

	void BoundingBox::transform(const Transform& transform)
	{
		const cv::Point2d tl = m_Rect.tl();
		const cv::Point2d br = m_Rect.br();

		m_TopLeft = transform.apply(tl);
		m_BottomRight = transform.apply(br);
		m_TopRight = transform.apply({br.x, tl.y});
		m_BottomLeft = transform.apply({tl.x, br.y});

		m_LocalExtent.width = m_Rect.width * transform.scale;
		m_LocalExtent.height = m_Rect.height * transform.scale;

		// NOTE: The Y normal is inverted to deal with OpenCV's inverted
		// Y coordinate system, as top-left is actually bottom-left.
		m_XUnitNormal = (m_BottomRight - m_BottomLeft) / m_LocalExtent.width;
		m_YUnitNormal = (m_BottomLeft - m_TopLeft) / m_LocalExtent.height;
	}

	//-------------------------------------------------------------------------------------

	bool BoundingBox::encloses(const cv::Rect2d& rect) const
	{
		const cv::Point2d tl = rect.tl();
		const cv::Point2d br = rect.br();
		const cv::Point2d tr(br.x, tl.y);
		const cv::Point2d bl(tl.x, br.y);

		return m_LocalExtent.contains(to_local_space(tl))
			&& m_LocalExtent.contains(to_local_space(tr))
			&& m_LocalExtent.contains(to_local_space(br))
			&& m_LocalExtent.contains(to_local_space(bl));
	}

	//-------------------------------------------------------------------------------------

	bool BoundingBox::encloses(const BoundingBox& box) const
	{
		return m_LocalExtent.contains(to_local_space(box.m_TopLeft))
			&& m_LocalExtent.contains(to_local_space(box.m_TopRight))
			&& m_LocalExtent.contains(to_local_space(box.m_BottomRight))
			&& m_LocalExtent.contains(to_local_space(box.m_BottomLeft));
	}

	//-------------------------------------------------------------------------------------

	cv::Point2d BoundingBox::to_local_space(cv::Point2d point) const
	{
		point -= m_TopLeft;
		return cv::Point2d(m_XUnitNormal.dot(point), m_YUnitNormal.dot(point));
	}

	//-------------------------------------------------------------------------------------

}
