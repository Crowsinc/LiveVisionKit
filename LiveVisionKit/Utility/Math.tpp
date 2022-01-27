
#include <opencv2/opencv.hpp>

#include "../Diagnostics/Assert.hpp"

namespace lvk
{

	//-------------------------------------------------------------------------------------

	template<typename T>
	T round_even(const T value)
	{
		return std::round(value / 2) * 2;
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	int sign(const T value, const T reference)
	{
		// Returns 0 if the value is equal to the reference, -1 if it is on its left, 1 if its on its right
		return (reference < value) - (value < reference);
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	int sign_2d(const cv::Point_<T> p, const cv::Point_<T> l1, const cv::Point_<T> l2)
	{
		// Returns 0 if p is on the infinite line l1 to l2, -1 if it is on its left, 1 if its on its right
		return sign((l1.x - l2.x) * (p.y - l2.y) - (l1.y - l2.y) * (p.x - l2.x));
	}

	//-------------------------------------------------------------------------------------

	template<typename V, typename T>
	V lerp(const V from, const V to, const T t)
	{
		return from + t * (to - from);
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	bool intersects(const cv::Point_<T> l1, const cv::Point_<T> l2, const cv::Rect_<T>& rect)
	{
		// Checks if the infinite line l1 to l2 intersects the given rect
		const auto tl = rect.tl();
		const auto br = rect.br();
		const cv::Point_<T> tr(br.x, tl.y);
		const cv::Point_<T> bl(tl.x, br.y);

		// The l1-l2 intersects the rect if any vertices are on opposite sides of the line
		const auto s1 = sign_2d(tl, l1, l2);
		const auto s2 = sign_2d(bl, l1, l2);
		const auto s3 = sign_2d(br, l1, l2);
		const auto s4 = sign_2d(tr, l1, l2);

		return (s1 >= 0 || s2 >= 0 || s3 >= 0 || s4 >= 0) && (s1 <= 0 || s2 <= 0 || s3 <= 0 || s4 <= 0);
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	bool encloses(const cv::Rect_<T>& r1, const cv::Rect_<T>& r2)
	{
		// Checks if r1 encloses r2

		const auto tl = r2.tl();
		const auto br = r2.br();
		const cv::Point_<T> tr(br.x, tl.y);
		const cv::Point_<T> bl(tl.x, br.y);

		return r1.contains(tl)
			&& r1.contains(bl)
			&& r1.contains(br)
			&& r1.contains(tr);
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	bool encloses(const cv::Rect_<T>& aabb, const cv::Rect_<T>& rect, const Transform& transform)
	{
		// Checks if the AABB encloses the transformed rect

		const cv::Point2d tl = rect.tl();
		const cv::Point2d br = rect.br();

		const cv::Point2d new_tl = transform.apply(tl);
		const cv::Point2d new_bl = transform.apply({tl.x, br.y});
		const cv::Point2d new_br = transform.apply(br);
		const cv::Point2d new_tr = transform.apply({br.x, tl.y});

		return aabb.contains(new_tl)
			&& aabb.contains(new_bl)
			&& aabb.contains(new_br)
			&& aabb.contains(new_tr);
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	bool encloses(const cv::Rect_<T>& rect, const Transform& transform, const cv::Rect_<T>& aabb)
	{
		// Checks if the transformed rect encloses the AABB

		// NOTE: Rect is specified in inverted Y coordinate system, so the top left
		// point is actually the bottom left point, bottom right is top right etc.

		// Find the AABB vertices relative to the top left (actually bottom left) corner.
		// Project the relative vertices of the AABB along the normals of the transformed
		// rect to bring the AABB to its local coordinate system. Then perform a simple
		// point enclosure check. We are effectively swapping which rect is the AABB.

		const cv::Point2d rect_tl = transform.apply(rect.tl());
		const cv::Point2d rect_br = transform.apply(rect.br());
		const cv::Point2d rect_bl = transform.apply({rect.tl().x, rect.br().y});

		const cv::Rect2d local_rect({0.0, 0.0}, cv::Size2d(rect.size()) * transform.scale);

		const cv::Point2d unit_x = (rect_br - rect_bl) / local_rect.width;
		const cv::Point2d unit_y = (rect_bl - rect_tl) / local_rect.height;

		const cv::Point2d rel_aabb_tl = cv::Point2d(aabb.tl()) - rect_tl;
		const cv::Point2d rel_aabb_br = cv::Point2d(aabb.br()) - rect_tl;
		const cv::Point2d rel_aabb_tr(rel_aabb_br.x, rel_aabb_tl.y);
		const cv::Point2d rel_aabb_bl(rel_aabb_tl.x, rel_aabb_br.y);

		return local_rect.contains({unit_x.dot(rel_aabb_tl), unit_y.dot(rel_aabb_tl)})
			&& local_rect.contains({unit_x.dot(rel_aabb_br), unit_y.dot(rel_aabb_br)})
			&& local_rect.contains({unit_x.dot(rel_aabb_tr), unit_y.dot(rel_aabb_tr)})
			&& local_rect.contains({unit_x.dot(rel_aabb_bl), unit_y.dot(rel_aabb_bl)});
	}

}
