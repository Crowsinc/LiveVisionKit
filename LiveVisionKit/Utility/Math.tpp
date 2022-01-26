
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

		const auto new_tl = transform.apply(tl);
		const auto new_bl = transform.apply({tl.x, br.y});
		const auto new_br = transform.apply(br);
		const auto new_tr = transform.apply({br.x, tl.y});

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

		// NOTE: We can speed this up by projecting the the vertices of the AABB
		// along the normals of the transformed rect. Which brings the AABB to the
		// local coordinate system of the transformed rect, effectively making the
		// transformed rect become the AABB instead. We then just do a simple
		// point enclosure check. It requires significantly less computations but
		// introduces mathematical complexity which is likely unnecessary.


		const cv::Point2d tl = rect.tl();
		const cv::Point2d br = rect.br();

		const cv::Point2d rect_tl = transform.apply(tl);
		const cv::Point2d rect_bl = transform.apply({tl.x, br.y});
		const cv::Point2d rect_br = transform.apply(br);
		const cv::Point2d rect_tr = transform.apply({br.x, tl.y});

		const cv::Point2d aabb_tl = aabb.tl();
		const cv::Point2d aabb_br = aabb.br();
		const cv::Point2d aabb_tr(aabb_br.x, aabb_tl.y);
		const cv::Point2d aabb_bl(aabb_tl.x, aabb_br.y);

		// The AABB is enclosed if all of its points fall on the side of each edge of the rect.
		const bool all_right = sign_2d(aabb_tl, rect_tl, rect_tr) >= 0
							&& sign_2d(aabb_tl, rect_tr, rect_br) >= 0
							&& sign_2d(aabb_tl, rect_br, rect_bl) >= 0
							&& sign_2d(aabb_tl, rect_bl, rect_tl) >= 0
							&& sign_2d(aabb_tr, rect_tl, rect_tr) >= 0
							&& sign_2d(aabb_tr, rect_tr, rect_br) >= 0
							&& sign_2d(aabb_tr, rect_br, rect_bl) >= 0
							&& sign_2d(aabb_tr, rect_bl, rect_tl) >= 0
							&& sign_2d(aabb_br, rect_tl, rect_tr) >= 0
							&& sign_2d(aabb_br, rect_tr, rect_br) >= 0
							&& sign_2d(aabb_br, rect_br, rect_bl) >= 0
							&& sign_2d(aabb_br, rect_bl, rect_tl) >= 0
							&& sign_2d(aabb_bl, rect_tl, rect_tr) >= 0
							&& sign_2d(aabb_bl, rect_tr, rect_br) >= 0
							&& sign_2d(aabb_bl, rect_br, rect_bl) >= 0
							&& sign_2d(aabb_bl, rect_bl, rect_tl) >= 0;

		const bool all_left  = sign_2d(aabb_tl, rect_tl, rect_tr) <= 0
							&& sign_2d(aabb_tl, rect_tr, rect_br) <= 0
							&& sign_2d(aabb_tl, rect_br, rect_bl) <= 0
							&& sign_2d(aabb_tl, rect_bl, rect_tl) <= 0
							&& sign_2d(aabb_tr, rect_tl, rect_tr) <= 0
							&& sign_2d(aabb_tr, rect_tr, rect_br) <= 0
							&& sign_2d(aabb_tr, rect_br, rect_bl) <= 0
							&& sign_2d(aabb_tr, rect_bl, rect_tl) <= 0
							&& sign_2d(aabb_br, rect_tl, rect_tr) <= 0
							&& sign_2d(aabb_br, rect_tr, rect_br) <= 0
							&& sign_2d(aabb_br, rect_br, rect_bl) <= 0
							&& sign_2d(aabb_br, rect_bl, rect_tl) <= 0
							&& sign_2d(aabb_bl, rect_tl, rect_tr) <= 0
							&& sign_2d(aabb_bl, rect_tr, rect_br) <= 0
							&& sign_2d(aabb_bl, rect_br, rect_bl) <= 0
							&& sign_2d(aabb_bl, rect_bl, rect_tl) <= 0;


		return all_left || all_right;
	}

}
