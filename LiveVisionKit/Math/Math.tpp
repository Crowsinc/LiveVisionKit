
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
	cv::Rect_<T> crop(const cv::Size_<T>& region, const double proportion)
	{
		const T total_horz_crop = region.width * proportion;
		const T total_vert_crop = region.height * proportion;

		return cv::Rect_<T>(
				total_horz_crop / 2,
				total_vert_crop / 2,
				region.width - total_horz_crop,
				region.height - total_vert_crop
		);
	}

	//-------------------------------------------------------------------------------------
}
