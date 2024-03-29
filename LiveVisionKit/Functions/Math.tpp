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
#include <numbers>

#include "Directives.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline T to_degrees(const T& radians)
    {
        return static_cast<T>(static_cast<double>(radians) * 180.0 / std::numbers::pi);
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline T to_radians(const T& degrees)
    {
        return static_cast<T>(static_cast<double>(degrees) * std::numbers::pi / 180.0);
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline T angle_of(const cv::Point_<T>& v, const cv::Point_<T>& ref)
    {
        return std::atan2(
            ref.x * v.y - ref.y  * v.x,
            ref.x * v.x + ref.y  * v.y
        );
    }

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
    inline T round_multiple(const T& value, const T& base)
	{
		return std::round<T>(value / base) * base;
	}

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline T round_even(const T& value)
    {
        return std::round<T>(value / 2) * 2;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T, typename V>
    inline T ratio_of(const V& numerator, const V& denominator)
    {
        return static_cast<T>(numerator) / static_cast<T>(denominator);
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline T index_2d(T x, T y, T row_length)
    {
        static_assert(std::is_integral_v<T>);
        LVK_ASSERT(row_length > 0);

        return y * row_length + x;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline cv::Point_<T> inv_index_2d(T index, T row_length)
    {
        static_assert(std::is_integral_v<T>);

        return cv::Point_<T>(
            index % row_length,
            index / row_length
        );
    }

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
    inline int sign(const T& value, const T& origin)
	{
		// Returns 0 if the value is equal to the origin, -1 if it is on its left, 1 if its on its right
		return (origin < value) - (value < origin);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
    inline int sign_2d(const cv::Point_<T>& point, const cv::Point_<T>& l1, const cv::Point_<T>& l2)
	{
		// Returns 0 if p is on the infinite line l1 to l2, -1 if it is on its left, 1 if its on its right
		return sign((l1.x - l2.x) * (point.y - l2.y) - (l1.y - l2.y) * (point.x - l2.x));
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename V, typename T>
    inline V lerp(const V& current, const V& target, const T& amount)
	{
        LVK_ASSERT(amount >= 0);

		return current + amount * (target - current);
	}
	
//---------------------------------------------------------------------------------------------------------------------

	template<typename V, typename T>
    inline V step(const V& current, const V& target, const T& amount)
	{
		LVK_ASSERT(amount >= 0);

		if(current > target)
			return std::max<V>(current - amount, target);
		else
			return std::min<V>(current + amount, target);
	}

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline bool between_01(const T& value)
    {
        return (value >= static_cast<T>(0)) && (value <= static_cast<T>(1));
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline bool between_01_strict(const T& value)
    {
        return (value > static_cast<T>(0)) && (value < static_cast<T>(1));
    }

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
    inline bool between(const T& value, const T& min, const T& max)
	{
		LVK_ASSERT(min <= max);

		return (value >= min) && (value <= max);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
    inline bool between_strict(const T& value, const T& min, const T& max)
	{
		LVK_ASSERT(min < max);

		return (value > min) && (value < max);
	}

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline bool within(const T& value, const T& target, const T& tolerance)
    {
        return (value >= target - tolerance) && (value <= target + tolerance);
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline bool within_strict(const T& value, const T& target, const T& tolerance)
    {
        return (value > target - tolerance) && (value < target + tolerance);
    }

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
    inline T exp_moving_average(const T average, const T new_sample, const float smoothing_factor)
	{
        LVK_ASSERT(smoothing_factor >= 0.0f);

        return average + smoothing_factor * (new_sample - average);
	}

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline T moving_median(const T median, const T new_sample, const float learning_rate)
    {
        LVK_ASSERT(learning_rate >= 0.0f);

        return median + learning_rate * sign(new_sample - median);
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline cv::Rect_<T> crop(const cv::Size_<T>& region, const cv::Size2f& proportions)
    {
        LVK_ASSERT_01(proportions.width);
        LVK_ASSERT_01(proportions.height);

        const T total_horz_crop = region.width * proportions.width;
        const T total_vert_crop = region.height * proportions.height;

        return cv::Rect_<T>(
            total_horz_crop / 2,
            total_vert_crop / 2,
            region.width - total_horz_crop,
            region.height - total_vert_crop
        );
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline cv::Rect_<T> crop(const cv::Size_<T>& region, const float proportion)
    {
        LVK_ASSERT_01(proportion);

        return crop(region, cv::Size2f{proportion, proportion});
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    cv::Scalar barycentric_rect(const cv::Rect_<T>& rect, const cv::Point_<T>& point)
    {
        const T inverse_area = T(1) / rect.area();
        const T x1 = rect.x, x2 = x1 + rect.width;
        const T y1 = rect.y, y2 = y1 + rect.height;

        const T rx1 = x2 - point.x;
        const T ry1 = y2 - point.y;
        const T rx2 = point.x - x1;
        const T ry2 = point.y - y1;

        return cv::Scalar(
            rx1 * ry1 * inverse_area, // TL
            rx1 * ry2 * inverse_area, // BL
            rx2 * ry2 * inverse_area, // BR
            rx2 * ry1 * inverse_area  // TR
        );
    }

//---------------------------------------------------------------------------------------------------------------------

}
