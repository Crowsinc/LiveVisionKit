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

namespace lvk
{
	template<typename T>
	T round_even(const T& value);


	template<typename T>
	int sign(const T& value, const T& reference = T());

	// 2D Line based sign operation
	template<typename T>
	int sign_2d(const cv::Point_<T>& p, const cv::Point_<T>& l1 = {0,0}, const cv::Point_<T>& l2 = {0,1});


	template<typename V, typename T>
	inline V lerp(const V& from, const V& to, const T& t);

	// Applies percentage crop to the given size
	template<typename T>
	cv::Rect_<T> crop(const cv::Size_<T>& region, const double proportion);

	template<typename T>
	bool is_between(const T value, const T min, const T max);

}

#include "../Math/Math.tpp"
