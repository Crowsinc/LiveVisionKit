//     *************************** LiveVisionKit ****************************
//     Copyright (C) 2022  Sebastian Di Marco (crowsinc.dev@gmail.com)
//
//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the GNU General Public License
//     along with this program.  If not, see <https://www.gnu.org/licenses/>.
//     **********************************************************************

#pragma once

#include <opencv2/opencv.hpp>

namespace lvk
{

    template<typename T>
    T to_degrees(const T& radians);

    template<typename T>
    T to_radians(const T& degrees);

    template<typename T>
    T angle_of(const cv::Point_<T>& v, const cv::Point_<T>& ref = {1, 0});


    template<typename T>
    T round_multiple(const T& value, const T& base);

    template<typename T>
	T round_even(const T& value);


    template<typename T, typename V>
    T ratio_of(const V& numerator, const V& denominator);


    size_t index_2d(size_t x, size_t y, size_t row_length);

    cv::Point_<size_t> inv_index_2d(size_t index, size_t row_length);


	template<typename T>
	int sign(const T& value, const T& origin = T());

	template<typename T>
	int sign_2d(const cv::Point_<T>& point, const cv::Point_<T>& l1, const cv::Point_<T>& l2);


	template<typename V, typename T>
	inline V lerp(const V& current, const V& target, const T& amount);

	template<typename V, typename T>
	inline V step(const V& current, const V& target, const T& amount);


    template<typename T>
    bool between_01(const T& value);

    template<typename T>
    bool between_01_strict(const T& value);

    template<typename T>
	bool between(const T& value, const T& min, const T& max);

	template<typename T>
	bool between_strict(const T& value, const T& min, const T& max);


    template<typename T>
    bool within(const T& value, const T& target, const T& tolerance);

    template<typename T>
    bool within_strict(const T& value, const T& target, const T& tolerance);


	template<typename T>
	T exp_moving_average(const T average, const T new_sample, const float smoothing_factor);

    template<typename T>
    T moving_median(const T median, const T new_sample, const float learning_rate);


    template<typename T>
    cv::Rect_<T> crop(const cv::Size_<T>& region, const float x_proportion, const float y_proportion);

    template<typename T>
    cv::Rect_<T> crop(const cv::Size_<T>& region, const float proportion);

}

#include "Math.tpp"
