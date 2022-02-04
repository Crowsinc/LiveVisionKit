#pragma once

#include <opencv2/opencv.hpp>

namespace lvk
{

	template<typename T>
	T round_even(const T value);

	template<typename T>
	int sign(const T value, const T reference = T());

	template<typename T>
	int sign_2d(const cv::Point_<T> p, const cv::Point_<T> l1 = {0,0}, const cv::Point_<T> l2 = {0,1});

	template<typename V, typename T>
	inline V lerp(const V from, const V to, const T t);

	template<typename T>
	cv::Rect_<T> crop(const cv::Size_<T>& region, const double proportion);

}

#include "../Math/Math.tpp"
