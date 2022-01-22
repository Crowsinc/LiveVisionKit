#pragma once

#include <opencv2/opencv.hpp>

#include "../Vision/Transform.hpp"

namespace lvk
{

	template<typename T>
	int sign(const T value, const T reference = T());

	template<typename T>
	int sign_2d(const cv::Point_<T> p, const cv::Point_<T> l1 = {0,0}, const cv::Point_<T> l2 = {0,1});

	template<typename V, typename T>
	inline V lerp(const V from, const V to, const T t);

	template<typename T>
	bool intersects(const cv::Point_<T> l1, const cv::Point_<T> l2, const cv::Rect_<T>& rect);

	template<typename T>
	bool encloses(const cv::Rect_<T>& r1, const cv::Rect_<T>& r2);

	template<typename T>
	bool encloses(const cv::Rect_<T>& aabb, const cv::Rect_<T>& rect, const Transform& transform);

	template<typename T>
	bool encloses(const cv::Rect_<T>& rect, const Transform& transform, const cv::Rect_<T>& aabb);

}

#include "Math.tpp"
