#pragma once

#include <opencv2/opencv.hpp>

namespace lvk
{

	struct Transform
	{
		cv::Point2d translation;
		double rotation; // Radians
		double scale;

		static Transform FromAffine2D(const cv::Mat& affine);

		static Transform Identity();

		static Transform Zero();

		Transform(cv::Point2d translation, double rotation, double scale);

		void operator+=(const Transform& other);

		void operator-=(const Transform& other);

		void operator*=(const double scaling);

		void operator/=(const double scaling);

		cv::Point2d apply(const cv::Point2d& point) const;

		Transform apply(const Transform& transform) const;

		cv::Mat as_matrix() const;

	};

	Transform operator+(const Transform& left, const Transform& right);

	Transform operator-(const Transform& left, const Transform& right);

	Transform operator*(const Transform& transform, const double scaling);

	Transform operator/(const Transform& transform, const double scaling);

	Transform operator*(const double scaling, const Transform& transform);

	Transform operator/(const double scaling, const Transform& transform);

}
