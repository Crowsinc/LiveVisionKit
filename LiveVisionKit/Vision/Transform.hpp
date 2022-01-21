#pragma once

#include <opencv2/core.hpp>

namespace lvk
{

	struct Transform
	{
		cv::Point2d translation = {0.0, 0.0};
		double rotation = 0.0; // Radians
		double scale = 1.0;

		void operator+=(const Transform& other);

		void operator-=(const Transform& other);

		void operator*=(const double scaling);

		void operator/=(const double scaling);

		void apply(const Transform& transform);

		cv::Mat as_matrix() const;

	};

	Transform operator+(const Transform& left, const Transform& right);

	Transform operator-(const Transform& left, const Transform& right);

	Transform operator*(const Transform& transform, const double scaling);

	Transform operator/(const Transform& transform, const double scaling);

	Transform operator*(const double scaling, const Transform& transform);

	Transform operator/(const double scaling, const Transform& transform);

}
