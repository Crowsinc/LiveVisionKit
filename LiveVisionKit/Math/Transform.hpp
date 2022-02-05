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

	struct Transform
	{
		cv::Point2d translation;
		double rotation; // radians
		double scale;


		static Transform FromAffine2D(const cv::Mat& affine);

		static Transform Identity();

		static Transform Zero();


		Transform(const cv::Point2d& translation, const double rotation, const double scale);

		cv::Point2d apply(const cv::Point2d& point) const;

		Transform apply(const Transform& transform) const;

		cv::Mat as_matrix() const;


		void operator+=(const Transform& other);

		void operator-=(const Transform& other);

		void operator*=(const double scaling);

		void operator/=(const double scaling);

	};

	Transform operator+(const Transform& left, const Transform& right);

	Transform operator-(const Transform& left, const Transform& right);

	Transform operator*(const Transform& transform, const double scaling);

	Transform operator/(const Transform& transform, const double scaling);

	Transform operator*(const double scaling, const Transform& transform);

	Transform operator/(const double scaling, const Transform& transform);

}
