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

#include "../Math/Transform.hpp"

#include "../Diagnostics/Assert.hpp"
#include "../Math/Math.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	Transform Transform::FromAffine2D(const cv::Mat& affine)
	{
		LVK_ASSERT(affine.cols == 3 && affine.rows == 2 && affine.type() == CV_64FC1);

		const auto tx = affine.at<double>(0, 2);
		const auto ty = affine.at<double>(1, 2);
		const auto scaled_cos = affine.at<double>(0, 0);
		const auto scaled_sin = affine.at<double>(1, 0);

		// Decompose transform from 2x3 affine matrix of rotation, uniform scaling and translation
		const cv::Point2d translation(tx, ty);
		const double rotation = std::atan2(scaled_sin, scaled_cos);
		const double scale = sign(scaled_cos) * std::sqrt(scaled_cos * scaled_cos + scaled_sin * scaled_sin);

		return Transform(translation, rotation, scale);
	}

//---------------------------------------------------------------------------------------------------------------------

	Transform Transform::Identity()
	{
		// Identity transform is just a transform which doesn't transform anything.
		// This function exists to attach an informative name to it.
		return Transform({0.0, 0.0}, 0.0, 1.0);
	}

//---------------------------------------------------------------------------------------------------------------------

	Transform Transform::Zero()
	{
		// Zero transform is just a transform which is all zero.
		// This function exists to attach an informative name to it.
		return Transform({0.0, 0.0}, 0.0, 0.0);
	}

//---------------------------------------------------------------------------------------------------------------------

	Transform::Transform(const cv::Point2d& translation, const double rotation, const double scale)
		: translation(translation),
		  rotation(rotation),
		  scale(scale) {}

//---------------------------------------------------------------------------------------------------------------------

	cv::Point2d Transform::apply(const cv::Point2d& point) const
	{
		const double cos = scale * std::cos(rotation);
		const double sin = scale * std::sin(rotation);
		return {
			point.x * cos + point.y * -sin + translation.x,
			point.x * sin + point.y *  cos + translation.y
		};
	}

//---------------------------------------------------------------------------------------------------------------------

	Transform Transform::apply(const Transform& transform) const
	{
		return {translation + transform.translation, rotation + transform.rotation, scale * transform.scale};
	}

//---------------------------------------------------------------------------------------------------------------------

	cv::Mat Transform::as_matrix() const
	{
		const double cos = scale * std::cos(rotation);
		const double sin = scale * std::sin(rotation);

		return cv::Mat({2, 3}, {cos, -sin, translation.x, sin, cos, translation.y});
	}


//---------------------------------------------------------------------------------------------------------------------

	void Transform::operator+=(const Transform& other)
	{
		translation += other.translation;
		rotation += other.rotation;
		scale += other.scale;
	}

//---------------------------------------------------------------------------------------------------------------------

	void Transform::operator-=(const Transform& other)
	{
		translation -= other.translation;
		rotation -= other.rotation;
		scale -= other.scale;
	}

//---------------------------------------------------------------------------------------------------------------------

	void Transform::operator*=(const double scaling)
	{
		translation *= scaling;
		rotation *= scaling;
		scale *= scaling;
	}

//---------------------------------------------------------------------------------------------------------------------

	void Transform::operator/=(const double scaling)
	{
		LVK_ASSERT(scaling != 0.0);

		translation /= scaling;
		rotation /= scaling;
		scale /= scaling;
	}

//---------------------------------------------------------------------------------------------------------------------

	Transform operator+(const Transform& left, const Transform& right)
	{
		return {left.translation + right.translation, left.rotation + right.rotation, left.scale + right.scale};
	}

//---------------------------------------------------------------------------------------------------------------------

	Transform operator-(const Transform& left, const Transform& right)
	{
		return {left.translation - right.translation, left.rotation - right.rotation, left.scale - right.scale};
	}

//---------------------------------------------------------------------------------------------------------------------

	Transform operator*(const Transform& transform, const double scaling)
	{
		return {transform.translation * scaling, transform.rotation * scaling, transform.scale * scaling};
	}

//---------------------------------------------------------------------------------------------------------------------

	Transform operator/(const Transform& transform, const double scaling)
	{
		LVK_ASSERT(scaling != 0.0);

		return {transform.translation / scaling, transform.rotation / scaling, transform.scale / scaling};
	}

//---------------------------------------------------------------------------------------------------------------------

	Transform operator*(const double scaling, const Transform& transform)
	{
		return transform * scaling;
	}

//---------------------------------------------------------------------------------------------------------------------

	Transform operator/(const double scaling, const Transform& transform)
	{
		return transform / scaling;
	}

//---------------------------------------------------------------------------------------------------------------------

}
