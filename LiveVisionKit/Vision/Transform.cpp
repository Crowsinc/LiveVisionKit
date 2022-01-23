#include "Transform.hpp"

#include "../Utility/Algorithm.hpp"
#include "../Utility/Math.hpp"

namespace lvk
{

	//-------------------------------------------------------------------------------------

	Transform Transform::FromAffine2D(const cv::Mat& affine)
	{
		//TODO: asserts

		const auto& tx = affine.at<double>(0, 2);
		const auto& ty = affine.at<double>(1, 2);
		const auto& scaled_cos = affine.at<double>(0, 0);
		const auto& scaled_sin = affine.at<double>(1, 0);

		// Decompose transform from 2x3 affine matrix of rotation, uniform scaling and translation
		const cv::Point2d translation(tx, ty);
		const double rotation = std::atan2(scaled_sin, scaled_cos);
		const double scale = sign(scaled_cos) * std::sqrt(scaled_cos * scaled_cos + scaled_sin * scaled_sin);

		return Transform(translation, rotation, scale);
	}

	//-------------------------------------------------------------------------------------

	Transform Transform::Identity()
	{
		// Identify transform is just a transform which doesn't do anything.
		// This function exists to attach an informative name to it.
		return Transform({0.0, 0.0}, 0.0, 1.0);
	}

	//-------------------------------------------------------------------------------------

	Transform::Transform(cv::Point2d translation, double rotation, double scale)
		: translation(translation),
		  rotation(rotation),
		  scale(scale) {}

	//-------------------------------------------------------------------------------------

	void Transform::operator+=(const Transform& other)
	{
		translation += other.translation;
		rotation += other.rotation;
		scale += other.scale;
	}

	//-------------------------------------------------------------------------------------

	void Transform::operator-=(const Transform& other)
	{
		translation -= other.translation;
		rotation -= other.rotation;
		scale -= other.scale;
	}

	//-------------------------------------------------------------------------------------

	void Transform::operator*=(const double scaling)
	{
		translation *= scaling;
		rotation *= scaling;
		scale *= scaling;
	}

	//-------------------------------------------------------------------------------------

	void Transform::operator/=(const double scaling)
	{
		translation /= scaling;
		rotation /= scaling;
		scale /= scaling;
	}

	//-------------------------------------------------------------------------------------

	cv::Point2d Transform::apply(const cv::Point2d& point) const
	{
		const double cos = scale * std::cos(rotation);
		const double sin = scale * std::sin(rotation);
		return {
			point.x * cos + point.y * -sin + translation.x,
			point.x * sin + point.y *  cos + translation.y
		};
	}

	//-------------------------------------------------------------------------------------

	Transform Transform::apply(const Transform& transform) const
	{
		return {translation + transform.translation, rotation + transform.rotation, scale * transform.scale};
	}

	//-------------------------------------------------------------------------------------

	cv::Mat Transform::as_matrix() const
	{
		const double cos = scale * std::cos(rotation);
		const double sin = scale * std::sin(rotation);

		return cv::Mat({2, 3}, {cos, -sin, translation.x, sin, cos, translation.y});
	}

	//-------------------------------------------------------------------------------------

	Transform operator+(const Transform& left, const Transform& right)
	{
		return {left.translation + right.translation, left.rotation + right.rotation, left.scale + right.scale};
	}

	//-------------------------------------------------------------------------------------

	Transform operator-(const Transform& left, const Transform& right)
	{
		return {left.translation - right.translation, left.rotation - right.rotation, left.scale - right.scale};
	}

	//-------------------------------------------------------------------------------------

	Transform operator*(const Transform& transform, const double scaling)
	{
		return {transform.translation * scaling, transform.rotation * scaling, transform.scale * scaling};
	}

	//-------------------------------------------------------------------------------------

	Transform operator/(const Transform& transform, const double scaling)
	{
		return {transform.translation / scaling, transform.rotation / scaling, transform.scale / scaling};
	}

	//-------------------------------------------------------------------------------------

	Transform operator*(const double scaling, const Transform& transform)
	{
		return transform * scaling;
	}

	//-------------------------------------------------------------------------------------

	Transform operator/(const double scaling, const Transform& transform)
	{
		return transform / scaling;
	}
	//-------------------------------------------------------------------------------------
}
