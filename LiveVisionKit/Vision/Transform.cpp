#include "Transform.hpp"

namespace lvk
{

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

	void Transform::apply(const Transform& transform)
	{
		translation += transform.translation;
		rotation += transform.rotation;
		scale *= transform.scale;
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
