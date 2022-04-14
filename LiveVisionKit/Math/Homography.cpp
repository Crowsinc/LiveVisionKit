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

#include "Homography.hpp"

#include "Diagnostics/Directives.hpp"
#include "Math.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	Homography Homography::Identity()
	{
		// NOTE: A default-initialised homography is identity
		return {};
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography Homography::Zero()
	{
		return Homography::FromMatrix(cv::Mat::zeros(3, 3, CV_64FC1));
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography Homography::FromMatrix(const cv::Mat& matrix)
	{
		if(matrix.cols == 3 && matrix.rows == 2)
			return FromAffineMatrix(matrix);
		else
			return {matrix};
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography Homography::FromAffineMatrix(const cv::Mat& affine)
	{
		LVK_ASSERT(affine.cols == 3);
		LVK_ASSERT(affine.rows == 2);
		LVK_ASSERT(affine.type() == CV_64FC1);

		// Copy affine data over
		Homography h = Homography::Identity();
		for(int r = 0; r < 2; r++)
			for(int c = 0; c < 3; c++)
				h.m_Matrix.at<double>(r, c) = affine.at<double>(r, c);

		return h;
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography::Homography()
		: m_Matrix(cv::Mat::eye(3, 3, CV_64FC1))
	{}

//---------------------------------------------------------------------------------------------------------------------

	Homography::Homography(const cv::Mat& matrix)
		: m_Matrix(matrix.clone())
	{
		LVK_ASSERT(matrix.cols == 3);
		LVK_ASSERT(matrix.rows == 3);
		LVK_ASSERT(matrix.type() == CV_64FC1);
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography::Homography(const Homography& copy)
		: Homography(copy.m_Matrix)
	{}

//---------------------------------------------------------------------------------------------------------------------

	cv::Point2d Homography::transform(const cv::Point2d& point) const
	{
		std::vector<cv::Point2d> out, in = {point};
		cv::perspectiveTransform(in, out, m_Matrix);
		return out[0];
	}

//---------------------------------------------------------------------------------------------------------------------

	std::vector<cv::Point2d> Homography::transform(const std::vector<cv::Point2d>& points) const
	{
		std::vector<cv::Point2d> out;
		out.reserve(points.size());

		cv::perspectiveTransform(points, out, m_Matrix);

		return out;
	}

//---------------------------------------------------------------------------------------------------------------------

	void Homography::warp(const cv::UMat& src, cv::UMat& dst) const
	{
		if(is_affine())
			cv::warpAffine(src, dst, m_Matrix.rowRange(0, 2), src.size());
		else
			cv::warpPerspective(src, dst, m_Matrix, src.size());
	}

//---------------------------------------------------------------------------------------------------------------------

	cv::Mat Homography::as_matrix() const
	{
		return m_Matrix.clone();
	}

//---------------------------------------------------------------------------------------------------------------------

	bool Homography::is_affine() const
	{
		// We consider the homography affine if the bottom row is unchanged from identity
		return m_Matrix.at<double>(2, 0) == 0
			&& m_Matrix.at<double>(2, 1) == 0
			&& m_Matrix.at<double>(2, 2) == 1;
	}

//---------------------------------------------------------------------------------------------------------------------

	void Homography::operator+=(const Homography& other)
	{
		cv::add(m_Matrix, other.m_Matrix, m_Matrix);
	}

//---------------------------------------------------------------------------------------------------------------------

	void Homography::operator-=(const Homography& other)
	{
		cv::subtract(m_Matrix, other.m_Matrix, m_Matrix);
	}

//---------------------------------------------------------------------------------------------------------------------

	void Homography::operator*=(const Homography& other)
	{
		cv::multiply(m_Matrix, other.m_Matrix, m_Matrix);
	}

//---------------------------------------------------------------------------------------------------------------------

	void Homography::operator*=(const double scaling)
	{
		m_Matrix *= scaling;
	}

//---------------------------------------------------------------------------------------------------------------------

	void Homography::operator/=(const double scaling)
	{
		LVK_ASSERT(scaling != 0.0);

		m_Matrix /= scaling;
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography operator+(const Homography& left, const Homography& right)
	{
		Homography h(left);
		h += right;
		return h;
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography operator-(const Homography& left, const Homography& right)
	{
		Homography h(left);
		h -= right;
		return h;
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography operator*(const Homography& left, const Homography& right)
	{
		Homography h(left);
		h *= right;
		return h;
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography operator*(const Homography& homography, const double scaling)
	{
		Homography h(homography);
		h *= scaling;
		return h;
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography operator/(const Homography& homography, const double scaling)
	{
		LVK_ASSERT(scaling != 0.0);

		Homography h(homography);
		h /= scaling;
		return h;
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography operator*(const double scaling, const Homography& homography)
	{
		return homography * scaling;
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography operator/(const double scaling, const Homography& homography)
	{
		return homography / scaling;
	}

//---------------------------------------------------------------------------------------------------------------------

}
