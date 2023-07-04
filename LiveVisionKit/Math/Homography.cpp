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

#include "Directives.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	Homography Homography::Identity()
	{
		// NOTE: A default-initialised homography is identity
		return Homography();
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography Homography::Zero()
	{
        cv::Mat data = cv::Mat::zeros(3, 3, CV_64FC1);
		return WrapMatrix(data);
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography Homography::WrapMatrix(cv::Mat& matrix)
	{
        LVK_ASSERT(matrix.cols == 3);
        LVK_ASSERT(matrix.rows == 3);
        LVK_ASSERT(matrix.type() == CV_64FC1);

        // Use private matrix 'move' constructor
        return Homography(matrix);
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography Homography::FromAffineMatrix(const cv::Mat& affine)
	{
		LVK_ASSERT(affine.cols == 3);
		LVK_ASSERT(affine.rows == 2);
		LVK_ASSERT(affine.type() == CV_64FC1);

		// Copy affine data over to the internal 3x3 matrix
		Homography perspective;
		for(int r = 0; r < 2; r++)
			for(int c = 0; c < 3; c++)
                perspective.m_Matrix.at<double>(r, c) = affine.at<double>(r, c);

		return perspective;
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

    // This is a private matrix 'move' constructor.
    Homography::Homography(cv::Mat& data)
        : m_Matrix(data)
    {
        LVK_ASSERT(data.cols == 3);
        LVK_ASSERT(data.rows == 3);
        LVK_ASSERT(data.type() == CV_64FC1);
    }

//---------------------------------------------------------------------------------------------------------------------

	Homography::Homography(const Homography& other)
		: Homography(other.m_Matrix)
	{}

//---------------------------------------------------------------------------------------------------------------------

	Homography::Homography(Homography&& other) noexcept
		: m_Matrix(std::move(other.m_Matrix))
	{}

//---------------------------------------------------------------------------------------------------------------------

    void Homography::set_zero()
    {
        m_Matrix.setTo(cv::Scalar::all(0.0));
    }

//---------------------------------------------------------------------------------------------------------------------

    void Homography::set_identity()
    {
        set_zero();
        m_Matrix.at<double>(0,0) = 1.0;
        m_Matrix.at<double>(1,1) = 1.0;
        m_Matrix.at<double>(2,2) = 1.0;
    }

//---------------------------------------------------------------------------------------------------------------------

	cv::Point2d Homography::transform(const cv::Point2d& point) const
	{
		std::vector<cv::Point2d> out, in = {point};
		cv::perspectiveTransform(in, out, m_Matrix);
		return out[0];
	}

//---------------------------------------------------------------------------------------------------------------------

    cv::Point2f Homography::transform(const cv::Point2f& point) const
    {
        std::vector<cv::Point2f> out, in = {point};
        cv::perspectiveTransform(in, out, m_Matrix);
        return out[0];
    }

//---------------------------------------------------------------------------------------------------------------------

    cv::Point2d Homography::operator*(const cv::Point2d& point) const
    {
        return transform(point);
    }

//---------------------------------------------------------------------------------------------------------------------

    cv::Point2f Homography::operator*(const cv::Point2f& point) const
    {
        return transform(point);
    }

//---------------------------------------------------------------------------------------------------------------------

	void Homography::transform(const std::vector<cv::Point2d>& points, std::vector<cv::Point2d>& dst) const
	{
        cv::perspectiveTransform(points, dst, m_Matrix);
	}

//---------------------------------------------------------------------------------------------------------------------

    void Homography::transform(const std::vector<cv::Point2f>& points, std::vector<cv::Point2f>& dst) const
    {
        cv::perspectiveTransform(points, dst, m_Matrix);
    }

//---------------------------------------------------------------------------------------------------------------------

    std::vector<cv::Point2d> Homography::operator*(const std::vector<cv::Point2d>& points) const
    {
        std::vector<cv::Point2d> transformed_points;
        transformed_points.reserve(points.size());

        transform(points, transformed_points);

        return transformed_points;
    }

//---------------------------------------------------------------------------------------------------------------------

    std::vector<cv::Point2f> Homography::operator*(const std::vector<cv::Point2f>& points) const
    {
        std::vector<cv::Point2f> transformed_points;
        transformed_points.reserve(points.size());

        transform(points, transformed_points);

        return transformed_points;
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

	const cv::Mat& Homography::data() const
	{
		return m_Matrix;
	}

//---------------------------------------------------------------------------------------------------------------------

    Homography Homography::invert() const
    {
        cv::Mat result = m_Matrix.inv();
        return Homography(result);
    }

//---------------------------------------------------------------------------------------------------------------------

    bool Homography::is_identity() const
    {
        return m_Matrix.at<cv::Vec3d>(0,0) == cv::Vec3d(1.0, 0.0, 0.0)
            && m_Matrix.at<cv::Vec3d>(1,0) == cv::Vec3d(0.0, 1.0, 0.0)
            && m_Matrix.at<cv::Vec3d>(2,0) == cv::Vec3d(0.0, 0.0, 1.0);
    }

//---------------------------------------------------------------------------------------------------------------------

	bool Homography::is_affine() const
	{
		// We consider the homography affine if the bottom row is unchanged from identity
		return m_Matrix.at<cv::Vec3d>(2, 0) == cv::Vec3d(0.0, 0.0, 1.0);
	}

//---------------------------------------------------------------------------------------------------------------------

    bool Homography::is_zero() const
    {
        return m_Matrix.at<cv::Vec3d>(0,0) == cv::Vec3d(0.0, 0.0, 0.0)
            && m_Matrix.at<cv::Vec3d>(1,0) == cv::Vec3d(0.0, 0.0, 0.0)
            && m_Matrix.at<cv::Vec3d>(2,0) == cv::Vec3d(0.0, 0.0, 0.0);
    }

//---------------------------------------------------------------------------------------------------------------------

    Homography& Homography::operator=(const cv::Mat& other)
    {
        m_Matrix = other.clone();
        return *this;
    }

//---------------------------------------------------------------------------------------------------------------------

    Homography& Homography::operator=(const Homography& other)
	{
		m_Matrix = other.m_Matrix.clone();
        return *this;
	}

//---------------------------------------------------------------------------------------------------------------------

    Homography& Homography::operator=(Homography&& other) noexcept
	{
		m_Matrix = std::move(other.m_Matrix);
        return *this;
    }

//---------------------------------------------------------------------------------------------------------------------

	void Homography::operator+=(const Homography& other)
	{
		cv::add(m_Matrix, other.m_Matrix, m_Matrix);
	}

//---------------------------------------------------------------------------------------------------------------------

    void Homography::operator+=(const cv::Mat& other)
    {
        cv::add(m_Matrix, other, m_Matrix);
    }

//---------------------------------------------------------------------------------------------------------------------

	void Homography::operator-=(const Homography& other)
	{
		cv::subtract(m_Matrix, other.m_Matrix, m_Matrix);
	}

//---------------------------------------------------------------------------------------------------------------------

    void Homography::operator-=(const cv::Mat& other)
    {
        cv::subtract(m_Matrix, other, m_Matrix);
    }

//---------------------------------------------------------------------------------------------------------------------

	void Homography::operator*=(const Homography& other)
	{
        // This is matrix multiplication
        cv::gemm(m_Matrix, other.m_Matrix, 1, cv::Mat(), 0, m_Matrix, 0);
	}

//---------------------------------------------------------------------------------------------------------------------

    void Homography::operator*=(const cv::Mat& other)
    {
        // This is matrix multiplication
        cv::gemm(m_Matrix, other, 1, cv::Mat(), 0, m_Matrix, 0);
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
        cv::Mat result = left.data() + right.data();
		return Homography::WrapMatrix(result);
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography operator-(const Homography& left, const Homography& right)
	{
        cv::Mat result = left.data() - right.data();
        return Homography::WrapMatrix(result);
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography operator*(const Homography& left, const Homography& right)
	{
        // This is matrix multiplication
        cv::Mat result = left.data() * right.data();
        return Homography::WrapMatrix(result);
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography operator*(const Homography& homography, const double scaling)
	{
		cv::Mat result = homography.data() * scaling;
		return Homography::WrapMatrix(result);
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography operator/(const Homography& homography, const double scaling)
	{
		LVK_ASSERT(scaling != 0.0);

        cv::Mat result = homography.data() / scaling;
        return Homography::WrapMatrix(result);
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
