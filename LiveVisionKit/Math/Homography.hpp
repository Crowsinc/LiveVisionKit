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

	class Homography
	{
	public:

		static Homography Zero();

		static Homography Identity();

		static Homography FromMatrix(const cv::Mat& matrix);

		static Homography FromAffineMatrix(const cv::Mat& affine);

		Homography();

		Homography(const cv::Mat& matrix);

		Homography(const Homography& other);

		Homography(Homography&& other);

		cv::Point2d transform(const cv::Point2d& point) const;

		cv::Point2f transform(const cv::Point2f& point) const;

		std::vector<cv::Point2d> transform(const std::vector<cv::Point2d>& points) const;
		
		std::vector<cv::Point2f> transform(const std::vector<cv::Point2f>& points) const;

		void warp(const cv::UMat& src, cv::UMat& dst) const;

		bool is_affine() const;

		cv::Mat as_matrix() const;

		void operator=(const Homography& other);

		void operator=(Homography&& other);

		void operator+=(const Homography& other);

		void operator-=(const Homography& other);

		void operator*=(const Homography& other);

		void operator*=(const double scaling);

		void operator/=(const double scaling);

	private:
		cv::Mat m_Matrix;
	};

	Homography operator+(const Homography& left, const Homography& right);

	Homography operator-(const Homography& left, const Homography& right);

	Homography operator*(const Homography& left, const Homography& right);

	Homography operator*(const Homography& homography, const double scaling);

	Homography operator/(const Homography& homography, const double scaling);

	Homography operator*(const double scaling, const Homography& homography);

	Homography operator/(const double scaling, const Homography& homography);

}
