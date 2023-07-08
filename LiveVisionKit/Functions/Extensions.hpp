//     *************************** LiveVisionKit ****************************
//     Copyright (C) 2022  Sebastian Di Marco (crowsinc.dev@gmail.com)
//
//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the GNU General Public License
//     along with this program.  If not, see <https://www.gnu.org/licenses/>.
//     **********************************************************************

#include <opencv2/opencv.hpp>

namespace lvk
{
    // cv::Size2f Operators
    cv::Size2f operator*(const cv::Size2f& v1, const cv::Size2f& v2);

    cv::Size2f operator/(const cv::Size2f& v1, const cv::Size2f& v2);

    cv::Size2f operator/(const float v1, const cv::Size2f& v2);

    cv::Size2f operator+(const cv::Size2f& v1, const float v2);

    cv::Size2f operator-(const cv::Size2f& v1, const float v2);

    // cv::Size Operators
    cv::Size operator*(const cv::Size& v1, const cv::Size& v2);

    cv::Size operator/(const cv::Size& v1, const cv::Size& v2);

    cv::Size operator/(const int v1, const cv::Size& v2);

    cv::Size operator+(const cv::Size& v1, const int v2);

    cv::Size operator-(const cv::Size& v1, const int v2);


    // cv::Point2f Operators
    cv::Point2f operator*(const cv::Point2f& p, const cv::Size2f& s);

    cv::Point2f operator/(const cv::Point2f& p, const cv::Size2f& s);

    cv::Point2f operator+(const cv::Point2f& p, const cv::Size2f& s);

    cv::Point2f operator-(const cv::Point2f& p, const cv::Size2f& s);

    cv::Point2f operator*(const cv::Point2f& p, const cv::Scalar& s);

    cv::Point2f operator/(const cv::Point2f& p, const cv::Scalar& s);

    cv::Point2f operator+(const cv::Point2f& p, const cv::Scalar& s);

    cv::Point2f operator-(const cv::Point2f& p, const cv::Scalar& s);


    // cv::Point Operators
    cv::Point operator*(const cv::Point& p, const cv::Size& s);

    cv::Point operator/(const cv::Point& p, const cv::Size& s);

    cv::Point operator+(const cv::Point& p, const cv::Size& s);

    cv::Point operator-(const cv::Point& p, const cv::Size& s);


    // cv::Scalar Operators
    cv::Scalar operator*(const cv::Scalar& v1, const cv::Scalar& v2);

    cv::Scalar operator/(const cv::Scalar& v1, const cv::Scalar& v2);

    cv::Scalar operator/(const cv::Scalar& v1, const double v2);

    cv::Scalar operator+(const cv::Scalar& v1, const double v2);

    cv::Scalar operator-(const cv::Scalar& v1, const double v2);

}