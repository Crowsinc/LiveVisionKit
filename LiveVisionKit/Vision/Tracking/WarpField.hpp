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

#pragma once

#include <opencv2/opencv.hpp>

#include "Math/Homography.hpp"

namespace lvk
{

    class WarpField
    {
    public:

        inline static const cv::Size MinimumSize = {2,2};

        static std::optional<WarpField> Estimate(
            const cv::Size& field_size,
            const cv::Rect& field_region,
            const std::vector<cv::Point2f>& tracked_points,
            const std::vector<cv::Point2f>& matched_points,
            std::vector<uint8_t>& inlier_status,
            bool map_inverse = false
        );


        explicit WarpField(const cv::Size& size);

        explicit WarpField(cv::Mat&& velocity_field);

        explicit WarpField(const cv::Mat& velocity_field);

        WarpField(const Homography& warp, const cv::Size& size, const cv::Size2f& scale);

        WarpField(WarpField&& other) noexcept;

        WarpField(const WarpField& other);


        void resize(const cv::Size& new_size);

        cv::Size size() const;

        int cols() const;

        int rows() const;


        const cv::Mat& data() const;


        void set_identity();

        void set_to(const Homography& warp, const cv::Size2f& scale);


        // TODO: add more linear and non-linear transformation operations.

        void translate_by(const cv::Vec2f& amount);


        cv::Point2f sample(const cv::Point2f& position) const;

        cv::Point2f trace(const cv::Point2f& position) const;


        void warp(const cv::UMat& src, cv::UMat& dst) const;

        // TODO: add a constrain function here.


        WarpField& operator=(WarpField&& other) noexcept;

        WarpField& operator=(const WarpField& other);


        void operator+=(const WarpField& other);

        void operator-=(const WarpField& other);


        void operator*=(const float scaling);

        void operator/=(const float scaling);


        void operator*=(const cv::Vec2f& scaling);

        void operator/=(const cv::Vec2f& scaling);

    private:

        static const cv::UMat view_identity_field(const cv::Size& resolution);

    private:
        cv::Mat m_VelocityField;
    };

    WarpField operator+(const WarpField& left, const WarpField& right);

    WarpField operator-(const WarpField& left, const WarpField& right);


    WarpField operator*(const WarpField& field, const float scaling);

    WarpField operator*(const float scaling, const WarpField& field);

    WarpField operator*(const WarpField& field, const cv::Vec2f& scaling);

    WarpField operator*(const cv::Vec2f& scaling, const WarpField& field);


    WarpField operator/(const WarpField& field, const float scaling);

    WarpField operator/(const float scaling, const WarpField& field);

    WarpField operator/(const WarpField& field, const cv::Vec2f& scaling);

    WarpField operator/(const cv::Vec2f& scaling, const WarpField& field);

}
