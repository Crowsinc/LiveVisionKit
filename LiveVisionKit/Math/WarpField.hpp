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

#include <optional>
#include <functional>
#include <opencv2/opencv.hpp>

#include "Math/Homography.hpp"
#include "Functions/Drawing.hpp"

namespace lvk
{

    class WarpField
    {
    public:

        inline static const cv::Size MinimumSize = {2,2};


        explicit WarpField(const cv::Size& size);

        WarpField(const cv::Size& size, const cv::Point2f& motion);

        explicit WarpField(const cv::Mat& warp_offsets);

        explicit WarpField(cv::Mat&& warp_offsets);

        WarpField(WarpField&& other) noexcept;

        WarpField(const WarpField& other);

        
        WarpField(
            const cv::Size& size,
            const Homography& motion,
            const cv::Size2f& scale
        );

        WarpField(
            const cv::Size& size,
            const cv::Rect2f& described_region,
            const std::vector<cv::Point2f>& origin_points,
            const std::vector<cv::Point2f>& warped_points,
            const std::optional<Homography>& motion_hint
        );


        void resize(const cv::Size& new_size);

        cv::Size size() const;

        int cols() const;

        int rows() const;


        cv::Mat& offsets();

        const cv::Mat& offsets() const;


        cv::Point2f sample(const cv::Point& coord) const;

        cv::Point2f sample(const cv::Point2f& coord) const;

        cv::Point2f trace(const cv::Point2f& coord) const;


        void warp(const cv::UMat& src, cv::UMat& dst) const;

        void undistort(const float tolerance = 0.7f);


        void set_identity();

        void set_to(const cv::Point2f& motion);

        void set_to(const Homography& motion, const cv::Size2f& scale);

        void fit_points(
            const cv::Rect2f& described_region,
            const std::vector<cv::Point2f>& origin_points,
            const std::vector<cv::Point2f>& warped_points,
            const std::optional<Homography>& motion_hint
        );

        void clamp(const cv::Size2f& magnitude);

        void clamp(const cv::Size2f& min, const cv::Size2f& max);


        void combine(const WarpField& field, const float scaling = 1.0f);


        void blend(const float field_weight, const WarpField& field);

        void blend(const float weight_1, const float weight_2, const WarpField& field);


        void write(
            const std::function<void(cv::Point2f& offset, const cv::Point& coord)>& operation,
            const bool parallel = true
        );

        void read(
            const std::function<void(const cv::Point2f& offset, const cv::Point& coord)>& operation,
            const bool parallel = true
        ) const;


        void draw(cv::UMat& dst, const cv::Scalar& color = yuv::MAGENTA, const int thickness = 2) const;


        WarpField& operator=(WarpField&& other) noexcept;

        WarpField& operator=(const WarpField& other);


        void operator+=(const WarpField& other);

        void operator-=(const WarpField& other);

        void operator*=(const WarpField& other);


        void operator+=(const cv::Vec2f& motion);

        void operator-=(const cv::Vec2f& motion);

        void operator*=(const cv::Vec2f& scaling);

        void operator/=(const cv::Vec2f& scaling);


        void operator*=(const float scaling);

        void operator/=(const float scaling);

    private:

        static const cv::UMat view_identity_field(const cv::Size& resolution);

        static void accumulate_motions(
            cv::Mat& motion_field,
            const float motion_weight,
            const cv::Rect2f& alignment,
            const std::vector<cv::Point2f>& origin_points,
            const std::vector<cv::Point2f>& warped_points
        );

    private:
        // NOTE: The warp offsets define where warped points originate from.
        cv::Mat m_WarpOffsets;
    };

    WarpField operator+(const WarpField& left, const WarpField& right);

    WarpField operator-(const WarpField& left, const WarpField& right);

    WarpField operator*(const WarpField& left, const WarpField& right);


    WarpField operator+(const WarpField& left, const cv::Vec2f& right);

    WarpField operator-(const WarpField& left, const cv::Vec2f& right);

    WarpField operator*(const cv::Vec2f& scaling, const WarpField& field);

    WarpField operator*(const WarpField& field, const cv::Vec2f& scaling);

    WarpField operator/(const cv::Vec2f& scaling, const WarpField& field);

    WarpField operator/(const WarpField& field, const cv::Vec2f& scaling);


    WarpField operator*(const WarpField& field, const float scaling);

    WarpField operator*(const float scaling, const WarpField& field);

    WarpField operator/(const WarpField& field, const float scaling);

    WarpField operator/(const float scaling, const WarpField& field);

}
