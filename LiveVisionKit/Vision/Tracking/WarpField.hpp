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
#include "Structures/SpatialMap.hpp"

namespace lvk
{

    class WarpField
    {
    public:

        inline static const cv::Size MinimumSize = {2,2};

        // TODO: add fit and set constructors, clean up function ordering

        explicit WarpField(const cv::Size& size);

        explicit WarpField(cv::Mat&& warp_motions);

        explicit WarpField(const cv::Mat& warp_motions);

        WarpField(WarpField&& other) noexcept;

        WarpField(const WarpField& other);


        void resize(const cv::Size& new_size);

        cv::Size size() const;

        int cols() const;

        int rows() const;


        cv::Mat& data();

        const cv::Mat& data() const;


        cv::Point2f sample(const cv::Point& position) const;

        cv::Point2f sample(const cv::Point2f& position) const;

        cv::Point2f trace(const cv::Point2f& position) const;


        void set_identity();

        void set_to(const Homography& warp, const cv::Size2f& scale);

        void fit_to(
            const cv::Rect2f& described_region,
            const std::vector<cv::Point2f>& origin_points,
            const std::vector<cv::Point2f>& warped_points,
            const std::optional<Homography>& motion_hint
        );


        // TODO: add more linear and non-linear transformation operations.

        void translate_by(const cv::Vec2f& amount);

        void clamp(const cv::Size2f& magnitude);

        void clamp(const cv::Size2f& min, const cv::Size2f& max);

        void merge_with(const WarpField& other, const float weight = 1.0f);

        void merge_with(const WarpField& other, const float weight_1, const float weight_2, const float offset = 0.0f);

        void modify(const std::function<void(cv::Point2f&, cv::Point)>& operation);


        void draw(cv::UMat& dst, const float motion_scaling) const;

        void warp(const cv::UMat& src, cv::UMat& dst, const bool smoothing = true) const;


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

        static void accumulate_motions(
            cv::Mat& motion_field,
            const float motion_weight,
            const cv::Rect2f& alignment,
            const std::vector<cv::Point2f>& origin_points,
            const std::vector<cv::Point2f>& warped_points
        );

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
