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
#include "Data/VideoFrame.hpp"
#include "Functions/Drawing.hpp"

namespace lvk
{

    class WarpMesh
    {
    public:

        inline static const cv::Size MinimumSize = {2,2};


        WarpMesh(const cv::Size& size);

        WarpMesh(const WarpMesh& other);

        WarpMesh(WarpMesh&& other) noexcept;

        WarpMesh(cv::Mat&& warp_map, const bool as_offsets, const bool normalized);

        WarpMesh(const cv::Mat& warp_map, const bool as_offsets, const bool normalized);

        WarpMesh(const Homography& motion, const cv::Size2f& motion_scale, const cv::Size& size = MinimumSize);


        void resize(const cv::Size& new_size);

        cv::Size size() const;

        int cols() const;

        int rows() const;


        cv::Mat& offsets();

        const cv::Mat& offsets() const;

        operator cv::Mat&();

        operator const cv::Mat&() const;

        operator cv::_InputOutputArray();

        operator cv::_InputArray() const;


        void to_map(cv::Mat& dst) const;

        void to_map(cv::UMat& dst) const;

        void normalize(const cv::Size2f& motion_scale);


        void undistort(const float tolerance = 0.7f);

        void apply(const VideoFrame& src, VideoFrame& dst, const cv::Scalar& background = {0,0,0}) const;

        void draw(cv::UMat& dst, const cv::Scalar& color = yuv::MAGENTA, const int thickness = 2) const;


        void read(
            const std::function<void(const cv::Point2f& offset, const cv::Point& coord)>& operation,
            const bool parallel = true
        ) const;

        void write(
            const std::function<void(cv::Point2f& offset, const cv::Point& coord)>& operation,
            const bool parallel = true
        );


        void set_identity();

        void set_to(const cv::Point2f& motion);

        void set_to(const Homography& motion, const cv::Size2f& motion_scale);

        void set_to(cv::Mat&& warp_map, const bool as_offsets, const bool normalized);

        void set_to(const cv::Mat& warp_map, const bool as_offsets, const bool normalized);


        void scale(const cv::Size2f& scaling_factor);

        void crop_in(const cv::Rect2f& region);

        void rotate(const float degrees);


        void clamp(const cv::Size2f& magnitude);

        void clamp(const cv::Size2f& min, const cv::Size2f& max);


        void blend(const float mesh_weight, const WarpMesh& mesh);

        void blend(const float weight_1, const float weight_2, const WarpMesh& mesh);

        void combine(const WarpMesh& mesh, const float scaling = 1.0f);


        WarpMesh& operator=(WarpMesh&& other) noexcept;

        WarpMesh& operator=(const WarpMesh& other);


        void operator+=(const WarpMesh& other);

        void operator-=(const WarpMesh& other);

        void operator*=(const WarpMesh& other);


        void operator+=(const cv::Point2f& motion);

        void operator-=(const cv::Point2f& motion);

        void operator*=(const cv::Size2f& scaling);

        void operator/=(const cv::Size2f& scaling);


        void operator*=(const float scaling);

        void operator/=(const float scaling);

    private:

        static const cv::Mat view_identity_mesh(const cv::Size& resolution);

    private:
        // Offsets map mesh vertices from warped coord to identity coord.
        // e.g. Mesh Offsets = Warped Mesh - Identity Grid.
        cv::Mat m_MeshOffsets;

        mutable cv::UMat m_WarpMap{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
    };

    WarpMesh operator+(const WarpMesh& left, const WarpMesh& right);

    WarpMesh operator-(const WarpMesh& left, const WarpMesh& right);

    WarpMesh operator*(const WarpMesh& left, const WarpMesh& right);


    WarpMesh operator+(const WarpMesh& left, const cv::Point2f& right);

    WarpMesh operator-(const WarpMesh& left, const cv::Point2f& right);

    WarpMesh operator*(const cv::Size2f& scaling, const WarpMesh& mesh);

    WarpMesh operator*(const WarpMesh& mesh, const cv::Size2f& scaling);

    WarpMesh operator/(const cv::Size2f& scaling, const WarpMesh& mesh);

    WarpMesh operator/(const WarpMesh& mesh, const cv::Size2f& scaling);


    WarpMesh operator*(const WarpMesh& mesh, const float scaling);

    WarpMesh operator*(const float scaling, const WarpMesh& mesh);

    WarpMesh operator/(const WarpMesh& mesh, const float scaling);

    WarpMesh operator/(const float scaling, const WarpMesh& mesh);

}
