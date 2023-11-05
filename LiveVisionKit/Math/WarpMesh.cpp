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

#include "WarpMesh.hpp"

#include <opencv2/core/ocl.hpp>
#include <array>

#include "Functions/Extensions.hpp"
#include "Functions/Drawing.hpp"
#include "Functions/Image.hpp"
#include "Functions/Math.hpp"
#include "VirtualGrid.hpp"
#include "Directives.hpp"

namespace lvk
{
//---------------------------------------------------------------------------------------------------------------------

    WarpMesh::WarpMesh(const cv::Size& size)
        : m_MeshOffsets(size, CV_32FC2)
    {
        LVK_ASSERT(size.height >= MinimumSize.height);
        LVK_ASSERT(size.width >= MinimumSize.width);

        set_identity();
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh::WarpMesh(const WarpMesh& other)
        : m_MeshOffsets(other.m_MeshOffsets.clone())
    {}

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh::WarpMesh(WarpMesh&& other) noexcept
        : m_MeshOffsets(std::move(other.m_MeshOffsets))
    {}

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh::WarpMesh(cv::Mat&& warp_map, const bool as_offsets, const bool normalized)
    {
        set_to(std::move(warp_map), as_offsets, normalized);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh::WarpMesh(const cv::Mat& warp_map, const bool as_offsets, const bool normalized)
    {
        set_to(warp_map, as_offsets, normalized);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh::WarpMesh(const Homography& motion, const cv::Size2f& motion_scale, const cv::Size& size)
        : m_MeshOffsets(size, CV_32FC2)
    {
        set_to(motion, motion_scale);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::resize(const cv::Size& new_size)
    {
        LVK_ASSERT(new_size.height >= MinimumSize.height);
        LVK_ASSERT(new_size.width >= MinimumSize.width);

        if(m_MeshOffsets.size() == new_size)
            return;

        cv::Mat new_offsets;
        cv::resize(m_MeshOffsets, new_offsets, new_size, 0, 0, cv::INTER_LINEAR_EXACT);
        m_MeshOffsets = std::move(new_offsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    cv::Size WarpMesh::size() const
    {
        return m_MeshOffsets.size();
    }

//---------------------------------------------------------------------------------------------------------------------

    int WarpMesh::cols() const
    {
        return m_MeshOffsets.cols;
    }

//---------------------------------------------------------------------------------------------------------------------

    int WarpMesh::rows() const
    {
        return m_MeshOffsets.rows;
    }

//---------------------------------------------------------------------------------------------------------------------

    cv::Mat& WarpMesh::offsets()
    {
        return m_MeshOffsets;
    }

//---------------------------------------------------------------------------------------------------------------------

    const cv::Mat& WarpMesh::offsets() const
    {
        return m_MeshOffsets;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh::operator cv::Mat&()
    {
        return m_MeshOffsets;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh::operator const cv::Mat&() const
    {
        return m_MeshOffsets;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh::operator cv::_InputOutputArray()
    {
        return m_MeshOffsets;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh::operator cv::_InputArray() const
    {
        return m_MeshOffsets;
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::to_map(cv::Mat& dst) const
    {
        cv::add(m_MeshOffsets, view_identity_mesh(m_MeshOffsets.size()), dst);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::to_map(cv::UMat& dst) const
    {
        cv::add(m_MeshOffsets, view_identity_mesh(m_MeshOffsets.size()), dst);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::normalize(const cv::Size2f& motion_scale)
    {
        const cv::Scalar norm_factor(
            1.0f / motion_scale.width,
            1.0f / motion_scale.height
        );

        cv::multiply(m_MeshOffsets, norm_factor, m_MeshOffsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::apply(const VideoFrame& src, VideoFrame& dst, const cv::Scalar& background) const
    {
        const cv::Scalar motion_scaling(src.cols, src.rows);

        if(m_MeshOffsets.size() != MinimumSize)
        {
            // If our mesh is larger than 2x2 then scale it up and remap the input.
            cv::resize(m_MeshOffsets, m_WarpMap, src.size(), 0, 0, cv::INTER_LINEAR_EXACT);
            cv::multiply(m_WarpMap, motion_scaling, m_WarpMap);
            lvk::remap(src, dst, m_WarpMap, background);
        }
        else
        {
            // If our mesh is 2x2, then we can directly model it with a homography.
            const auto w = static_cast<float>(src.cols);
            const auto h = static_cast<float>(src.rows);
            const std::array<cv::Point2f, 4> destination = {
                    cv::Point2f(0, 0), cv::Point2f(w, 0),
                    cv::Point2f(0, h), cv::Point2f(w, h)
            };

            const std::array<cv::Point2f, 4> source = {
                destination[0] + m_MeshOffsets.at<cv::Point2f>(0, 0) * motion_scaling,
                destination[1] + m_MeshOffsets.at<cv::Point2f>(0, 1) * motion_scaling,
                destination[2] + m_MeshOffsets.at<cv::Point2f>(1, 0) * motion_scaling,
                destination[3] + m_MeshOffsets.at<cv::Point2f>(1, 1) * motion_scaling
            };

            cv::warpPerspective(
                src,
                dst,
                cv::getPerspectiveTransform(destination.data(), source.data()),
                src.size(),
                cv::WARP_INVERSE_MAP | cv::INTER_LINEAR,
                cv::BORDER_CONSTANT,
                background
            );
        }

        // Update metadata.
        dst.timestamp = src.timestamp;
        dst.format = src.format;
    }

//---------------------------------------------------------------------------------------------------------------------

    // TODO: optimize this
    void WarpMesh::draw(cv::UMat& dst, const cv::Scalar& color, const int thickness) const
    {
        LVK_ASSERT(thickness > 0);
        LVK_ASSERT(!dst.empty());

        const cv::Size2f motion_scale(dst.size());
        const cv::Size2f frame_scaling = cv::Size2f(dst.size()) / cv::Size2f(size() - 1);

        thread_local cv::UMat gpu_draw_mask(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
        cv::Mat draw_mask;

        draw_mask.create(dst.size(), CV_8UC1);
        draw_mask.setTo(cv::Scalar(0));

        // Draw all the motion vectors
        read([&](const cv::Point2f& offset, const cv::Point& coord){
            const cv::Point2f origin(
                static_cast<float>(coord.x) * frame_scaling.width,
                static_cast<float>(coord.y) * frame_scaling.height
            );

            cv::line(
                draw_mask,
                origin,
                origin - offset * motion_scale,
                cv::Scalar(255),
                thickness
            );
        });

        draw_mask.copyTo(gpu_draw_mask);
        dst.setTo(color, gpu_draw_mask);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::read(
        const std::function<void(const cv::Point2f&, const cv::Point&)>& operation,
        const bool parallel
    ) const
    {
        if(parallel)
        {
            // NOTE: this uses a parallel loop internally
            m_MeshOffsets.forEach<cv::Point2f>([&](const cv::Point2f& value, const int coord[]){
                operation(value, {coord[1], coord[0]});
            });
        }
        else
        {
            for(int r = 0; r < m_MeshOffsets.rows; r++)
            {
                const auto* row_ptr = m_MeshOffsets.ptr<cv::Point2f>(r);
                for(int c = 0; c < m_MeshOffsets.cols; c++)
                {
                    operation(row_ptr[c], {c, r});
                }
            }
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::write(
        const std::function<void(cv::Point2f&, const cv::Point&)>& operation,
        const bool parallel
    )
    {
        if(parallel)
        {
            // NOTE: this uses a parallel loop internally
            m_MeshOffsets.forEach<cv::Point2f>([&](cv::Point2f& value, const int coord[]){
                operation(value, {coord[1], coord[0]});
            });
        }
        else
        {
            for(int r = 0; r < m_MeshOffsets.rows; r++)
            {
                auto* row_ptr = m_MeshOffsets.ptr<cv::Point2f>(r);
                for(int c = 0; c < m_MeshOffsets.cols; c++)
                {
                    operation(row_ptr[c], {c, r});
                }
            }
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::set_identity()
    {
        m_MeshOffsets.setTo(cv::Scalar(0.0f, 0.0f));
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::set_to(const cv::Point2f& motion)
    {
        // NOTE: we invert the motion as the warp is specified backwards.
        m_MeshOffsets.setTo(cv::Scalar(-motion.x, -motion.y));
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::set_to(const Homography& motion, const cv::Size2f& motion_scale)
    {
        const auto coord_scaling = motion_scale / cv::Size2f(size() - 1);
        const auto norm_factor = 1.0f / motion_scale;

        write([&](cv::Point2f& offset, const cv::Point& coord){
            const auto sample_point = cv::Point2f(coord) * coord_scaling;
            offset = (sample_point - (motion * sample_point)) * norm_factor;
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::set_to(cv::Mat&& warp_map, const bool as_offsets, const bool normalized)
    {
        LVK_ASSERT(warp_map.type() == CV_32FC2);

        m_MeshOffsets = std::move(warp_map);
        if(!as_offsets) cv::subtract(m_MeshOffsets, view_identity_mesh(m_MeshOffsets.size()), m_MeshOffsets);
        if(!normalized) normalize(m_MeshOffsets.size());
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::set_to(const cv::Mat& warp_map, const bool as_offsets, const bool normalized)
    {
        LVK_ASSERT(warp_map.type() == CV_32FC2);

        warp_map.copyTo(m_MeshOffsets);
        if(!as_offsets) cv::subtract(m_MeshOffsets, view_identity_mesh(m_MeshOffsets.size()), m_MeshOffsets);
        if(!normalized) normalize(m_MeshOffsets.size());
    }

//---------------------------------------------------------------------------------------------------------------------


    void WarpMesh::scale(const cv::Size2f& scaling_factor)
    {
        const auto coord_scaling = ((1.0f / scaling_factor) - 1.0f) / cv::Size2f(size() - 1);
        write([&](cv::Point2f& offset, const cv::Point& coord){
            offset += cv::Point2f(coord) * coord_scaling;
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::crop_in(const cv::Rect2f& region)
    {
        LVK_ASSERT_RANGE(region.width, 0, cols());
        LVK_ASSERT_RANGE(region.height, 0, rows());
        LVK_ASSERT(region.x >= 0 && region.y >= 0);

        // Offset the region to the top left corner then scale it to fit.
        const auto coord_scaling = (region.size() - 1.0f) / cv::Size2f(size() - 1);
        write([&](cv::Point2f& offset, const cv::Point& coord){
            offset += cv::Point2f(coord) * coord_scaling + region.tl();
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::rotate(const float degrees)
    {
        const float radians = to_radians(degrees);
        const float cos = std::cos(radians), sin = std::sin(radians);

        // Rotate the coordinate grid about the centre.
        const auto norm_factor = 1.0f / cv::Size2f(size());
        const auto center = cv::Point2f(m_MeshOffsets.size() - 1) / 2;
        write([&](cv::Point2f& offset, const cv::Point& coord){
            const cv::Point2f arm = (coord - center) * norm_factor;
            offset.x += (arm.x * cos - arm.y * sin) - arm.x;
            offset.y += (arm.x * sin + arm.y * cos) - arm.y;
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::clamp(const cv::Size2f& magnitude)
    {
        write([&](cv::Point2f& offset, const cv::Point& coord){
            offset.x = std::clamp(offset.x, -magnitude.width, magnitude.width);
            offset.y = std::clamp(offset.y, -magnitude.height, magnitude.height);
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::clamp(const cv::Size2f& min, const cv::Size2f& max)
    {
        write([&](cv::Point2f& offset, const cv::Point& coord){
            offset.x = std::clamp(offset.x, min.width, max.width);
            offset.y = std::clamp(offset.y, min.height, max.height);
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::blend(const float mesh_weight, const WarpMesh& mesh)
    {
        cv::addWeighted(m_MeshOffsets, (1.0f - mesh_weight), mesh.m_MeshOffsets, mesh_weight, 0.0, m_MeshOffsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::blend(const float weight_1, const float weight_2, const WarpMesh& mesh)
    {
        cv::addWeighted(m_MeshOffsets, weight_1, mesh.m_MeshOffsets, weight_2, 0.0, m_MeshOffsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::combine(const WarpMesh& mesh, const float scaling)
    {
        cv::scaleAdd(mesh.m_MeshOffsets, scaling, m_MeshOffsets, m_MeshOffsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    // NOTE: This returns a view into a shared cache, do not modify the value.
    const cv::Mat WarpMesh::view_identity_mesh(const cv::Size& resolution)
    {
        // Since this is a repeated operation that often results in the same output,
        // we cache a copy of the grid in thread storage and return a view onto it.
        // If the grid is too small, it must be remade to match the larger size.

        thread_local cv::Mat coord_grid;
        if(resolution.width > coord_grid.cols || resolution.height > coord_grid.rows)
        {
            // Combine the resolutions maximally to avoid always throwing out the
            // cache for non-square resolutions which are rotations of each other.
            coord_grid = VirtualGrid({
                 std::max(resolution.height, coord_grid.rows),
                 std::max(resolution.width, coord_grid.cols)
            }).make_grid();
        }

        return coord_grid(cv::Rect({0,0}, resolution));
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh& WarpMesh::operator=(WarpMesh&& other) noexcept
    {
        m_MeshOffsets = std::move(other.m_MeshOffsets);

        return *this;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh& WarpMesh::operator=(const WarpMesh& other)
    {
        other.m_MeshOffsets.copyTo(m_MeshOffsets);

        return *this;
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::operator+=(const WarpMesh& other)
    {
        LVK_ASSERT(size() == other.size());

        cv::add(m_MeshOffsets, other.m_MeshOffsets, m_MeshOffsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::operator-=(const WarpMesh& other)
    {
        LVK_ASSERT(size() == other.size());

        cv::subtract(m_MeshOffsets, other.m_MeshOffsets, m_MeshOffsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::operator*=(const WarpMesh& other)
    {
        cv::multiply(m_MeshOffsets, other.m_MeshOffsets, m_MeshOffsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::operator+=(const cv::Point2f& offset)
    {
        cv::add(m_MeshOffsets, cv::Scalar(offset.x, offset.y), m_MeshOffsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::operator-=(const cv::Point2f& offset)
    {
        cv::subtract(m_MeshOffsets, cv::Scalar(offset.x, offset.y), m_MeshOffsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::operator*=(const cv::Size2f& scaling)
    {
        cv::multiply(m_MeshOffsets, cv::Scalar(scaling.width, scaling.height), m_MeshOffsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::operator/=(const cv::Size2f& scaling)
    {
        LVK_ASSERT(scaling.width != 0.0f && scaling.height != 0.0f);

        cv::divide(m_MeshOffsets, cv::Scalar(scaling.width, scaling.height), m_MeshOffsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::operator*=(const float scaling)
    {
        m_MeshOffsets *= scaling;
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpMesh::operator/=(const float scaling)
    {
        LVK_ASSERT(scaling != 0.0f);

        m_MeshOffsets /= scaling;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh operator+(const WarpMesh& left, const WarpMesh& right)
    {
        LVK_ASSERT(left.size() == right.size());

        cv::Mat result = left.offsets() + right.offsets();
        return WarpMesh(std::move(result), true, true);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh operator-(const WarpMesh& left, const WarpMesh& right)
    {
        LVK_ASSERT(left.size() == right.size());

        cv::Mat result = left.offsets() - right.offsets();
        return WarpMesh(std::move(result), true, true);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh operator*(const WarpMesh& left, const WarpMesh& right)
    {
        WarpMesh result(left);
        result *= right;
        return result;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh operator+(const WarpMesh& left, const cv::Point2f& right)
    {
        cv::Mat result = left.offsets() + cv::Scalar(right.x, right.y);
        return WarpMesh(std::move(result), true, true);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh operator-(const WarpMesh& left, const cv::Point2f& right)
    {
        cv::Mat result = left.offsets() - cv::Scalar(right.x, right.y);
        return WarpMesh(std::move(result), true, true);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh operator*(const WarpMesh& mesh, const cv::Size2f& scaling)
    {
        WarpMesh result(mesh);
        result *= scaling;
        return result;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh operator*(const cv::Size2f& scaling, const WarpMesh& mesh)
    {
        return mesh * scaling;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh operator/(const WarpMesh& mesh, const cv::Size2f& scaling)
    {
        LVK_ASSERT(scaling.height != 0.0f && scaling.height != 0.0f);

        WarpMesh result(mesh);
        result /= scaling;
        return result;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh operator/(const cv::Size2f& scaling, const WarpMesh& mesh)
    {
        return mesh / scaling;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh operator*(const WarpMesh& mesh, const float scaling)
    {
        cv::Mat result = mesh.offsets() * scaling;
        return WarpMesh(std::move(result), true, true);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh operator*(const float scaling, const WarpMesh& mesh)
    {
        return mesh * scaling;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh operator/(const WarpMesh& mesh, const float scaling)
    {
        LVK_ASSERT(scaling != 0.0f);

        cv::Mat result = mesh.offsets() / scaling;
        return WarpMesh(std::move(result), true, true);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh operator/(const float scaling, const WarpMesh& mesh)
    {
        return mesh / scaling;
    }

//---------------------------------------------------------------------------------------------------------------------
}