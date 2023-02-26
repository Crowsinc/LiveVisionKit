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

#include "WarpField.hpp"

#include <array>
#include <numeric>

#include "Diagnostics/Directives.hpp"
#include "Math/VirtualGrid.hpp"
#include "Utility/Drawing.hpp"
#include "Math/Math.hpp"

namespace lvk
{
//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(const cv::Size& size)
        : m_WarpOffsets(size, CV_32FC2)
    {
        LVK_ASSERT(size.height >= MinimumSize.height);
        LVK_ASSERT(size.width >= MinimumSize.width);

        set_identity();
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(const cv::Size& size, const cv::Point2f& motion)
        : m_WarpOffsets(size, CV_32FC2)
    {
        LVK_ASSERT(size.height >= MinimumSize.height);
        LVK_ASSERT(size.width >= MinimumSize.width);

        set_to(motion);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(const cv::Mat& warp_offsets)
        : m_WarpOffsets(warp_offsets.clone())
    {
        LVK_ASSERT(warp_offsets.type() == CV_32FC2);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(cv::Mat&& warp_offsets)
        : m_WarpOffsets(std::move(warp_offsets))
    {
        LVK_ASSERT(m_WarpOffsets.type() == CV_32FC2);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(WarpField&& other) noexcept
        : m_WarpOffsets(std::move(other.m_WarpOffsets))
    {}

//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(const WarpField& other)
        : m_WarpOffsets(other.m_WarpOffsets.clone())
    {}

//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(const cv::Size& size, const Homography& motion, const cv::Size2f& scale)
        : m_WarpOffsets(size, CV_32FC2)
    {
        LVK_ASSERT(size.height >= MinimumSize.height);
        LVK_ASSERT(size.width >= MinimumSize.width);

        set_to(motion, scale);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(
        const cv::Size& size,
        const cv::Rect2f& described_region,
        const std::vector<cv::Point2f>& origin_points,
        const std::vector<cv::Point2f>& warped_points,
        const std::optional<Homography>& motion_hint
    )
        : m_WarpOffsets(size, CV_32FC2)
    {
        LVK_ASSERT(size.height >= MinimumSize.height);
        LVK_ASSERT(size.width >= MinimumSize.width);

        fit_points(described_region, origin_points, warped_points, motion_hint);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::resize(const cv::Size& new_size)
    {
        LVK_ASSERT(new_size.height >= MinimumSize.height);
        LVK_ASSERT(new_size.width >= MinimumSize.width);

        if(m_WarpOffsets.size() == new_size)
            return;

        cv::Mat result;
        cv::resize(m_WarpOffsets, result, new_size, 0, 0, cv::INTER_LINEAR);
        m_WarpOffsets = std::move(result);
    }

//---------------------------------------------------------------------------------------------------------------------

    cv::Size WarpField::size() const
    {
        return m_WarpOffsets.size();
    }

//---------------------------------------------------------------------------------------------------------------------

    int WarpField::cols() const
    {
        return m_WarpOffsets.cols;
    }

//---------------------------------------------------------------------------------------------------------------------

    int WarpField::rows() const
    {
        return m_WarpOffsets.rows;
    }

//---------------------------------------------------------------------------------------------------------------------

    cv::Mat& WarpField::offsets()
    {
        return m_WarpOffsets;
    }

//---------------------------------------------------------------------------------------------------------------------

    const cv::Mat& WarpField::offsets() const
    {
        return m_WarpOffsets;
    }

//---------------------------------------------------------------------------------------------------------------------

    cv::Point2f WarpField::sample(const cv::Point& coord) const
    {
        LVK_ASSERT(coord.x >= 0 && coord.x < cols());
        LVK_ASSERT(coord.y >= 0 && coord.y < rows());

        return m_WarpOffsets.at<cv::Point2f>(coord);
    }

//---------------------------------------------------------------------------------------------------------------------

    // Sample the warping velocity of the field at the given coord.
    cv::Point2f WarpField::sample(const cv::Point2f& coord) const
    {
        LVK_ASSERT(coord.x >= 0.0f && coord.x < static_cast<float>(cols()));
        LVK_ASSERT(coord.y >= 0.0f && coord.y < static_cast<float>(rows()));

        // To get an accurate reading, we will use bilinear filtering on the field. First,
        // we need to determine which four field points surround the coord. This is done
        // by analysing the directions in which we are interpolating towards, with respect
        // to the center of the 'origin' field point that the coord lies within.

        cv::Point2f top_left(std::floor(coord.x), std::floor(coord.y));
        cv::Point2f top_right = top_left, bot_left = top_left, bot_right = top_left;

        const cv::Point2f origin = top_left + cv::Point2f(0.5f, 0.5f);
        if(coord.x >= origin.x)
        {
            // We are in quadrant 1 or 4, interpolating rightwards
            if(top_right.x < static_cast<float>(cols()))
            {
                top_right.x++;
                bot_right.x++;
            }
        }
        else
        {
            // We are in quadrant 2 or 3, interpolating leftwards
            if(top_left.x > 0)
            {
                top_left.x--;
                bot_left.x--;
            }
        }

        if(coord.y >= origin.y)
        {
            // We are in quadrant 1 or 2, interpolating upwards
            if(top_left.y < static_cast<float>(rows()))
            {
                top_right.y++;
                top_left.y++;
            }
        }
        else
        {
            // We are in quadrant 3 or 4, interpolating downwards
            if(bot_left.y > 0)
            {
                bot_right.y--;
                bot_left.y--;
            }
        }

        // Perform the billinear interpolation using a unit square mapping as outlined
        // on the Wikipedia page (https://en.wikipedia.org/wiki/Bilinear_interpolation)

        // TODO: check this this is the correct origin point with the inverted y axis.
        const float x_unit = coord.x - static_cast<float>(top_left.x);
        const float y_unit = coord.y - static_cast<float>(top_left.y);

        const float inv_x_unit = 1.0f - x_unit;
        const float inv_y_unit = 1.0f - y_unit;

        return m_WarpOffsets.at<cv::Point2f>(top_left) * inv_x_unit * inv_y_unit
               + m_WarpOffsets.at<cv::Point2f>(top_right) * x_unit * inv_y_unit
               + m_WarpOffsets.at<cv::Point2f>(bot_left) * inv_x_unit * y_unit
               + m_WarpOffsets.at<cv::Point2f>(bot_right) * x_unit * y_unit;
    }

//---------------------------------------------------------------------------------------------------------------------

    // Trace the position to get its final position after warping.
    cv::Point2f WarpField::trace(const cv::Point2f& coord) const
    {
        return sample(coord) + coord;
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::set_identity()
    {
        m_WarpOffsets.setTo(cv::Vec2f::all(0.0f));
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::set_to(const cv::Point2f& motion)
    {
        // NOTE: we invert the motion as the warp is specified backwards.
        m_WarpOffsets.setTo(cv::Scalar(-motion.x, -motion.y));
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::set_to(const Homography& motion, const cv::Size2f& scale)
    {
        const cv::Size2f point_scaling(
            scale.width / static_cast<float>(m_WarpOffsets.cols - 1),
            scale.height / static_cast<float>(m_WarpOffsets.rows - 1)
        );

        const Homography inverse_warp = motion.invert();
        write([&](cv::Point2f& offset, const cv::Point& coord){
            const cv::Point2f sample_point(
                static_cast<float>(coord.x) * point_scaling.width,
                static_cast<float>(coord.y) * point_scaling.height
            );
            offset = (inverse_warp * sample_point) - sample_point;
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::fit_points(
        const cv::Rect2f& described_region,
        const std::vector<cv::Point2f>& origin_points,
        const std::vector<cv::Point2f>& warped_points,
        const std::optional<Homography>& motion_hint
    )
    {
        LVK_ASSERT(origin_points.size() == warped_points.size());

        // This estimation algorithm is inspired the MeshFlow algorithm.
        // The original article is cited below:
        //
        // S. Liu, P. Tan, L. Yuan, J. Sun, and B. Zeng,
        // “MeshFlow: Minimum latency online video stabilization,"
        // Computer Vision – ECCV 2016, pp. 800–815, 2016.
        // TODO: optimize and document the entire algorithm

        const auto region_offset = described_region.tl();
        const auto region_size = described_region.size();

        cv::Mat motions(2, 2, CV_32FC2);
        if(motion_hint.has_value())
        {
            const auto warp_transform = motion_hint->invert();

            const cv::Point2f tl(region_offset.x, region_offset.y);
            const cv::Point2f tr(region_offset.x + region_size.width, region_offset.y);
            const cv::Point2f bl(region_offset.x, region_offset.y + region_size.height);
            const cv::Point2f br(tr.x, bl.y);

            motions.at<cv::Point2f>(0, 0) = (warp_transform * tl) - tl;
            motions.at<cv::Point2f>(0, 1) = (warp_transform * tr) - tr;
            motions.at<cv::Point2f>(1, 0) = (warp_transform * bl) - bl;
            motions.at<cv::Point2f>(1, 1) = (warp_transform * br) - br;
        }
        else
        {
            accumulate_motions(
                motions,
                1.0f,
                cv::Rect2f(
                    region_offset - cv::Point2f(region_size / 2.0f),
                    region_size * 2.0f
                ),
                origin_points,
                warped_points
            );
        }

        float motion_weight = 0.5f;
        while(motions.size() != m_WarpOffsets.size())
        {
            cv::Mat submotions(
                std::min(motions.rows * 2, m_WarpOffsets.rows),
                std::min(motions.cols * 2, m_WarpOffsets.cols),
                CV_32FC2
            );

            const cv::Size2f submotion_cell_size(
                region_size.width / static_cast<float>(submotions.cols - 1),
                region_size.height / static_cast<float>(submotions.rows - 1)
            );
            const cv::Rect2f submotion_alignment(
                region_offset.x - (submotion_cell_size.width * 0.5f),
                region_offset.y - (submotion_cell_size.height * 0.5f),
                static_cast<float>(submotions.cols) * submotion_cell_size.width,
                static_cast<float>(submotions.rows) * submotion_cell_size.height
            );

            motion_weight /= 2.0f;
            cv::resize(motions, submotions, submotions.size(), 0, 0, cv::INTER_LINEAR);
            accumulate_motions(submotions, motion_weight, submotion_alignment, origin_points, warped_points);

            motions = std::move(submotions);
        }

        m_WarpOffsets = std::move(motions);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::accumulate_motions(
        cv::Mat& motion_field,
        const float motion_weight,
        const cv::Rect2f& alignment,
        const std::vector<cv::Point2f>& origin_points,
        const std::vector<cv::Point2f>& warped_points
    )
    {
        LVK_ASSERT(motion_weight > 0);

        VirtualGrid partitions(motion_field.size(), alignment);
        for(size_t i = 0; i < origin_points.size(); i++)
        {
            const cv::Point2f& origin_point = origin_points[i];
            const cv::Point2f& warped_point = warped_points[i];
            auto warp_motion = origin_point - warped_point;

            if(const auto key = partitions.try_key_of(warped_point); key.has_value())
            {
                auto& motion_estimate = motion_field.at<cv::Point2f>(
                    static_cast<int>(key->y), static_cast<int>(key->x)
                );

                motion_estimate.x += motion_weight * static_cast<float>(sign(warp_motion.x - motion_estimate.x));
                motion_estimate.y += motion_weight * static_cast<float>(sign(warp_motion.y - motion_estimate.y));
            }
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::clamp(const cv::Size2f& magnitude)
    {
        write([&](cv::Point2f& offset, const cv::Point& coord){
            offset.x = std::clamp(offset.x, -magnitude.width, magnitude.width);
            offset.y = std::clamp(offset.y, -magnitude.height, magnitude.height);
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::clamp(const cv::Size2f& min, const cv::Size2f& max)
    {
        write([&](cv::Point2f& offset, const cv::Point& coord){
            offset.x = std::clamp(offset.x, min.width, max.width);
            offset.y = std::clamp(offset.y, min.height, max.height);
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::blend(const WarpField& field, const float scaling)
    {
        cv::scaleAdd(field.m_WarpOffsets, scaling, m_WarpOffsets, m_WarpOffsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::blend(const float weight_1, const float weight_2, const WarpField& field)
    {
        cv::addWeighted(m_WarpOffsets, weight_1, field.m_WarpOffsets, weight_2, 0.0, m_WarpOffsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::write(const std::function<void(cv::Point2f&, const cv::Point&)>& operation)
    {
        m_WarpOffsets.forEach<cv::Point2f>([&](cv::Point2f& value, const int coord[]){
            operation(value, {coord[1], coord[0]});
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::read(const std::function<void(const cv::Point2f&, const cv::Point&)>& operation) const
    {
        m_WarpOffsets.forEach<cv::Point2f>([&](const cv::Point2f& value, const int coord[]){
            operation(value, {coord[1], coord[0]});
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::draw(cv::UMat& dst, const cv::Scalar& color, const int thickness) const
    {
        LVK_ASSERT(thickness > 0);

        const cv::Size2f frame_scaling(
            static_cast<float>(dst.cols) / static_cast<float>(m_WarpOffsets.cols - 1),
            static_cast<float>(dst.rows) / static_cast<float>(m_WarpOffsets.rows - 1)
        );

        thread_local cv::UMat staging_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
        cv::Mat draw_buffer(dst.size(), CV_8UC3);

        // Draw all the motion vectors
        draw_buffer.setTo(cv::Scalar(0, 0, 0));
        read([&](const cv::Point2f& offset, const cv::Point& coord){
            const cv::Point2f origin(
                static_cast<float>(coord.x) * frame_scaling.width,
                static_cast<float>(coord.y) * frame_scaling.height
            );

            cv::line(
                draw_buffer,
                origin,
                origin - offset,
                color,
                thickness
            );
        });

        draw_buffer.copyTo(staging_buffer);
        cv::add(dst, staging_buffer, dst);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::warp(const cv::UMat& src, cv::UMat& dst, const bool smoothing) const
    {
        // If we have a minimum size field, we can handle this as a Homography.
        if(m_WarpOffsets.cols == 2 && m_WarpOffsets.rows == 2)
        {
            const auto width = static_cast<float>(src.cols);
            const auto height = static_cast<float>(src.rows);

            const std::array<cv::Point2f, 4> destination = {
                cv::Point2f(0, 0),
                cv::Point2f(width, 0),
                cv::Point2f(0, height),
                cv::Point2f(width, height)
            };
            const std::array<cv::Point2f, 4> source = {
                destination[0] + m_WarpOffsets.at<cv::Point2f>(0, 0),
                destination[1] + m_WarpOffsets.at<cv::Point2f>(0, 1),
                destination[2] + m_WarpOffsets.at<cv::Point2f>(1, 0),
                destination[3] + m_WarpOffsets.at<cv::Point2f>(1, 1)
            };

            cv::warpPerspective(
                src,
                dst,
                cv::getPerspectiveTransform(destination.data(), source.data()),
                src.size(),
                cv::WARP_INVERSE_MAP,
                cv::BORDER_CONSTANT
            );
            return;
        }

        // Upload the velocity field to the staging buffer and resize it to
        // match the source input. We then need to add the velocities onto
        // an identity field in order to create the final warp locations.
        // If the smoothing option is set, perform a 3x3 Gaussian on the
        // velocity field to ensure that the field is spatially continous.

        thread_local cv::UMat staging_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
        thread_local cv::UMat warp_map(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

        m_WarpOffsets.copyTo(staging_buffer);
        if(smoothing)
        {
            thread_local cv::UMat smooth_field(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
            cv::resize(staging_buffer, warp_map, src.size(), 0, 0, cv::INTER_LINEAR);

            // TODO: rework smoothing
            cv::GaussianBlur(warp_map, smooth_field, {5,5}, 0.5f);
            cv::GaussianBlur(smooth_field, warp_map, {3,3}, 0.5f);
        }
        else cv::resize(staging_buffer, warp_map, src.size(), 0, 0, cv::INTER_LINEAR);

        cv::add(warp_map, view_identity_field(src.size()), warp_map);
        cv::remap(src, dst, warp_map, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    }

//---------------------------------------------------------------------------------------------------------------------

    // NOTE: This returns an ROI into a shared cache, you do not own the returned value.
    const cv::UMat WarpField::view_identity_field(const cv::Size& resolution)
    {
        // Since this is a repeated operation that often results in the same
        // output. We will cache an identity field in static thread storage
        // and return a view onto it as desired. If the cached field is too
        // small, then we must remake it.

        thread_local cv::UMat identity_field;
        if(resolution.width > identity_field.cols || resolution.height > identity_field.rows)
        {
            // Combine the resolutions maximally to help avoid throwing out the cache
            // when we have two fields which both larger than each other in one dimension.
            identity_field.create(
                cv::Size(
                    std::max(resolution.width, identity_field.cols),
                    std::max(resolution.height, identity_field.rows)
                ),
                CV_32FC2,
                cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY
            );

            // Create row and column vectors with the respective x and y values.
            // Then use a nearest neighbour resize operation to bring them up to size.

            cv::Mat row_values(1, identity_field.cols, CV_32FC1);
            for(int i = 0; i < identity_field.cols; i++)
                row_values.at<float>(0, i) = static_cast<float>(i);

            cv::Mat col_values(identity_field.rows, 1, CV_32FC1);
            for(int i = 0; i < identity_field.rows; i++)
                col_values.at<float>(i, 0) = static_cast<float>(i);

            cv::UMat x_plane, y_plane;
            cv::resize(row_values, x_plane, identity_field.size(), 0, 0, cv::INTER_NEAREST_EXACT);
            cv::resize(col_values, y_plane, identity_field.size(), 0, 0, cv::INTER_NEAREST_EXACT);
            cv::merge(std::vector{x_plane, y_plane}, identity_field);
        }

        return identity_field(cv::Rect({0,0}, resolution));
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField& WarpField::operator=(WarpField&& other) noexcept
    {
        m_WarpOffsets = std::move(other.m_WarpOffsets);

        return *this;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField& WarpField::operator=(const WarpField& other)
    {
        m_WarpOffsets = other.m_WarpOffsets.clone();

        return *this;
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator+=(const WarpField& other)
    {
        LVK_ASSERT(size() == other.size());

        cv::add(m_WarpOffsets, other.m_WarpOffsets, m_WarpOffsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator-=(const WarpField& other)
    {
        LVK_ASSERT(size() == other.size());

        cv::subtract(m_WarpOffsets, other.m_WarpOffsets, m_WarpOffsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator*=(const WarpField& other)
    {
        write([&](cv::Point2f& offset, const cv::Point& coord){
            const auto multiplier = other.sample(coord);
            offset.x *= multiplier.x;
            offset.y *= multiplier.y;
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator+=(const cv::Vec2f& offset)
    {
        m_WarpOffsets += offset;
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator-=(const cv::Vec2f& offset)
    {
        m_WarpOffsets -= offset;
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator*=(const cv::Vec2f& scaling)
    {
        write([&](cv::Point2f& offset, const cv::Point& coord){
            offset.x *= scaling[0];
            offset.y *= scaling[1];
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator*=(const float scaling)
    {
        m_WarpOffsets *= scaling;
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator/=(const cv::Vec2f& scaling)
    {
        LVK_ASSERT(scaling[0] != 0.0f && scaling[1] != 0.0f);

        write([&](cv::Point2f& offset, const cv::Point& coord){
            offset.x /= scaling[0];
            offset.y /= scaling[1];
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator/=(const float scaling)
    {
        LVK_ASSERT(scaling != 0.0f);

        m_WarpOffsets /= scaling;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator+(const WarpField& left, const WarpField& right)
    {
        LVK_ASSERT(left.size() == right.size());

        cv::Mat result = left.offsets() + right.offsets();
        return WarpField(std::move(result));
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator-(const WarpField& left, const WarpField& right)
    {
        LVK_ASSERT(left.size() == right.size());

        cv::Mat result = left.offsets() - right.offsets();
        return WarpField(std::move(result));
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator*(const WarpField& left, const WarpField& right)
    {
        WarpField result(left);
        result *= right;
        return result;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator+(const WarpField& left, const cv::Vec2f& right)
    {
        cv::Mat result = left.offsets() + right;
        return WarpField(std::move(result));
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator-(const WarpField& left, const cv::Vec2f& right)
    {
        cv::Mat result = left.offsets() - right;
        return WarpField(std::move(result));
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator*(const WarpField& field, const cv::Vec2f& scaling)
    {
        WarpField result(field);
        result *= scaling;
        return result;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator*(const cv::Vec2f& scaling, const WarpField& field)
    {
        return field * scaling;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator/(const WarpField& field, const cv::Vec2f& scaling)
    {
        LVK_ASSERT(scaling[0] != 0.0f && scaling[1] != 0.0f);

        WarpField result(field);
        result /= scaling;
        return result;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator/(const cv::Vec2f& scaling, const WarpField& field)
    {
        return field / scaling;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator*(const WarpField& field, const float scaling)
    {
        cv::Mat result = field.offsets() * scaling;
        return WarpField(std::move(result));
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator*(const float scaling, const WarpField& field)
    {
        return field * scaling;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator/(const WarpField& field, const float scaling)
    {
        LVK_ASSERT(scaling != 0.0f);

        cv::Mat result = field.offsets() / scaling;
        return WarpField(std::move(result));
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator/(const float scaling, const WarpField& field)
    {
        return field / scaling;
    }

//---------------------------------------------------------------------------------------------------------------------
}