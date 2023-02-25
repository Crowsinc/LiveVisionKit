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
#include "Utility/Drawing.hpp"

namespace lvk
{
//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(const cv::Size& size)
        : m_VelocityField(size, CV_32FC2)
    {
        LVK_ASSERT(size.height >= MinimumSize.height);
        LVK_ASSERT(size.width >= MinimumSize.width);

        set_identity();
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(cv::Mat&& warp_motions)
        : m_VelocityField(std::move(warp_motions))
    {
        LVK_ASSERT(m_VelocityField.type() == CV_32FC2);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(const cv::Mat& warp_motions)
        : m_VelocityField(warp_motions.clone())
    {
        LVK_ASSERT(warp_motions.type() == CV_32FC2);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(WarpField&& other) noexcept
        : m_VelocityField(std::move(other.m_VelocityField))
    {}

//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(const WarpField& other)
        : m_VelocityField(other.m_VelocityField.clone())
    {}

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::resize(const cv::Size& new_size)
    {
        LVK_ASSERT(new_size.height >= MinimumSize.height);
        LVK_ASSERT(new_size.width >= MinimumSize.width);

        if(m_VelocityField.size() == new_size)
            return;

        cv::Mat result;
        cv::resize(m_VelocityField, result, new_size, 0, 0, cv::INTER_LINEAR);
        m_VelocityField = std::move(result);
    }

//---------------------------------------------------------------------------------------------------------------------

    cv::Size WarpField::size() const
    {
        return m_VelocityField.size();
    }

//---------------------------------------------------------------------------------------------------------------------

    int WarpField::cols() const
    {
        return m_VelocityField.cols;
    }

//---------------------------------------------------------------------------------------------------------------------

    int WarpField::rows() const
    {
        return m_VelocityField.rows;
    }

//---------------------------------------------------------------------------------------------------------------------

    cv::Mat& WarpField::data()
    {
        return m_VelocityField;
    }

//---------------------------------------------------------------------------------------------------------------------

    const cv::Mat& WarpField::data() const
    {
        return m_VelocityField;
    }

//---------------------------------------------------------------------------------------------------------------------

    cv::Point2f WarpField::sample(const cv::Point& position) const
    {
        LVK_ASSERT(position.x >= 0 && position.x < cols());
        LVK_ASSERT(position.y >= 0 && position.y < rows());

        return m_VelocityField.at<cv::Point2f>(position);
    }

//---------------------------------------------------------------------------------------------------------------------

    // Sample the warping velocity of the field at the given position.
    cv::Point2f WarpField::sample(const cv::Point2f& position) const
    {
        LVK_ASSERT(position.x >= 0.0f && position.x < static_cast<float>(cols()));
        LVK_ASSERT(position.y >= 0.0f && position.y < static_cast<float>(rows()));

        // To get an accurate reading, we will use bilinear filtering on the field. First,
        // we need to determine which four field points surround the position. This is done
        // by analysing the directions in which we are interpolating towards, with respect
        // to the center of the 'origin' field point that the position lies within.

        cv::Point2f top_left(std::floor(position.x), std::floor(position.y));
        cv::Point2f top_right = top_left, bot_left = top_left, bot_right = top_left;

        const cv::Point2f origin = top_left + cv::Point2f(0.5f, 0.5f);
        if(position.x >= origin.x)
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

        if(position.y >= origin.y)
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
        const float x_unit = position.x - static_cast<float>(top_left.x);
        const float y_unit = position.y - static_cast<float>(top_left.y);

        const float inv_x_unit = 1.0f - x_unit;
        const float inv_y_unit = 1.0f - y_unit;

        return m_VelocityField.at<cv::Point2f>(top_left) * inv_x_unit * inv_y_unit
               + m_VelocityField.at<cv::Point2f>(top_right) * x_unit * inv_y_unit
               + m_VelocityField.at<cv::Point2f>(bot_left) * inv_x_unit * y_unit
               + m_VelocityField.at<cv::Point2f>(bot_right) * x_unit * y_unit;
    }

//---------------------------------------------------------------------------------------------------------------------

    // Trace the position to get its final position after warping.
    cv::Point2f WarpField::trace(const cv::Point2f& position) const
    {
        return sample(position) + position;
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::set_identity()
    {
        m_VelocityField.setTo(cv::Vec2f::all(0.0f));
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::set_to(const cv::Point2f& motion)
    {
        m_VelocityField.setTo(cv::Scalar(-motion.x, -motion.y));
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::set_to(const Homography& warp, const cv::Size2f& scale)
    {
        const cv::Size2f point_scaling(
            scale.width / static_cast<float>(m_VelocityField.cols - 1),
            scale.height / static_cast<float>(m_VelocityField.rows - 1)
        );

        const auto inverse_warp = warp.invert();
        m_VelocityField.forEach<cv::Point2f>([&](cv::Point2f& source_point, const int position[]){
            const cv::Point2f sample_point(
                static_cast<float>(position[1]) * point_scaling.width,
                static_cast<float>(position[0]) * point_scaling.height
            );
            source_point = (inverse_warp * sample_point) - sample_point;
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::fit_to(
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

        const auto region_offset = described_region.tl();
        const auto region_size = described_region.size();

        // TODO: optimize and document the entire algorithm
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

        float accumulation_weight = 0.8f;
        cv::Rect2f alignment(region_offset - cv::Point2f(region_size / 2.0f), region_size * 2.0f);
        accumulate_motions(motions, accumulation_weight, alignment, origin_points, warped_points);

        while(motions.size() != m_VelocityField.size())
        {
            cv::Mat submotions(
                std::min(motions.rows * 2, m_VelocityField.rows),
                std::min(motions.cols * 2, m_VelocityField.cols),
                CV_32FC2
            );

            const cv::Size2f submotion_cell_size(
                region_size.width / static_cast<float>(submotions.cols - 1),
                region_size.height / static_cast<float>(submotions.rows - 1)
            );
            const cv::Rect2f submotion_alignment(
                region_offset.x - (submotion_cell_size.width * 0.5f),
                region_offset.y - (submotion_cell_size.height * 0.5f),
                static_cast<float>(submotions.cols) * alignment.x,
                static_cast<float>(submotions.rows) * alignment.y
            );

            accumulation_weight /= 2;
            cv::resize(motions, submotions, submotions.size(), 0, 0, cv::INTER_LINEAR);
            accumulate_motions(submotions, accumulation_weight, submotion_alignment, origin_points, warped_points);

            motions = std::move(submotions);
        }

        m_VelocityField = std::move(motions);
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

    void WarpField::translate_by(const cv::Vec2f& amount)
    {
        m_VelocityField.forEach<cv::Point2f>([&](cv::Point2f& warp_velocity, const int position[]){
            warp_velocity.x += amount[0];
            warp_velocity.y += amount[1];
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::clamp(const cv::Size2f& magnitude)
    {
        m_VelocityField.forEach<cv::Point2f>([&](cv::Point2f& warp_velocity, const int position[]){
            warp_velocity.x = std::clamp(warp_velocity.x, -magnitude.width, magnitude.width);
            warp_velocity.y = std::clamp(warp_velocity.y, -magnitude.height, magnitude.height);
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::clamp(const cv::Size2f& min, const cv::Size2f& max)
    {
        m_VelocityField.forEach<cv::Point2f>([&](cv::Point2f& warp_velocity, const int position[]){
            warp_velocity.x = std::clamp(warp_velocity.x, min.width, max.width);
            warp_velocity.y = std::clamp(warp_velocity.y, min.height, max.height);
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::merge_with(const WarpField& other, const float weight)
    {
        cv::scaleAdd(other.m_VelocityField, weight, m_VelocityField, m_VelocityField);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::merge_with(const WarpField& other, const float weight_1, const float weight_2, const float offset)
    {
        cv::addWeighted(m_VelocityField, weight_1, other.m_VelocityField, weight_2, offset, m_VelocityField);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::modify(const std::function<void(cv::Point2f&, cv::Point)>& operation)
    {
        m_VelocityField.forEach<cv::Point2f>([&](cv::Point2f& v, const int position[]){
            operation(v, {position[1], position[0]});
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::draw(cv::UMat& dst, const cv::Scalar& color, const float scaling) const
    {

        const cv::Size2f frame_scaling(
            static_cast<float>(dst.cols) / static_cast<float>(m_VelocityField.cols - 1),
            static_cast<float>(dst.rows) / static_cast<float>(m_VelocityField.rows - 1)
        );

        cv::Mat draw_buffer(dst.size(), CV_8UC3);
        draw_buffer.setTo(cv::Scalar(0, 0, 0));

        // Draw all the motion vectors
        m_VelocityField.forEach<cv::Point2f>([&](cv::Point2f& velocity, const int position[]){
            const cv::Point2f origin(
                static_cast<float>(position[1]) * frame_scaling.width,
                static_cast<float>(position[0]) * frame_scaling.height
            );

            // TODO: proper velocity colouring
            cv::line(
                draw_buffer,
                origin,
                origin + scaling * velocity,
                color,
                3
            );
        });

        thread_local cv::UMat staging_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
        draw_buffer.copyTo(staging_buffer);
        cv::add(dst, staging_buffer, dst);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::warp(const cv::UMat& src, cv::UMat& dst, const bool smoothing) const
    {
        // If we have a minimum size field, we can handle this as a Homography.
        if(m_VelocityField.cols == 2 && m_VelocityField.rows == 2)
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
                destination[0] + m_VelocityField.at<cv::Point2f>(0, 0),
                destination[1] + m_VelocityField.at<cv::Point2f>(0, 1),
                destination[2] + m_VelocityField.at<cv::Point2f>(1, 0),
                destination[3] + m_VelocityField.at<cv::Point2f>(1, 1)
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

        m_VelocityField.copyTo(staging_buffer);
        if(smoothing)
        {
            thread_local cv::UMat smooth_field(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

            cv::medianBlur(staging_buffer, smooth_field, 5);
            cv::blur(smooth_field, staging_buffer, cv::Size(3, 3));
        }
        cv::resize(staging_buffer, warp_map, src.size(), 0, 0, cv::INTER_LINEAR);
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
        m_VelocityField = std::move(other.m_VelocityField);

        return *this;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField& WarpField::operator=(const WarpField& other)
    {
        m_VelocityField = other.m_VelocityField.clone();

        return *this;
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator+=(const WarpField& other)
    {
        LVK_ASSERT(size() == other.size());

        cv::add(m_VelocityField, other.m_VelocityField, m_VelocityField);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator-=(const WarpField& other)
    {
        LVK_ASSERT(size() == other.size());

        cv::subtract(m_VelocityField, other.m_VelocityField, m_VelocityField);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator*=(const WarpField& other)
    {
        this->modify([&](cv::Point2f& v, const cv::Point coord){
            const auto multiplier = other.sample(coord);
            v.x *= multiplier.x;
            v.y *= multiplier.y;
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator*=(const float scaling)
    {
        m_VelocityField *= scaling;
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator/=(const float scaling)
    {
        LVK_ASSERT(scaling != 0.0f);

        m_VelocityField /= scaling;
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator*=(const cv::Vec2f& scaling)
    {
        m_VelocityField.forEach<cv::Point2f>([&](cv::Point2f& velocity, const int position[]){
            velocity.x *= scaling[0];
            velocity.y *= scaling[1];
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator/=(const cv::Vec2f& scaling)
    {
        LVK_ASSERT(scaling[0] != 0.0f && scaling[1] != 0.0f);

        m_VelocityField.forEach<cv::Point2f>([&](cv::Point2f& velocity, const int position[]){
            velocity.x /= scaling[0];
            velocity.y /= scaling[1];
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator+(const WarpField& left, const WarpField& right)
    {
        LVK_ASSERT(left.size() == right.size());

        cv::Mat result = left.data() + right.data();
        return WarpField(std::move(result));
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator-(const WarpField& left, const WarpField& right)
    {
        LVK_ASSERT(left.size() == right.size());

        cv::Mat result = left.data() - right.data();
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

    WarpField operator*(const WarpField& field, const float scaling)
    {
        cv::Mat result = field.data() * scaling;
        return WarpField(std::move(result));
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator*(const float scaling, const WarpField& field)
    {
        return field * scaling;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator*(const WarpField& field, const cv::Vec2f& scaling)
    {
        cv::Mat result = field.data();
        result.forEach<cv::Point2f>([&](cv::Point2f& velocity, const int position[]){
            velocity.x *= scaling[0];
            velocity.y *= scaling[1];
        });
        return WarpField(std::move(result));
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator*(const cv::Vec2f& scaling, const WarpField& field)
    {
        return field * scaling;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator/(const WarpField& field, const float scaling)
    {
        LVK_ASSERT(scaling != 0.0f);

        cv::Mat result = field.data() / scaling;
        return WarpField(std::move(result));
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator/(const float scaling, const WarpField& field)
    {
        return field / scaling;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator/(const WarpField& field, const cv::Vec2f& scaling)
    {
        LVK_ASSERT(scaling[0] != 0.0f && scaling[1] != 0.0f);

        cv::Mat result = field.data();
        result.forEach<cv::Point2f>([&](cv::Point2f& velocity, const int position[]){
            velocity.x /= scaling[0];
            velocity.y /= scaling[1];
        });
        return WarpField(std::move(result));
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator/(const cv::Vec2f& scaling, const WarpField& field)
    {
        return field / scaling;
    }

//---------------------------------------------------------------------------------------------------------------------

}