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

    WarpField WarpField::Estimate(
        const cv::Size& field_size,
        const cv::Rect2f& field_region,
        const std::vector<cv::Point2f>& origin_points,
        const std::vector<cv::Point2f>& warped_points,
        const std::optional<Homography>& motion_hint
    )
    {
        LVK_ASSERT(field_size.width >= MinimumSize.width);
        LVK_ASSERT(field_size.height >= MinimumSize.height);
        LVK_ASSERT(origin_points.size() == warped_points.size());

        // This estimation algorithm is inspired by the MeshFlow algorithm.
        // The original article is cited below:
        //
        // S. Liu, P. Tan, L. Yuan, J. Sun, and B. Zeng,
        // “MeshFlow: Minimum latency online video stabilization,"
        // Computer Vision – ECCV 2016, pp. 800–815, 2016.
        //

        // TODO: optimize and document the entire algorithm

        SpatialMap<cv::Point2f> motion_accumulator(cv::Size(2,2), cv::Rect2f(
                field_region.x - (field_region.width / 2),
                field_region.y - (field_region.height / 2),
                2 * field_region.height,
                2 * field_region.width
        ));

        if(motion_hint.has_value())
        {
            const auto warp_transform = motion_hint->invert();

            const cv::Point2f top_left = field_region.tl();
            motion_accumulator.place_at({0,0}, (warp_transform * top_left) - top_left);

            const cv::Point2f top_right(field_region.x + field_region.width, field_region.y);
            motion_accumulator.place_at({1,0}, (warp_transform * top_right) - top_right);

            const cv::Point2f bot_left(field_region.x, field_region.y + field_region.height);
            motion_accumulator.place_at({0,1}, (warp_transform * bot_left) - bot_left);

            const cv::Point2f bot_right = field_region.br();
            motion_accumulator.place_at({1,1}, (warp_transform * bot_right) - bot_right);
        } else motion_accumulator.set_to(0.0f, 0.0f);


        while(motion_accumulator.resolution() != field_size)
        {
            SpatialMap<cv::Point2f> submotion_accumulator(
                cv::Size(
                    std::min<int>(2 * motion_accumulator.resolution().width, field_size.width),
                    std::min<int>(2 * motion_accumulator.resolution().height, field_size.height)
                )
            );

            const cv::Size2f aligned_cell_size(
                field_region.width / static_cast<float>(submotion_accumulator.cols() - 1),
                field_region.height / static_cast<float>(submotion_accumulator.rows() - 1)
            );

            submotion_accumulator.align(cv::Rect2f(
                field_region.x - 0.5f * aligned_cell_size.width,
                field_region.y - 0.5f * aligned_cell_size.height,
                static_cast<float>(submotion_accumulator.cols()) * aligned_cell_size.width,
                static_cast<float>(submotion_accumulator.rows()) * aligned_cell_size.height
            ));

            const cv::Size2f accumulator_scaling(
                static_cast<float>(submotion_accumulator.cols()) / static_cast<float>(motion_accumulator.cols()),
                static_cast<float>(submotion_accumulator.rows()) / static_cast<float>(motion_accumulator.rows())
            );

            // TODO: optimize this logic
            // Fill in accumulator with medians of upper accumulator
            const size_t dx = (submotion_accumulator.cols() > motion_accumulator.cols()) ? 1 : 0;
            const size_t dy = (submotion_accumulator.rows() > motion_accumulator.rows()) ? 1 : 0;
            for(const auto& [key, motion_estimate] : motion_accumulator)
            {
                const SpatialKey scaled_key(
                    static_cast<size_t>(static_cast<float>(key.x) * accumulator_scaling.width),
                    static_cast<size_t>(static_cast<float>(key.y) * accumulator_scaling.height)
                );

                submotion_accumulator.place_at(scaled_key, motion_estimate);
                submotion_accumulator.place_at({scaled_key.x + dx, scaled_key.y}, motion_estimate);
                submotion_accumulator.place_at({scaled_key.x, scaled_key.y + dy}, motion_estimate);
                submotion_accumulator.place_at({scaled_key.x + dx, scaled_key.y + dy}, motion_estimate);
            }

            for(size_t i = 0; i < origin_points.size(); i++)
            {
                const cv::Point2f& origin_point = origin_points[i];
                const cv::Point2f& warped_point = warped_points[i];
                auto warp_motion = origin_point - warped_point;

                if(const auto key = submotion_accumulator.try_key_of(warped_point); key.has_value())
                {
                    auto& motion_estimate = submotion_accumulator.at(*key);

                    constexpr float epa = 0.5f; // TODO: set properly
                    motion_estimate.x += epa * static_cast<float>(sign(warp_motion.x - motion_estimate.x));
                    motion_estimate.y += epa * static_cast<float>(sign(warp_motion.y - motion_estimate.y));
                }
            }

            motion_accumulator = std::move(submotion_accumulator);
        }

        cv::Mat warp_velocity_field(field_size, CV_32FC2);
        warp_velocity_field.forEach<cv::Point2f>([&](cv::Point2f& warp_motion, const int position[]){
            warp_motion = motion_accumulator.at(SpatialKey(position[1], position[0]));
        });

        return WarpField(std::move(warp_velocity_field));
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(const cv::Size& size)
        : m_VelocityField(size, CV_32FC2)
    {
        LVK_ASSERT(size.height >= MinimumSize.height);
        LVK_ASSERT(size.width >= MinimumSize.width);

        set_identity();
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(cv::Mat&& velocity_field)
        : m_VelocityField(std::move(velocity_field))
    {
        LVK_ASSERT(m_VelocityField.type() == CV_32FC2);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(const cv::Mat& velocity_field)
        : m_VelocityField(velocity_field.clone())
    {
        LVK_ASSERT(velocity_field.type() == CV_32FC2);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(const Homography& warp, const cv::Size& size, const cv::Size2f& scale)
        : m_VelocityField(size, CV_32FC2)
    {
        LVK_ASSERT(size.height >= MinimumSize.height);
        LVK_ASSERT(size.width >= MinimumSize.width);

        set_to(warp, scale);
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

    const cv::Mat& WarpField::data() const
    {
        return m_VelocityField;
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::set_identity()
    {
        m_VelocityField.setTo(cv::Vec2f::all(0.0f));
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

    void WarpField::translate_by(const cv::Vec2f& amount)
    {
        m_VelocityField.forEach<cv::Point2f>([&](cv::Point2f& source_point, const int position[]){
            source_point.x += amount[0];
            source_point.y += amount[1];
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField WarpField::smoothen(const int size) const
    {
        LVK_ASSERT(size % 2 == 1);
        LVK_ASSERT(size >= 3);

        cv::Mat smooth_field;
        cv::stackBlur(m_VelocityField, smooth_field, cv::Size(size, size));
        return WarpField(std::move(smooth_field));
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

    void WarpField::draw(cv::UMat& dst, const float motion_scaling) const
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
                origin + motion_scaling * velocity,
                10.0f * motion_scaling * cv::Scalar(velocity.x * velocity.y, velocity.x, velocity.y),
                3
            );
        });

        thread_local cv::UMat staging_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
        draw_buffer.copyTo(staging_buffer);
        cv::add(dst, staging_buffer, dst);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::warp(const cv::UMat& src, cv::UMat& dst) const
    {
        thread_local cv::UMat staging_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
        thread_local cv::UMat warp_map(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

        // If we have a minimum size field, we can handle this as a Homography
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
                cv::BORDER_REFLECT
            );
            return;
        }

        // Upload the velocity field to the staging buffer and resize it to
        // match the source input. We then need to add the velocities onto
        // an identity field in order to create the final warp locations.

        m_VelocityField.copyTo(staging_buffer);
        cv::resize(staging_buffer, warp_map, src.size(), 0, 0, cv::INTER_LINEAR);
        cv::add(warp_map, view_identity_field(src.size()), warp_map);
        cv::remap(src, dst, warp_map, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_REFLECT);
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

    void WarpField::operator*=(const float scaling)
    {
        m_VelocityField *= scaling;
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator/=(const float scaling)
    {
        LVK_ASSERT(scaling == 0.0f);

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