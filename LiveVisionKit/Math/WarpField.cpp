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

#include <opencv2/core/ocl.hpp>
#include <array>

#include "Functions/Extensions.hpp"
#include "Functions/Drawing.hpp"
#include "Timing/Stopwatch.hpp"
#include "Data/SpatialMap.hpp"
#include "Functions/Image.hpp"
#include "Functions/Math.hpp"
#include "Directives.hpp"

namespace lvk
{
//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(const cv::Size& size)
        : m_Field(size, CV_32FC2)
    {
        LVK_ASSERT(size.height >= MinimumSize.height);
        LVK_ASSERT(size.width >= MinimumSize.width);

        set_identity();
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(cv::Mat&& warp_map, const bool as_offsets, const bool normalized)
    {
        set_to(std::move(warp_map), as_offsets, normalized);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(const cv::Mat& warp_map, const bool as_offsets, const bool normalized)
    {
        set_to(warp_map, as_offsets, normalized);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(WarpField&& other) noexcept
        : m_Field(std::move(other.m_Field))
    {}

//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(const WarpField& other)
        : m_Field(other.m_Field.clone())
    {}

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::resize(const cv::Size& new_size)
    {
        LVK_ASSERT(new_size.height >= MinimumSize.height);
        LVK_ASSERT(new_size.width >= MinimumSize.width);

        if(m_Field.size() == new_size)
            return;

        cv::Mat new_field;
        cv::resize(m_Field, new_field, new_size, 0, 0, cv::INTER_LINEAR);
        m_Field = std::move(new_field);
    }

//---------------------------------------------------------------------------------------------------------------------

    cv::Size WarpField::size() const
    {
        return m_Field.size();
    }

//---------------------------------------------------------------------------------------------------------------------

    int WarpField::cols() const
    {
        return m_Field.cols;
    }

//---------------------------------------------------------------------------------------------------------------------

    int WarpField::rows() const
    {
        return m_Field.rows;
    }

//---------------------------------------------------------------------------------------------------------------------

    cv::Mat& WarpField::offsets()
    {
        return m_Field;
    }

//---------------------------------------------------------------------------------------------------------------------

    const cv::Mat& WarpField::offsets() const
    {
        return m_Field;
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::to_map(cv::Mat& dst) const
    {
        cv::add(m_Field, view_field_grid(m_Field.size()), dst);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::to_map(cv::UMat& dst) const
    {
        cv::add(m_Field, view_field_grid(m_Field.size()), dst);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::normalize(const cv::Size2f& motion_scale)
    {
        const cv::Scalar norm_factor(
            1.0f / motion_scale.width,
            1.0f / motion_scale.height
        );

        cv::multiply(m_Field, norm_factor, m_Field);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::undistort(const float tolerance)
    {
        LVK_ASSERT(tolerance >= 0);

        // Undistort the field by finding a parallelogram of best fit and anchoring all
        // offsets to be within a tolerance of that. This should result in a warp that
        // is more affine. To find the parallelogram, the line of best fit will be found
        // for all x and y offsets, which correspond to the y values of the vertical and
        // horizontal parallel lines of the parallelogram. For each line, the x coordinate
        // always corresponds to the respective grid coord.
        //
        // The linear regression formulae taken from:
        // https://www.tutorialspoint.com/regression-analysis-and-the-best-fitting-line-using-cplusplus

        const cv::Scalar n(cols(), rows());
        const cv::Scalar nt(rows(), cols());
        const cv::Scalar N = n * nt;

        // Simple sums can be calculated up front using series.
        const auto x_sum = nt * (n * (n - 1.0)) / 2.0;
        const auto x2_sum = nt * (n * (n + 1.0) * (2.0 * n + 1.0)) / 6.0;

        // Get the y sum directly from the offsets.
        const auto y_sum = cv::sum(m_Field);

        // Multiply the offsets by the coordinate grid to get the xy sum.
        cv::Mat xy_offsets;
        cv::multiply(m_Field, view_field_grid(size()), xy_offsets);
        const auto xy_sum = cv::sum(xy_offsets);

        // Calculate the slope and intercepts of the lines.
        auto slope = (N * xy_sum - x_sum * y_sum) / (N * x2_sum - x_sum * x_sum);
        auto intercept = (y_sum - slope * x_sum) / N;

        // Create the field anchor offsets using the lines.
        cv::Mat anchors;
        cv::multiply(view_field_grid(size()), slope, anchors);
        cv::add(anchors, intercept, anchors);

        // Apply the tolerance to the anchor points.
        if(tolerance >= 1.0f)
        {
            write([&](cv::Point2f& offset, const cv::Point& coord){
                const auto anchor = anchors.at<cv::Point2f>(coord);

                offset.x = anchor.x + std::clamp(offset.x - anchor.x, -tolerance, tolerance);
                offset.y = anchor.y + std::clamp(offset.y - anchor.y, -tolerance, tolerance);
            });
        }
        else std::swap(anchors, m_Field);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::apply(const cv::UMat& src, cv::UMat& dst) const
    {
        const cv::Scalar motion_scaling(src.cols, src.rows);

        if(m_Field.size() != MinimumSize)
        {
            // If our field is larger than 2x2 then scale up the field and remap the input.
            cv::resize(m_Field, m_WarpMap, src.size(), 0, 0, cv::INTER_LINEAR_EXACT);
            cv::multiply(m_WarpMap, motion_scaling, m_WarpMap);
            lvk::remap(src, dst, m_WarpMap, true /* assume yuv */);
        }
        else
        {
            // If our field is 2x2, then we can directly model it with a homography.
            const auto w = static_cast<float>(src.cols);
            const auto h = static_cast<float>(src.rows);
            const std::array<cv::Point2f, 4> destination = {
                cv::Point2f(0, 0), cv::Point2f(w, 0),
                cv::Point2f(0, h), cv::Point2f(w, h)
            };

            const std::array<cv::Point2f, 4> source = {
                destination[0] + m_Field.at<cv::Point2f>(0, 0) * motion_scaling,
                destination[1] + m_Field.at<cv::Point2f>(0, 1) * motion_scaling,
                destination[2] + m_Field.at<cv::Point2f>(1, 0) * motion_scaling,
                destination[3] + m_Field.at<cv::Point2f>(1, 1) * motion_scaling
            };

            cv::warpPerspective(
                src,
                dst,
                cv::getPerspectiveTransform(destination.data(), source.data()),
                src.size(),
                cv::WARP_INVERSE_MAP | cv::INTER_LINEAR,
                cv::BORDER_CONSTANT
            );
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::apply(const VideoFrame& src, VideoFrame& dst) const
    {
        // TODO: create specialized apply function for video frames.
        apply(static_cast<cv::UMat>(src), dst);

        // Update metadata.
        dst.timestamp = src.timestamp;
        dst.format = src.format;
    }

//---------------------------------------------------------------------------------------------------------------------

    // TODO: optimize this
    void WarpField::draw(cv::UMat& dst, const cv::Scalar& color, const int thickness) const
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

    void WarpField::read(
        const std::function<void(const cv::Point2f&, const cv::Point&)>& operation,
        const bool parallel
    ) const
    {
        if(parallel)
        {
            // NOTE: this uses a parallel loop internally
            m_Field.forEach<cv::Point2f>([&](const cv::Point2f& value, const int coord[]){
                operation(value, {coord[1], coord[0]});
            });
        }
        else
        {
            for(int r = 0; r < m_Field.rows; r++)
            {
                const auto* row_ptr = m_Field.ptr<cv::Point2f>(r);
                for(int c = 0; c < m_Field.cols; c++)
                {
                    operation(row_ptr[c], {c, r});
                }
            }
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::write(
        const std::function<void(cv::Point2f&, const cv::Point&)>& operation,
        const bool parallel
    )
    {
        if(parallel)
        {
            // NOTE: this uses a parallel loop internally
            m_Field.forEach<cv::Point2f>([&](cv::Point2f& value, const int coord[]){
                operation(value, {coord[1], coord[0]});
            });
        }
        else
        {
            for(int r = 0; r < m_Field.rows; r++)
            {
                auto* row_ptr = m_Field.ptr<cv::Point2f>(r);
                for(int c = 0; c < m_Field.cols; c++)
                {
                    operation(row_ptr[c], {c, r});
                }
            }
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::fit_points(
        const cv::Rect2f& region,
        const Homography& global_motion,
        const std::vector<cv::Point2f>& origin_points,
        const std::vector<cv::Point2f>& warped_points
    )
    {
        LVK_ASSERT(origin_points.size() == warped_points.size());

        cv::Mat motions(WarpField::MinimumSize, CV_32FC2);
        VirtualGrid submotion_grid(WarpField::MinimumSize);
        float submotion_weight = 0.7f;

        // Pre-fill the initial warps using the motion hint.
        const cv::Point2f tl = region.tl(), br = region.br();
        const cv::Point2f tr(br.x, tl.y), bl(tl.x, br.y);

        const auto global_warp = global_motion.invert();
        motions.at<cv::Point2f>(0, 0) = (global_warp * tl) - tl;
        motions.at<cv::Point2f>(0, 1) = (global_warp * tr) - tr;
        motions.at<cv::Point2f>(1, 0) = (global_warp * bl) - bl;
        motions.at<cv::Point2f>(1, 1) = (global_warp * br) - br;

        // NOTE: we force the first iteration of the algorithm for 2x2 fields.
        bool force_accumulation = m_Field.size() == WarpField::MinimumSize;

        // Sub-accumulate the motions until we reach the right field size.
        while(motions.size() != m_Field.size() || force_accumulation)
        {
            force_accumulation = false;

            // Expand the submotions grid to be 2x larger.
            submotion_grid.resize({
                std::min(motions.cols * 2, m_Field.cols),
                std::min(motions.rows * 2, m_Field.rows)
            });

            // Align the grid so that the cells are centered on the field coords.
            const cv::Size2f grid_size = submotion_grid.size();
            const auto cell_size = region.size() / (grid_size - 1.0f);
            submotion_grid.align(cv::Rect2f(region.tl() * (cell_size / 2.0f), grid_size * cell_size));

            // Interpolate the current motions field into the larger submotions field.
            cv::Mat submotions(submotion_grid.size(), CV_32FC2);
            cv::resize(motions, submotions, submotions.size(), 0, 0, cv::INTER_LINEAR);

            // Re-accumulate all the motions into the new submotions grid.
            for(size_t i = 0; i < origin_points.size(); i++)
            {
                const cv::Point2f& origin_point = origin_points[i];
                const cv::Point2f& warped_point = warped_points[i];
                auto warp_motion = origin_point - warped_point;

                // Update the motion estimate for the cell we reside in.
                auto& motion_estimate = submotions.at<cv::Point2f>(submotion_grid.key_of(warped_point));
                motion_estimate.x += submotion_weight * (warp_motion.x - motion_estimate.x);
                motion_estimate.y += submotion_weight * (warp_motion.y - motion_estimate.y);
            }

            // Spatially blur the submotions to remove noise.
            cv::medianBlur(submotions, motions, 3);

            submotion_weight /= 2.0f;
        }

        m_Field = std::move(motions);
        normalize(region.size());
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::set_identity()
    {
        m_Field.setTo(cv::Scalar(0.0f, 0.0f));
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::set_to(const cv::Point2f& motion)
    {
        // NOTE: we invert the motion as the warp is specified backwards.
        m_Field.setTo(cv::Scalar(-motion.x, -motion.y));
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::set_to(const Homography& motion, const cv::Size2f& motion_scale)
    {
        const cv::Point2f coord_scaling = motion_scale / cv::Size2f(size() - 1);
        const cv::Size2f norm_factor = 1.0f / motion_scale;

        const Homography inverse_warp = motion.invert();
        write([&](cv::Point2f& offset, const cv::Point& coord){
            const auto sample_point = cv::Point2f(coord) * coord_scaling;
            offset = ((inverse_warp * sample_point) - sample_point) * norm_factor;
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::set_to(cv::Mat&& warp_map, const bool as_offsets, const bool normalized)
    {
        LVK_ASSERT(warp_map.type() == CV_32FC2);

        m_Field = std::move(warp_map);
        if(!as_offsets) cv::subtract(m_Field, view_field_grid(m_Field.size()), m_Field);
        if(!normalized) normalize(m_Field.size());
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::set_to(const cv::Mat& warp_map, const bool as_offsets, const bool normalized)
    {
        LVK_ASSERT(warp_map.type() == CV_32FC2);

        warp_map.copyTo(m_Field);
        if(!as_offsets) cv::subtract(m_Field, view_field_grid(m_Field.size()), m_Field);
        if(!normalized) normalize(m_Field.size());
    }

//---------------------------------------------------------------------------------------------------------------------


    void WarpField::scale(const cv::Size2f& scaling_factor)
    {
        const auto coord_scaling = ((1.0f / scaling_factor) - 1.0f) / cv::Size2f(size() - 1);
        write([&](cv::Point2f& offset, const cv::Point& coord){
            offset += cv::Point2f(coord) * coord_scaling;
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::crop_in(const cv::Rect2f& region)
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

    void WarpField::rotate(const float degrees)
    {
        const float radians = to_radians(degrees);
        const float cos = std::cos(radians), sin = std::sin(radians);

        // Rotate the coordinate grid about the centre.
        const auto norm_factor = 1.0f / cv::Size2f(size());
        const auto center = cv::Point2f(m_Field.size() - 1) / 2;
        write([&](cv::Point2f& offset, const cv::Point& coord){
            const cv::Point2f arm = (coord - center) * norm_factor;
            offset.x += (arm.x * cos - arm.y * sin) - arm.x;
            offset.y += (arm.x * sin + arm.y * cos) - arm.y;
        });
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

    void WarpField::blend(const float field_weight, const WarpField& field)
    {
        cv::addWeighted(m_Field, (1.0f - field_weight), field.m_Field, field_weight, 0.0, m_Field);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::blend(const float weight_1, const float weight_2, const WarpField& field)
    {
        cv::addWeighted(m_Field, weight_1, field.m_Field, weight_2, 0.0, m_Field);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::combine(const WarpField& field, const float scaling)
    {
        cv::scaleAdd(field.m_Field, scaling, m_Field, m_Field);
    }

//---------------------------------------------------------------------------------------------------------------------

    // NOTE: This returns a view into a shared cache, do not modify the value.
    const cv::Mat WarpField::view_field_grid(const cv::Size& resolution)
    {
        // Since this is a repeated operation that often results in the same output,
        // we cache a copy of the grid in thread storage and return a view onto it.
        // If the grid is too small, it must be remade to match the larger size.

        thread_local cv::Mat coord_grid;
        if(resolution.width > coord_grid.cols || resolution.height > coord_grid.rows)
        {
            // Combine the resolutions maximally to avoid always throwing out the
            // cache for non-square resolutions which are rotations of each other.
            coord_grid.create(
                std::max(resolution.height, coord_grid.rows),
                std::max(resolution.width, coord_grid.cols),
                CV_32FC2
            );

            // Create row and column vectors with the respective coord x and y values.
            // Then use a nearest neighbour resize operation to bring them up to size.
            cv::Mat row_values(1, coord_grid.cols, CV_32FC1);
            for(int i = 0; i < coord_grid.cols; i++)
                row_values.at<float>(0, i) = static_cast<float>(i);

            cv::Mat col_values(coord_grid.rows, 1, CV_32FC1);
            for(int i = 0; i < coord_grid.rows; i++)
                col_values.at<float>(i, 0) = static_cast<float>(i);

            cv::Mat x_plane, y_plane;
            cv::resize(row_values, x_plane, coord_grid.size(), 0, 0, cv::INTER_NEAREST_EXACT);
            cv::resize(col_values, y_plane, coord_grid.size(), 0, 0, cv::INTER_NEAREST_EXACT);
            cv::merge(std::vector{x_plane, y_plane}, coord_grid);
        }

        return coord_grid(cv::Rect({0,0}, resolution));
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField& WarpField::operator=(WarpField&& other) noexcept
    {
        m_Field = std::move(other.m_Field);

        return *this;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField& WarpField::operator=(const WarpField& other)
    {
        other.m_Field.copyTo(m_Field);

        return *this;
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator+=(const WarpField& other)
    {
        LVK_ASSERT(size() == other.size());

        cv::add(m_Field, other.m_Field, m_Field);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator-=(const WarpField& other)
    {
        LVK_ASSERT(size() == other.size());

        cv::subtract(m_Field, other.m_Field, m_Field);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator*=(const WarpField& other)
    {
        cv::multiply(m_Field, other.m_Field, m_Field);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator+=(const cv::Point2f& offset)
    {
        cv::add(m_Field, cv::Scalar(offset.x, offset.y), m_Field);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator-=(const cv::Point2f& offset)
    {
        cv::subtract(m_Field, cv::Scalar(offset.x, offset.y), m_Field);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator*=(const cv::Size2f& scaling)
    {
        cv::multiply(m_Field, cv::Scalar(scaling.width, scaling.height), m_Field);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator/=(const cv::Size2f& scaling)
    {
        LVK_ASSERT(scaling.width != 0.0f && scaling.height != 0.0f);

        cv::divide(m_Field, cv::Scalar(scaling.width, scaling.height), m_Field);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator*=(const float scaling)
    {
        m_Field *= scaling;
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator/=(const float scaling)
    {
        LVK_ASSERT(scaling != 0.0f);

        m_Field /= scaling;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator+(const WarpField& left, const WarpField& right)
    {
        LVK_ASSERT(left.size() == right.size());

        cv::Mat result = left.offsets() + right.offsets();
        return WarpField(std::move(result), true, true);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator-(const WarpField& left, const WarpField& right)
    {
        LVK_ASSERT(left.size() == right.size());

        cv::Mat result = left.offsets() - right.offsets();
        return WarpField(std::move(result), true, true);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator*(const WarpField& left, const WarpField& right)
    {
        WarpField result(left);
        result *= right;
        return result;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator+(const WarpField& left, const cv::Point2f& right)
    {
        cv::Mat result = left.offsets() + cv::Scalar(right.x, right.y);
        return WarpField(std::move(result), true, true);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator-(const WarpField& left, const cv::Point2f& right)
    {
        cv::Mat result = left.offsets() - cv::Scalar(right.x, right.y);
        return WarpField(std::move(result), true, true);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator*(const WarpField& field, const cv::Size2f& scaling)
    {
        WarpField result(field);
        result *= scaling;
        return result;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator*(const cv::Size2f& scaling, const WarpField& field)
    {
        return field * scaling;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator/(const WarpField& field, const cv::Size2f& scaling)
    {
        LVK_ASSERT(scaling.height != 0.0f && scaling.height != 0.0f);

        WarpField result(field);
        result /= scaling;
        return result;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator/(const cv::Size2f& scaling, const WarpField& field)
    {
        return field / scaling;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator*(const WarpField& field, const float scaling)
    {
        cv::Mat result = field.offsets() * scaling;
        return WarpField(std::move(result), true, true);
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
        return WarpField(std::move(result), true, true);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator/(const float scaling, const WarpField& field)
    {
        return field / scaling;
    }

//---------------------------------------------------------------------------------------------------------------------
}