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
#include "Math/VirtualGrid.hpp"
#include "Timing/Stopwatch.hpp"
#include "Functions/Image.hpp"
#include "Functions/Math.hpp"
#include "Directives.hpp"

namespace lvk
{
//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(const cv::Size& size)
        : m_Offsets(size, CV_32FC2)
    {
        LVK_ASSERT(size.height >= MinimumSize.height);
        LVK_ASSERT(size.width >= MinimumSize.width);

        set_identity();
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(cv::Mat&& warp_map, const bool as_offsets)
        : m_Offsets(std::move(warp_map))
    {
        LVK_ASSERT(m_Offsets.type() == CV_32FC2);

        // If the warp was given as an absolute warp map, we need to convert it to offsets.
        if(!as_offsets) cv::subtract(m_Offsets, view_coord_grid(m_Offsets.size()), m_Offsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(const cv::Mat& warp_map, const bool as_offsets)
        : m_Offsets(warp_map.clone())
    {
        LVK_ASSERT(m_Offsets.type() == CV_32FC2);

        // If the warp was given as an absolute warp map, we need to convert it to offsets.
        if(!as_offsets) cv::subtract(m_Offsets, view_coord_grid(m_Offsets.size()), m_Offsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(WarpField&& other) noexcept
        : m_Offsets(std::move(other.m_Offsets))
    {}

//---------------------------------------------------------------------------------------------------------------------

    WarpField::WarpField(const WarpField& other)
        : m_Offsets(other.m_Offsets.clone())
    {}

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::resize(const cv::Size& new_size)
    {
        LVK_ASSERT(new_size.height >= MinimumSize.height);
        LVK_ASSERT(new_size.width >= MinimumSize.width);

        if(m_Offsets.size() == new_size)
            return;

        cv::Mat new_field;
        cv::resize(m_Offsets, new_field, new_size, 0, 0, cv::INTER_LINEAR);
        m_Offsets = std::move(new_field);
    }

//---------------------------------------------------------------------------------------------------------------------

    cv::Size WarpField::size() const
    {
        return m_Offsets.size();
    }

//---------------------------------------------------------------------------------------------------------------------

    int WarpField::cols() const
    {
        return m_Offsets.cols;
    }

//---------------------------------------------------------------------------------------------------------------------

    int WarpField::rows() const
    {
        return m_Offsets.rows;
    }

//---------------------------------------------------------------------------------------------------------------------

    const cv::Mat& WarpField::offsets() const
    {
        return m_Offsets;
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::to_map(cv::Mat& dst) const
    {
        cv::add(m_Offsets, view_coord_grid(m_Offsets.size()), dst);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::to_map(cv::UMat& dst) const
    {
        cv::add(m_Offsets, view_coord_grid(m_Offsets.size()), dst);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::undistort(const float tolerance)
    {
        LVK_ASSERT_01(tolerance);

        // 2x2 fields have no distortion.
        if(m_Offsets.size() == MinimumSize)
            return;

        // TODO: document and optimize

        // Linear regression formulae taken from..
        // https://www.tutorialspoint.com/regression-analysis-and-the-best-fitting-line-using-cplusplus

        // NOTE: for the verticals we treat the y values as x, vice-versa to
        // keep things consistent and avoid failure case with vertical lines of infinite slope.

        const auto nc = static_cast<float>(cols());
        const auto nr = static_cast<float>(rows());

        const float row_x_sum = static_cast<float>(nc * (nc - 1)) / 2.0f;
        const float col_x_sum = static_cast<float>(nr * (nr - 1)) / 2.0f;

        const float row_x2_sum = (nc * (nc + 1) * (2 * nc + 1)) / 6.0f;
        const float col_x2_sum = (nr * (nr + 1) * (2 * nr + 1)) / 6.0f;

        using LineData = std::tuple<float, float>;
        std::vector<LineData> col_lines(cols(), {0.0f, 0.0f});
        std::vector<LineData> row_lines(rows(), {0.0f, 0.0f});

        for(int r = 0; r < rows(); r++)
        {
            auto& [row_y_sum, row_xy_sum] = row_lines[r];
            for(int c = 0; c < cols(); c++)
            {
                auto& [col_y_sum, col_xy_sum] = col_lines[c];

                const auto& [x, y] = m_Offsets.at<cv::Point2f>(r, c);
                row_xy_sum += y * static_cast<float>(c);
                col_xy_sum += x * static_cast<float>(r);
                row_y_sum += y;
                col_y_sum += x;
            }

            // Once we reach here, row 'r' will be completed and we calculate coefficients.
            const float slope = (nc * row_xy_sum - row_x_sum * row_y_sum)/(nc * row_x2_sum - row_x_sum * row_x_sum);
            const float intercept = (row_y_sum - slope * row_x_sum) / nc;

            // switch the sum variables for calculated coefficient values.
            row_y_sum = slope, row_xy_sum = intercept;
        }

        for(auto& [col_y_sum, col_xy_sum] : col_lines)
        {
            const float slope = (nr * col_xy_sum - col_x_sum * col_y_sum)/(nr * col_x2_sum - col_x_sum * col_x_sum);
            const float intercept = (col_y_sum - slope * col_x_sum) / nr;

            // switch the sum variables for calculated coefficient values.
            col_y_sum = slope, col_xy_sum = intercept;
        }

        write([&](cv::Point2f& offset, const cv::Point& coord){
            auto& [row_slope, row_intercept] = row_lines[coord.y];
            auto& [col_slope, col_intercept] = col_lines[coord.x];

            const auto x = static_cast<float>(coord.x);
            const auto y = static_cast<float>(coord.y);

            const float rigid_x = y * col_slope + col_intercept;
            const float rigid_y = x * row_slope + row_intercept;

            offset.x = tolerance * offset.x + (1.0f - tolerance) * rigid_x;
            offset.y = tolerance * offset.y + (1.0f - tolerance) * rigid_y;
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::apply(const cv::UMat& src, cv::UMat& dst, const bool high_quality) const
    {
        // If we have a minimum size field, we can handle this as a Homography.

        // TODO: test without this
        if(m_Offsets.size() == MinimumSize)
        {
            const auto w = static_cast<float>(src.cols);
            const auto h = static_cast<float>(src.rows);

            const std::array<cv::Point2f, 4> destination = {
                cv::Point2f(0, 0), cv::Point2f(w, 0),
                cv::Point2f(0, h), cv::Point2f(w, h)
            };

            const std::array<cv::Point2f, 4> source = {
                destination[0] + m_Offsets.at<cv::Point2f>(0, 0),
                destination[1] + m_Offsets.at<cv::Point2f>(0, 1),
                destination[2] + m_Offsets.at<cv::Point2f>(1, 0),
                destination[3] + m_Offsets.at<cv::Point2f>(1, 1)
            };

            cv::warpPerspective(
                src,
                dst,
                cv::getPerspectiveTransform(destination.data(), source.data()),
                src.size(),
                cv::WARP_INVERSE_MAP | (high_quality ? cv::INTER_CUBIC : cv::INTER_LINEAR),
                cv::BORDER_CONSTANT
            );
            return;
        }

        cv::resize(m_Offsets, m_WarpMap, src.size(), 0, 0, cv::INTER_LINEAR);
        if(!high_quality)
        {
            cv::add(m_WarpMap, view_coord_grid_gpu(src.size()), m_WarpMap);
            cv::remap(src, dst, m_WarpMap, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        }
        else lvk::remap(src, dst, m_WarpMap, true /* assume yuv */);
    }

//---------------------------------------------------------------------------------------------------------------------

    // TODO: optimize this
    void WarpField::draw(cv::UMat& dst, const cv::Scalar& color, const int thickness) const
    {
        LVK_ASSERT(thickness > 0);
        LVK_ASSERT(!dst.empty());

        const cv::Size2f frame_scaling(
            static_cast<float>(dst.cols) / static_cast<float>(m_Offsets.cols - 1),
            static_cast<float>(dst.rows) / static_cast<float>(m_Offsets.rows - 1)
        );

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
                origin - offset,
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
            m_Offsets.forEach<cv::Point2f>([&](const cv::Point2f& value, const int coord[]){
                operation(value, {coord[1], coord[0]});
            });
        }
        else
        {
            for(int r = 0; r < m_Offsets.rows; r++)
            {
                const auto* row_ptr = m_Offsets.ptr<cv::Point2f>(r);
                for(int c = 0; c < m_Offsets.cols; c++)
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
            m_Offsets.forEach<cv::Point2f>([&](cv::Point2f& value, const int coord[]){
                operation(value, {coord[1], coord[0]});
            });
        }
        else
        {
            for(int r = 0; r < m_Offsets.rows; r++)
            {
                auto* row_ptr = m_Offsets.ptr<cv::Point2f>(r);
                for(int c = 0; c < m_Offsets.cols; c++)
                {
                    operation(row_ptr[c], {c, r});
                }
            }
        }
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

        VirtualGrid partitions(MinimumSize);
        cv::Mat motions, submotions;
        float motion_weight;

        if(motion_hint.has_value())
        {
            const auto warp_transform = motion_hint->invert();

            const cv::Point2f tl = region_offset;
            const cv::Point2f tr(region_offset.x + region_size.width, region_offset.y);
            const cv::Point2f bl(region_offset.x, region_offset.y + region_size.height);
            const cv::Point2f br(tr.x, bl.y);

            motions.create(2, 2, CV_32FC2);
            motions.at<cv::Point2f>(0, 0) = (warp_transform * tl) - tl;
            motions.at<cv::Point2f>(0, 1) = (warp_transform * tr) - tr;
            motions.at<cv::Point2f>(1, 0) = (warp_transform * bl) - bl;
            motions.at<cv::Point2f>(1, 1) = (warp_transform * br) - br;

            motion_weight = 0.5f;
        }
        else
        {
            motions.create(1, 1, CV_32FC2);
            motion_weight = 1.0f;
        }


        while(motions.size() != m_Offsets.size())
        {
            submotions.create(
                std::min(motions.rows * 2, m_Offsets.rows),
                std::min(motions.cols * 2, m_Offsets.cols),
                CV_32FC2
            );

            const cv::Size2f submotion_cell_size(
                region_size.width / static_cast<float>(submotions.cols - 1),
                region_size.height / static_cast<float>(submotions.rows - 1)
            );

            partitions.align(submotions.size(), cv::Rect2f(
                region_offset - (submotion_cell_size / 2.0f),
                cv::Size2f(submotions.size()) * submotion_cell_size
            ));

            // Re-accumulate all the motions into the new submotions grid.
            cv::resize(motions, submotions, submotions.size(), 0, 0, cv::INTER_LINEAR);
            for(size_t i = 0; i < origin_points.size(); i++)
            {
                const cv::Point2f& origin_point = origin_points[i];
                const cv::Point2f& warped_point = warped_points[i];
                auto warp_motion = origin_point - warped_point;

                if(const auto key = partitions.try_key_of(warped_point); key.has_value())
                {
                    auto& motion_estimate = submotions.at<cv::Point2f>(
                        static_cast<int>(key->y), static_cast<int>(key->x)
                    );

                    motion_estimate.x += motion_weight * static_cast<float>(sign(warp_motion.x - motion_estimate.x));
                    motion_estimate.y += motion_weight * static_cast<float>(sign(warp_motion.y - motion_estimate.y));
                }
            }
            cv::medianBlur(submotions, motions, 3);

            motion_weight /= 2.0f;
        }

        m_Offsets = std::move(motions);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::set_identity()
    {
        m_Offsets.setTo(cv::Scalar(0.0f, 0.0f));
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::set_to(const cv::Point2f& motion)
    {
        // NOTE: we invert the motion as the warp is specified backwards.
        m_Offsets.setTo(cv::Scalar(-motion.x, -motion.y));
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::set_to(const Homography& motion, const cv::Size2f& field_scale)
    {
        const cv::Size2f point_scaling(
            field_scale.width / static_cast<float>(m_Offsets.cols - 1),
            field_scale.height / static_cast<float>(m_Offsets.rows - 1)
        );

        const Homography inverse_warp = motion.invert();
        write([&](cv::Point2f& offset, const cv::Point& coord){
            const auto sample_point = cv::Point2f(coord) * point_scaling;
            offset = (inverse_warp * sample_point) - sample_point;
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::scale(const cv::Size2f& scaling_factor, const cv::Size2f& field_scale)
    {
        const cv::Size2f inverse_scaling = 1.0f / scaling_factor;
        cv::add(m_Offsets, view_field_coord_grid(field_scale), m_Offsets);
        cv::multiply(m_Offsets, cv::Scalar(inverse_scaling.width, inverse_scaling.height), m_Offsets);
        cv::subtract(m_Offsets, view_field_coord_grid(field_scale), m_Offsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::crop_in(const cv::Rect2f& region, const cv::Size2f& field_scale)
    {
        // Move the crop region to the top left of the field, then upscale to fit.
        cv::add(m_Offsets, cv::Scalar(region.x, region.y), m_Offsets);
        scale(cv::Size2f(field_scale) / cv::Size2f(region.size()), field_scale);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::rotate(const float degrees, const cv::Size2f& field_scale)
    {
        const cv::Point2f center = field_scale / 2.0f;

        // Rotate the field coord grid, then add its offset to the warp field.
        cv::warpAffine(
            view_field_coord_grid(field_scale),
            m_ResultsBuffer,
            cv::getRotationMatrix2D(center, degrees, 1.0f),
            m_Offsets.size()
        );

        cv::subtract(view_field_coord_grid(field_scale), m_ResultsBuffer, m_ResultsBuffer);
        cv::add(m_ResultsBuffer, m_Offsets, m_Offsets);
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
        cv::addWeighted(m_Offsets, (1.0f - field_weight), field.m_Offsets, field_weight, 0.0, m_Offsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::blend(const float weight_1, const float weight_2, const WarpField& field)
    {
        cv::addWeighted(m_Offsets, weight_1, field.m_Offsets, weight_2, 0.0, m_Offsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::combine(const WarpField& field, const float scaling)
    {
        cv::scaleAdd(field.m_Offsets, scaling, m_Offsets, m_Offsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    // NOTE: This returns a view into a shared cache, do not modify the value.
    const cv::Mat WarpField::view_coord_grid(const cv::Size& resolution)
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

            cv::UMat x_plane, y_plane;
            cv::resize(row_values, x_plane, coord_grid.size(), 0, 0, cv::INTER_NEAREST_EXACT);
            cv::resize(col_values, y_plane, coord_grid.size(), 0, 0, cv::INTER_NEAREST_EXACT);
            cv::merge(std::vector{x_plane, y_plane}, coord_grid);
        }

        return coord_grid(cv::Rect({0,0}, resolution));
    }

//---------------------------------------------------------------------------------------------------------------------

    // NOTE: This returns a view into a shared cache, do not modify the value.
    const cv::UMat WarpField::view_coord_grid_gpu(const cv::Size& resolution)
    {
        // Since this is a repeated operation that often results in the same output,
        // we cache a copy of the grid in thread storage and return a view onto it.
        // If the grid is too small, it must be remade to match the larger size.

        thread_local cv::UMat coord_grid;
        if(resolution.width > coord_grid.cols || resolution.height > coord_grid.rows)
        {
            view_coord_grid(resolution).copyTo(coord_grid);
        }

        return coord_grid(cv::Rect({0,0}, resolution));
    }

//---------------------------------------------------------------------------------------------------------------------

    // NOTE: This returns a view into a shared cache, do not modify the value.
    const cv::Mat WarpField::view_field_coord_grid(const cv::Size2f& field_scale)
    {
        // This operation is likely to be required multiple times
        // with the same scale so we cache it for better performance.
        if(field_scale != m_FieldGridScale)
        {
            cv::multiply(
                view_coord_grid(m_Offsets.size()),
                cv::Scalar(
                    field_scale.width / static_cast<float>(m_Offsets.cols),
                    field_scale.height / static_cast<float>(m_Offsets.rows)
                ),
                m_FieldGridCache
            );

            m_FieldGridScale = field_scale;
        }
        return m_FieldGridCache;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField& WarpField::operator=(WarpField&& other) noexcept
    {
        m_Offsets = std::move(other.m_Offsets);

        return *this;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField& WarpField::operator=(const WarpField& other)
    {
        m_Offsets = other.m_Offsets.clone();

        return *this;
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator+=(const WarpField& other)
    {
        LVK_ASSERT(size() == other.size());

        cv::add(m_Offsets, other.m_Offsets, m_Offsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator-=(const WarpField& other)
    {
        LVK_ASSERT(size() == other.size());

        cv::subtract(m_Offsets, other.m_Offsets, m_Offsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator*=(const WarpField& other)
    {
        cv::multiply(m_Offsets, other.m_Offsets, m_Offsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator+=(const cv::Point2f& offset)
    {
        cv::add(m_Offsets, cv::Scalar(offset.x, offset.y), m_Offsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator-=(const cv::Point2f& offset)
    {
        cv::subtract(m_Offsets, cv::Scalar(offset.x, offset.y), m_Offsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator*=(const cv::Size2f& scaling)
    {
        cv::multiply(m_Offsets, cv::Scalar(scaling.width, scaling.height), m_Offsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator/=(const cv::Size2f& scaling)
    {
        LVK_ASSERT(scaling.width != 0.0f && scaling.height != 0.0f);

        cv::divide(m_Offsets, cv::Scalar(scaling.width, scaling.height), m_Offsets);
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator*=(const float scaling)
    {
        m_Offsets *= scaling;
    }

//---------------------------------------------------------------------------------------------------------------------

    void WarpField::operator/=(const float scaling)
    {
        LVK_ASSERT(scaling != 0.0f);

        m_Offsets /= scaling;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator+(const WarpField& left, const WarpField& right)
    {
        LVK_ASSERT(left.size() == right.size());

        cv::Mat result = left.offsets() + right.offsets();
        return WarpField(std::move(result), true);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator-(const WarpField& left, const WarpField& right)
    {
        LVK_ASSERT(left.size() == right.size());

        cv::Mat result = left.offsets() - right.offsets();
        return WarpField(std::move(result), true);
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
        return WarpField(std::move(result), true);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator-(const WarpField& left, const cv::Point2f& right)
    {
        cv::Mat result = left.offsets() - cv::Scalar(right.x, right.y);
        return WarpField(std::move(result), true);
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
        return WarpField(std::move(result), true);
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
        return WarpField(std::move(result), true);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField operator/(const float scaling, const WarpField& field)
    {
        return field / scaling;
    }

//---------------------------------------------------------------------------------------------------------------------
}