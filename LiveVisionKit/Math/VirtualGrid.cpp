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

#include "VirtualGrid.hpp"

#include "Directives.hpp"
#include "Functions/Math.hpp"

namespace lvk
{
//---------------------------------------------------------------------------------------------------------------------

    VirtualGrid::VirtualGrid(const cv::Size& size)
        : m_Resolution(size),
          m_Alignment({0, 0}, size),
          m_KeySize(1, 1)
    {
        LVK_ASSERT(size.width > 0 && size.height > 0);
    }

//---------------------------------------------------------------------------------------------------------------------

    VirtualGrid::VirtualGrid(const cv::Size& size, const cv::Rect2f& alignment)
        : m_Resolution(size)
    {
        align(alignment);
    }

//---------------------------------------------------------------------------------------------------------------------

    VirtualGrid::VirtualGrid(const VirtualGrid& grid)
        : m_Resolution(grid.m_Resolution),
          m_Alignment(grid.m_Alignment),
          m_KeySize(grid.m_KeySize)
    {}

//---------------------------------------------------------------------------------------------------------------------

    void VirtualGrid::resize(const cv::Size& size)
    {
        LVK_ASSERT(size.width > 0 && size.height > 0);

        m_Resolution = size;
        align(m_Alignment);
    }

//---------------------------------------------------------------------------------------------------------------------

    const cv::Size& VirtualGrid::size() const
    {
        return m_Resolution;
    }

//---------------------------------------------------------------------------------------------------------------------

    int VirtualGrid::cols() const
    {
        return m_Resolution.width;
    }

//---------------------------------------------------------------------------------------------------------------------

    int VirtualGrid::rows() const
    {
        return m_Resolution.height;
    }

//---------------------------------------------------------------------------------------------------------------------

    void VirtualGrid::align(const cv::Rect2f& region)
    {
        m_Alignment = region;

        m_KeySize.width = region.width / static_cast<float>(m_Resolution.width);
        m_KeySize.height = region.height / static_cast<float>(m_Resolution.height);
    }

//---------------------------------------------------------------------------------------------------------------------

    void VirtualGrid::align(const cv::Size& size, const cv::Rect2f& region)
    {
        resize(size);
        align(region);
    }

//---------------------------------------------------------------------------------------------------------------------

    const cv::Rect2f& VirtualGrid::alignment() const
    {
        return m_Alignment;
    }

//---------------------------------------------------------------------------------------------------------------------

    const cv::Size2f& VirtualGrid::key_size() const
    {
        return m_KeySize;
    }

//---------------------------------------------------------------------------------------------------------------------

    cv::Mat VirtualGrid::make_grid() const
    {
        cv::Mat coord_grid(m_Resolution, CV_32FC2);

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

        return coord_grid;
    }

//---------------------------------------------------------------------------------------------------------------------

    cv::Mat VirtualGrid::make_aligned_grid() const
    {
        auto grid = make_grid();
        cv::multiply(grid, cv::Scalar(m_KeySize.width, m_KeySize.height), grid);
        return grid;
    }

//---------------------------------------------------------------------------------------------------------------------

    bool VirtualGrid::test_key(const SpatialKey& key) const
    {
        return key.x < m_Resolution.width && key.y < m_Resolution.height;
    }

//---------------------------------------------------------------------------------------------------------------------

    size_t VirtualGrid::key_to_index(const SpatialKey& key) const
    {
        LVK_ASSERT(test_key(key));

        return index_2d<size_t>(key.x, key.y, m_Resolution.width);
    }

//---------------------------------------------------------------------------------------------------------------------

    SpatialKey VirtualGrid::index_to_key(const size_t index) const
    {
        return inv_index_2d<size_t>(index, m_Resolution.width);
    }

//---------------------------------------------------------------------------------------------------------------------

    bool VirtualGrid::test_point(const cv::Point2f& point) const
    {
        return m_Alignment.contains(point);
    }

//---------------------------------------------------------------------------------------------------------------------

    SpatialKey VirtualGrid::key_of(const cv::Point2f& point) const
    {
        return SpatialKey(
            static_cast<size_t>((point.x - m_Alignment.x) / m_KeySize.width),
            static_cast<size_t>((point.y - m_Alignment.y) / m_KeySize.height)
        );
    }

//---------------------------------------------------------------------------------------------------------------------

    std::optional<SpatialKey> VirtualGrid::try_key_of(const cv::Point2f& point) const
    {
        if(test_point(point))
            return key_of(point);
        else
            return std::nullopt;
    }

//---------------------------------------------------------------------------------------------------------------------

    void VirtualGrid::for_each(
        const std::function<void(const int index, const cv::Point& coord)>& operation
    ) const
    {
        // TODO: add parallel option.
        int index = 0;
        for(int r = 0; r < m_Resolution.height; r++)
        {
            for(int c = 0; c < m_Resolution.width; c++)
            {
                operation(
                    index++,
                    cv::Point(c, r)
                );
            }
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    void VirtualGrid::for_each_aligned(
        const std::function<void(const int index, const cv::Point2f& coord)>& operation
    ) const
    {
        // TODO: add parallel option.
        int index = 0;
        for(int r = 0; r < m_Resolution.height; r++)
        {
            for(int c = 0; c < m_Resolution.width; c++)
            {
                operation(
                    index++,
                    cv::Point2f(static_cast<float>(c) * m_KeySize.width, static_cast<float>(r) * m_KeySize.height)
                );
            }
        }
    }

//---------------------------------------------------------------------------------------------------------------------

}