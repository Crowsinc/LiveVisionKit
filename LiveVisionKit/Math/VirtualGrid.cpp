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

    bool VirtualGrid::test_key(const SpatialKey& key) const
    {
        return key.x < m_Resolution.width && key.y < m_Resolution.height;
    }

//---------------------------------------------------------------------------------------------------------------------

    size_t VirtualGrid::key_to_index(const SpatialKey& key) const
    {
        LVK_ASSERT(test_key(key));

        return index_2d(key.x, key.y, m_Resolution.width);
    }

//---------------------------------------------------------------------------------------------------------------------

    SpatialKey VirtualGrid::index_to_key(const size_t index) const
    {
        return inv_index_2d(index, m_Resolution.width);
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
}