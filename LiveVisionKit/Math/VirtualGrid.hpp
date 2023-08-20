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
#include <opencv2/opencv.hpp>

namespace lvk
{
    using SpatialKey = cv::Point_<size_t>;

    class VirtualGrid
    {
    public:

        explicit VirtualGrid(const cv::Size& size);

        VirtualGrid(const cv::Size& size, const cv::Rect2f& alignment);

        VirtualGrid(const VirtualGrid& grid);


        void resize(const cv::Size& size);

        const cv::Size& size() const;

        int cols() const;

        int rows() const;


        void align(const cv::Rect2f& region);

        void align(const cv::Size& size, const cv::Rect2f& region);


        const cv::Rect2f& alignment() const;

        const cv::Size2f& key_size() const;


        cv::Mat make_grid() const;

        cv::Mat make_aligned_grid() const;


        bool test_key(const SpatialKey& key) const;

        size_t key_to_index(const SpatialKey& key) const;

        SpatialKey index_to_key(const size_t index) const;


        bool test_point(const cv::Point2f& point) const;

        SpatialKey key_of(const cv::Point2f& point) const;

        std::optional<SpatialKey> try_key_of(const cv::Point2f& point) const;

        // TODO: rename these
        cv::Point2f key_to_point(const SpatialKey& key) const;

        cv::Point2f index_to_point(const size_t index) const;



        void for_each(const std::function<void(const int index, const cv::Point& coord)>& operation) const;

        void for_each_aligned(const std::function<void(const int index, const cv::Point2f& coord)>& operation) const;

    private:
        cv::Size m_Resolution;
        cv::Rect2f m_Alignment;
        cv::Size2f m_KeySize;
    };

}
