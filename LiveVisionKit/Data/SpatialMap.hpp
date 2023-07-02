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

#include "Math/VirtualGrid.hpp"

#include <opencv2/opencv.hpp>
#include <optional>
#include <vector>
#include <tuple>

namespace lvk
{
    // NOTE:
    // * position = point on the input region, which is mapped onto the map resolution
    // * key = discrete point on the map resolution
    // A position becomes a key once an item has been placed.

    template<typename T>
    class SpatialMap
    {
    public:
        using iterator = std::vector<std::pair<SpatialKey, T>>::iterator;
        using const_iterator = std::vector<std::pair<SpatialKey, T>>::const_iterator;
    public:

        explicit SpatialMap(const cv::Size& resolution);

        SpatialMap(const cv::Size& resolution, const cv::Rect& input_region);

        SpatialMap(const SpatialMap&& other) noexcept;

        SpatialMap(const SpatialMap& other);


        SpatialMap& operator=(SpatialMap&& other) noexcept;

        SpatialMap& operator=(const SpatialMap& other);


        void reshape(const cv::Size& resolution);

        const cv::Size& resolution() const;

        size_t capacity() const;

        size_t size() const;

        size_t area() const;

        int rows() const;

        int cols() const;

        bool is_full() const;

        bool is_empty() const;


        void align(const cv::Rect2f& input_region);

        const cv::Rect2f& alignment() const;

        const cv::Size2f& key_size() const;


        T& place_at(const SpatialKey& key, const T& item);

        template<typename ...Args>
        T& emplace_at(const SpatialKey& key, Args... args);


        template<typename P>
        T& place(const cv::Point_<P>& position, const T& item);

        template<typename P>
        bool try_place(const cv::Point_<P>& position, const T& item);


        template<typename P, typename ...Args>
        T& emplace(const cv::Point_<P>& position, Args... args);

        template<typename P, typename ...Args>
        bool try_emplace(const cv::Point_<P>& position, Args... args);


        // Sets all slots to the value
        void set_to(const T& value);

        template<typename... Args>
        void set_to(Args... args);


        // Fills all empty slots to the value
        void fill_out(const T& value);

        template<typename... Args>
        void fill_out(Args... args);



        void remove(const SpatialKey& key);

        bool try_remove(const SpatialKey& key);

        void clear();


        T& at(const SpatialKey& key);

        const T& at(const SpatialKey& key) const;

        T& at_or(const SpatialKey& key, T& value);

        const T& at_or(const SpatialKey& key, const T& value) const;

        template<typename P>
        T& operator[](const cv::Point_<P>& position);


        template<typename P>
        SpatialKey key_of(const cv::Point_<P>& position) const;

        template<typename P>
        std::optional<SpatialKey> try_key_of(const cv::Point_<P>& position) const;

        template<typename P>
        bool within_bounds(const cv::Point_<P>& position) const;

        bool contains(const SpatialKey& key) const;


        template<typename P = float>
        cv::Point_<P> distribution_centroid() const;

        float distribution_quality() const;


        iterator begin();

        const_iterator begin() const;

        const_iterator cbegin() const;


        iterator end();

        const_iterator end() const;

        const_iterator cend() const;

    private:

        size_t& fetch_data_link(const SpatialKey& key);

        size_t fetch_data_link(const SpatialKey& key) const;

        bool is_data_link_empty(const size_t link) const;

        void clear_data_link(size_t& link);

    private:
        constexpr static size_t m_EmptySymbol = std::numeric_limits<size_t>::max();

        VirtualGrid m_VirtualGrid;
        std::vector<size_t> m_Map;
        std::vector<std::pair<SpatialKey, T>> m_Data;
    };

    template<typename T>
    using SpatialList = SpatialMap<std::vector<T>>;

    template<typename P>
    using SpatialSet = SpatialMap<cv::Point_<P>>;

}

#include "SpatialMap.tpp"

