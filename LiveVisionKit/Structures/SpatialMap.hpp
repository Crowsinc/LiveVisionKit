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

#include <opencv2/opencv.hpp>
#include <optional>
#include <vector>

namespace lvk
{
    // NOTE:
    // * position = point on the input region, which is mapped onto the map resolution
    // * key = discrete point on the map resolution
    // A position becomes a key once an item has been placed.

    using SpatialKey = cv::Point_<size_t>;

    template<typename T>
    class SpatialMap
    {
    public:
        using iterator = std::vector<std::pair<SpatialKey, T>>::iterator;
        using const_iterator = std::vector<std::pair<SpatialKey, T>>::const_iterator;
    public:

        explicit SpatialMap(const cv::Size resolution);

        SpatialMap(const cv::Size resolution, const cv::Rect& input_region);

        SpatialMap(const SpatialMap&& other) noexcept;

        SpatialMap(const SpatialMap& other);


        SpatialMap& operator=(SpatialMap&& other) noexcept;

        SpatialMap& operator=(const SpatialMap& other);


        void rescale(const cv::Size resolution);

        const cv::Size& resolution() const;

        size_t capacity() const;

        size_t rows() const;

        size_t cols() const;

        size_t size() const;

        bool is_full() const;

        bool is_empty() const;


        void align(const cv::Rect& input_region);

        const cv::Rect& alignment() const;

        const cv::Size2f& key_size() const;


        T& place_at(const SpatialKey key, const T& item);

        template<typename ...Args>
        T& emplace_at(const SpatialKey key, Args... args);


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



        void remove(const SpatialKey key);

        bool try_remove(const SpatialKey key);

        void clear();


        T& at(const SpatialKey key);

        const T& at(const SpatialKey key) const;

        T& at_or(const SpatialKey key, T& value);

        const T& at_or(const SpatialKey key, const T& value) const;

        template<typename P>
        T& operator[](const cv::Point_<P>& position);


        template<typename P>
        SpatialKey key_of(const cv::Point_<P>& position) const;

        template<typename P>
        std::optional<SpatialKey> try_key_of(const cv::Point_<P>& position) const;

        template<typename P>
        bool within_bounds(const cv::Point_<P>& position) const;

        bool contains(const SpatialKey key) const;


        template<typename P = float>
        cv::Point_<P> distribution_centroid() const;

        double distribution_quality() const;


        iterator begin();

        const_iterator begin() const;

        const_iterator cbegin() const;

        iterator end();

        const_iterator end() const;

        const_iterator cend() const;

    private:

        template<typename P>
        static SpatialKey simplify_key(
            const cv::Point_<P>& point,
            const cv::Size2f& key_size
        );

        static size_t map_key_to_index(
            const SpatialKey key,
            const cv::Size resolution
        );

        static SpatialKey map_index_to_key(
            const size_t index,
            const cv::Size resolution
        );

        bool is_key_valid(const SpatialKey key) const;

        size_t& fetch_data_link(const SpatialKey key);

        size_t fetch_data_link(const SpatialKey key) const;

        bool is_data_link_empty(const size_t link) const;

        void clear_data_link(size_t& link);

    private:
        constexpr static size_t m_EmptySymbol = std::numeric_limits<size_t>::max();

        cv::Size2f m_KeySize;
        cv::Rect m_InputRegion;
        cv::Size m_MapResolution;

        std::vector<size_t> m_Map;
        std::vector<std::pair<SpatialKey, T>> m_Data;
    };

    template<typename T>
    using SpatialList = SpatialMap<std::vector<T>>;

    template<typename P>
    using SpatialSet = SpatialMap<cv::Point_<P>>;

}

#include "SpatialMap.tpp"

