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
#include <vector>

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
        using spatial_key = cv::Point_<size_t>;
        using iterator = std::vector<std::pair<spatial_key, T>>::iterator;
        using const_iterator = std::vector<std::pair<spatial_key, T>>::const_iterator;
    public:

        explicit SpatialMap(const cv::Size resolution);

        SpatialMap(const cv::Size resolution, const cv::Rect input_region);

        void rescale(const cv::Size resolution);

        void rescale(const cv::Size resolution, const cv::Rect input_region);


        T& place_at(const spatial_key key, const T& item);

        template<typename ...Args>
        T& emplace_at(const spatial_key key, Args... args);


        template<typename P>
        T& place(const cv::Point_<P>& position, const T& item);

        template<typename P>
        bool try_place(const cv::Point_<P>& position, const T& item);


        template<typename P, typename ...Args>
        T& emplace(const cv::Point_<P>& position, Args... args);

        template<typename P, typename ...Args>
        bool try_emplace(const cv::Point_<P>& position, Args... args);


        void remove(const spatial_key key);

        bool try_remove(const spatial_key key);


        T& at(const spatial_key key);

        const T& at(const spatial_key key) const;

        template<typename P>
        T& operator[](const cv::Point_<P>& position);


        template<typename P>
        spatial_key key_of(const cv::Point_<P>& position) const;

        template<typename P>
        bool within_bounds(const cv::Point_<P>& position) const;

        bool contains(const spatial_key key) const;

        template<typename P = float>
        cv::Point_<P> distribution_centroid() const;

        double distribution_quality() const;

        const cv::Size_<float>& key_size() const;

        const cv::Rect& input_region() const;

        const cv::Size& resolution() const;

        size_t capacity() const;

        bool is_empty() const;

        size_t size() const;

        size_t rows() const;

        size_t cols() const;

        void clear();

        iterator begin();

        const_iterator begin() const;

        const_iterator cbegin() const;

        iterator end();

        const_iterator end() const;

        const_iterator cend() const;

    private:

        template<typename P>
        static spatial_key simplify_key(
            const cv::Point_<P>& point,
            const cv::Size_<float>& key_size
        );

        static size_t resolve_spatial_key(
            const spatial_key key,
            const cv::Size resolution
        );

        bool is_key_valid(const spatial_key key) const;

        size_t& fetch_data_link(const spatial_key key);

        size_t fetch_data_link(const spatial_key key) const;

        bool is_data_link_empty(const size_t link) const;

        void clear_data_link(size_t& link);

    private:
        constexpr static size_t m_EmptySymbol = std::numeric_limits<size_t>::max();

        cv::Rect m_InputRegion;
        cv::Size m_MapResolution;
        cv::Size_<float> m_KeySize;

        std::vector<size_t> m_Map;
        std::vector<std::pair<spatial_key, T>> m_Data;
    };


    template<typename T>
    using SpatialList = SpatialMap<std::vector<T>>;

    template<typename P>
    using SpatialSet = SpatialMap<cv::Point_<P>>;

}

#include "SpatialMap.tpp"

