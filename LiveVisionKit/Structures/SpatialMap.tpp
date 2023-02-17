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

#include "SpatialMap.hpp"

#include "Diagnostics/Directives.hpp"
#include "Utility/Algorithm.hpp"
#include "Math/Math.hpp"

#include <array>

namespace lvk
{
//---------------------------------------------------------------------------------------------------------------------

    // Max capacity to reserve in the data buffer when resizing the map.
    inline constexpr size_t MAX_DATA_RESERVE = 512;

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline SpatialMap<T>::SpatialMap(const cv::Size& resolution)
        : m_VirtualGrid(resolution)
    {
        rescale(resolution);
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline SpatialMap<T>::SpatialMap(const cv::Size& resolution, const cv::Rect& input_region)
    {
        rescale(resolution);
        align(input_region);
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline SpatialMap<T>::SpatialMap(const SpatialMap<T>&& other) noexcept
        : m_Map(std::move(other.m_Map)),
          m_Data(std::move(other.m_Data)),
          m_VirtualGrid(other.m_VirtualGrid)
    {}

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline SpatialMap<T>::SpatialMap(const SpatialMap<T>& other)
        : m_Map(other.m_Map),
          m_Data(other.m_Data),
          m_VirtualGrid(other.m_VirtualGrid)
    {}

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    SpatialMap<T>& SpatialMap<T>::operator=(SpatialMap&& other) noexcept
    {
        m_Map = std::move(other.m_Map);
        m_Data = std::move(other.m_Data);
        m_VirtualGrid = other.m_VirtualGrid;

        return *this;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    SpatialMap<T>& SpatialMap<T>::operator=(const SpatialMap& other)
    {
        m_Map = other.m_Map;
        m_Data = other.m_Data;
        m_VirtualGrid = other.m_VirtualGrid;

        return *this;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline void SpatialMap<T>::rescale(const cv::Size& resolution)
    {
        LVK_ASSERT(resolution.width >= 1);
        LVK_ASSERT(resolution.height >= 1);

        if(resolution != m_VirtualGrid.size() || m_Map.empty())
        {
            m_VirtualGrid.resize(resolution);

            m_Map.clear();
            m_Map.resize(resolution.area(), m_EmptySymbol);
            m_Data.reserve(std::min(m_Map.size(), MAX_DATA_RESERVE));

            // Re-map all the elements which still fit in the new resolution.
            for(size_t i = 0; i < m_Data.size(); i++)
            {
                // If the key is no longer valid erase the item,
                // otherwise set its data link in the new map.
                const auto& key = m_Data[i].first;
                if(!m_VirtualGrid.test_key(key))
                {
                    fast_erase(m_Data, i);
                    if(i > 0) i--;
                }
                else fetch_data_link(key) = i;
            }

        }
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline const cv::Size& SpatialMap<T>::resolution() const
    {
        return m_VirtualGrid.size();
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline size_t SpatialMap<T>::capacity() const
    {
        return m_Map.size();
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline size_t SpatialMap<T>::size() const
    {
        return m_Data.size();
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    int SpatialMap<T>::rows() const
    {
        return m_VirtualGrid.rows();
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    int SpatialMap<T>::cols() const
    {
        return m_VirtualGrid.cols();
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline bool SpatialMap<T>::is_full() const
    {
        return m_Data.size() == capacity();
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline bool SpatialMap<T>::is_empty() const
    {
        return m_Data.empty();
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline void SpatialMap<T>::align(const cv::Rect2f& input_region)
    {
        m_VirtualGrid.align(input_region);
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline const cv::Rect2f& SpatialMap<T>::alignment() const
    {
        return m_VirtualGrid.alignment();
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline const cv::Size2f& SpatialMap<T>::key_size() const
    {
        return m_VirtualGrid.key_size();
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline T& SpatialMap<T>::place_at(const SpatialKey& key, const T& item)
    {
        LVK_ASSERT(m_VirtualGrid.test_key(key));

        // If the key is empty, generate a new pointer,
        // otherwise we just replace the existing item.

        size_t& data_link = fetch_data_link(key);
        if(is_data_link_empty(data_link))
        {
            data_link = m_Data.size();
            return m_Data.emplace_back(key, item).second;
        }
        else
        {
            auto& stored_data = m_Data[data_link].second;
            stored_data = item;
            return stored_data;
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    template<typename... Args>
    inline T& SpatialMap<T>::emplace_at(const SpatialKey& key, Args... args)
    {
        LVK_ASSERT(m_VirtualGrid.test_key(key));

        // If the key is empty, generate a new pointer,
        // otherwise we just replace the existing item.

        // TODO: this isn't much of an optimization compared to the
        // place function, maybe there is something more we can do?

        size_t& data_link = fetch_data_link(key);
        if(is_data_link_empty(data_link))
        {
            data_link = m_Data.size();
            return m_Data.emplace_back(key, T{args...}).second;
        }
        else
        {
            auto& stored_data = m_Data[data_link].second;
            stored_data = T{args...};
            return stored_data;
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    template<typename P>
    inline T& SpatialMap<T>::place(const cv::Point_<P>& position, const T& item)
    {
        LVK_ASSERT(within_bounds(position));

        return place_at(key_of(position), item);
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    template<typename P>
    inline bool SpatialMap<T>::try_place(const cv::Point_<P>& position, const T& item)
    {
        if(within_bounds(position))
        {
            place(position, item);
            return true;
        }
        return false;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    template<typename P, typename... Args>
    inline T& SpatialMap<T>::emplace(const cv::Point_<P>& position, Args... args)
    {
        LVK_ASSERT(within_bounds(position));

        return emplace_at(key_of(position), args...);
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    template<typename P, typename... Args>
    inline bool SpatialMap<T>::try_emplace(const cv::Point_<P>& position, Args... args)
    {
        if(within_bounds(position))
        {
            emplace(position, args...);
            return true;
        }
        return false;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline void SpatialMap<T>::set_to(const T& value)
    {
        // Fill all slots with the given value in a contiguous
        // manner for potential cache hit efficiency boosts later.

        // Avoid running destructors by re-assigning any existing elements.
        size_t index = 0;
        for(; index < m_Data.size(); index++)
        {
            m_Map[index] = index;

            auto& stored_data = m_Data[index];
            stored_data.first = m_VirtualGrid.index_to_key(index);
            stored_data.second = value;
        }

        for(; index < m_Map.size(); index++)
        {
            m_Map[index] = index;

            m_Data.emplace_back(
                m_VirtualGrid.index_to_key(index),
                value
            );
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    template<typename... Args>
    inline void SpatialMap<T>::set_to(Args... args)
    {
        // Fill all slots with the given value in a contiguous
        // manner for potential cache hit efficiency boosts later.

        // Avoid running destructors by re-assigning any existing elements.
        size_t index = 0;
        for(; index < m_Data.size(); index++)
        {
            m_Map[index] = index;

            auto& stored_data = m_Data[index];
            stored_data.first = m_VirtualGrid.index_to_key(index);
            stored_data.second = T{args...};
        }

        for(; index < m_Map.size(); index++)
        {
            m_Map[index] = index;

            m_Data.emplace_back(
                m_VirtualGrid.index_to_key(index),
                T{args...}
            );
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline void SpatialMap<T>::fill_out(const T& value)
    {
        // Fill all empty slots with the given value
        for(size_t index = 0; index < m_Map.size(); index++)
        {
            size_t& data_link = m_Map[index];
            if(is_data_link_empty(data_link))
            {
                const SpatialKey key = m_VirtualGrid.index_to_key(index);

                data_link = m_Data.size();
                m_Data.emplace_back(key, value);
            }
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    template<typename... Args>
    inline void SpatialMap<T>::fill_out(Args... args)
    {
        // Emplace all empty slots using the given arguments
        for(size_t index = 0; index < m_Map.size(); index++)
        {
            size_t& data_link = m_Map[index];
            if(is_data_link_empty(data_link))
            {
                const SpatialKey key = m_VirtualGrid.index_to_key(index);

                data_link = m_Data.size();
                m_Data.emplace_back(key, T{args...});
            }
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline void SpatialMap<T>::remove(const SpatialKey& key)
    {
        LVK_ASSERT(contains(key));

        // To remove an item quickly we will swap it with the last added
        // item, which will be at the end of the data vector. We can then
        // pop it off without having to shuffle any items. The item which
        // used to be last will have its data pointer adjusted as necessary.

        size_t& item_data_link = fetch_data_link(key);

        const auto& [replace_key, replace_item] = m_Data.back();
        if(key != replace_key)
        {
            // Swap the replacement item to the new position
            std::swap(m_Data[item_data_link], replace_item);

            size_t& replace_data_link = fetch_data_link(key);
            replace_data_link = item_data_link;
        }

        // Remove the requested item
        m_Data.pop_back();
        clear_data_link(item_data_link);
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline bool SpatialMap<T>::try_remove(const SpatialKey& key)
    {
        if(contains(key))
        {
            remove(key);
            return true;
        }
        return false;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline void SpatialMap<T>::clear()
    {
        m_Data.clear();

        std::fill(m_Map.begin(), m_Map.end(), m_EmptySymbol);
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline T& SpatialMap<T>::at(const SpatialKey& key)
    {
        LVK_ASSERT(contains(key));

        return m_Data[fetch_data_link(key)].second;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline const T& SpatialMap<T>::at(const SpatialKey& key) const
    {
        LVK_ASSERT(contains(key));

        return m_Data[fetch_data_link(key)].second;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline T& SpatialMap<T>::at_or(const SpatialKey& key, T& value)
    {
        LVK_ASSERT(m_VirtualGrid.test_key(key));

        const size_t data_link = fetch_data_link(key);
        return is_data_link_empty(data_link) ? value : m_Data[data_link].second;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline const T& SpatialMap<T>::at_or(const SpatialKey& key, const T& value) const
    {
        LVK_ASSERT(m_VirtualGrid.test_key(key));

        const size_t data_link = fetch_data_link(key);
        return is_data_link_empty(data_link) ? value : m_Data[data_link].second;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    template<typename P>
    inline T& SpatialMap<T>::operator[](const cv::Point_<P>& position)
    {
        const SpatialKey key = key_of(position);

        if(!contains(key))
            return emplace_at(key);
        else return at(key);
    }

    //---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    template<typename P>
    inline bool SpatialMap<T>::within_bounds(const cv::Point_<P>& position) const
    {
        // NOTE: The bottom and right edges of the region are exclusive.
        // That is, spatial indexing starts counting from zero just like arrays.
        return m_VirtualGrid.test_point(position);
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    template<typename P>
    inline SpatialKey SpatialMap<T>::key_of(const cv::Point_<P>& position) const
    {
        LVK_ASSERT(within_bounds(position));

        return m_VirtualGrid.key_of(position);
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    template<typename P>
    inline std::optional<SpatialKey> SpatialMap<T>::try_key_of(const cv::Point_<P>& position) const
    {
        if(within_bounds(position))
            return key_of(position);
        else
            return std::nullopt;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline bool SpatialMap<T>::contains(const SpatialKey& key) const
    {
        LVK_ASSERT(m_VirtualGrid.test_key(key));

        return !is_data_link_empty(fetch_data_link(key));
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    template<typename P>
    inline cv::Point_<P> SpatialMap<T>::distribution_centroid() const
    {
        if(m_Data.empty())
            return {};

        cv::Point_<P> centroid;
        for(const auto& [point, value] : m_Data)
            centroid += cv::Point_<P>(point);

        centroid /= static_cast<P>(m_Data.size());

        return centroid;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline double SpatialMap<T>::distribution_quality() const
    {
        if(is_empty()) return 1.0;

        // To determine the distribution quality we will split the map into a grid of 4x4 sectors.
        // We then compare the number of items in each sector against the ideal distribution, where
        // each sector has an equal item count. We then get a percentage measure of the total number
        // of excess items in each sector, which are badly distributed, and the quality is simply
        // its inverse. If the map resolution is less than or equal to 4x4, then this technique will
        // not be meaningful so we instead approximate it by taking the map load.

        constexpr int sectors = 4;

        if(cols() <= sectors || rows() <= sectors)
            return static_cast<double>(m_Data.size()) / static_cast<double>(m_Map.size());

        const VirtualGrid sector_grid(cv::Size(sectors,  sectors), cv::Rect(0, 0, cols(), rows()));
        std::array<size_t, sectors * sectors> sector_buckets{};

        const auto ideal_distribution = static_cast<size_t>(
            static_cast<double>(m_Data.size()) / static_cast<double>(sector_buckets.size())
        );

        double excess = 0.0;
        for(const auto& [key, data] : m_Data)
        {
            const size_t index = sector_grid.key_to_index(sector_grid.key_of(key));
            if(++sector_buckets[index] > ideal_distribution)
                excess += 1.0;
        }

        // The maximum excess occurs when all points are in the same sector
        return  1.0 - (excess / static_cast<double>(m_Data.size() - ideal_distribution));
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline SpatialMap<T>::iterator SpatialMap<T>::begin()
    {
        return m_Data.begin();
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline SpatialMap<T>::iterator SpatialMap<T>::end()
    {
        return m_Data.end();
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline SpatialMap<T>::const_iterator SpatialMap<T>::begin() const
    {
        return m_Data.cbegin();
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline SpatialMap<T>::const_iterator SpatialMap<T>::end() const
    {
        return m_Data.cend();
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline SpatialMap<T>::const_iterator SpatialMap<T>::cbegin() const
    {
        return m_Data.cbegin();
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline SpatialMap<T>::const_iterator SpatialMap<T>::cend() const
    {
        return m_Data.cend();
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline size_t& SpatialMap<T>::fetch_data_link(const SpatialKey& key)
    {
        return  m_Map[m_VirtualGrid.key_to_index(key)];
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline size_t SpatialMap<T>::fetch_data_link(const SpatialKey& key) const
    {
        return  m_Map[m_VirtualGrid.key_to_index(key)];
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline void SpatialMap<T>::clear_data_link(size_t& link)
    {
        link = m_EmptySymbol;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline bool SpatialMap<T>::is_data_link_empty(const size_t link) const
    {
        return link == m_EmptySymbol;
    }

//---------------------------------------------------------------------------------------------------------------------

}

