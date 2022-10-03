//    *************************** LiveVisionKit ****************************
//    Copyright (C) 2022  Sebastian Di Marco (crowsinc.dev@gmail.com)
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.
// 	  **********************************************************************

#include "Diagnostics/Directives.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	SlidingBuffer<T>::SlidingBuffer(const uint32_t capacity)
		: m_Capacity(capacity)
	{
		LVK_ASSERT(capacity > 0);
		clear();
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	void SlidingBuffer<T>::advance_window()
	{
		if(full())
		{
			m_EndIndex = (m_EndIndex + 1) % m_Capacity;
			m_StartIndex = (m_StartIndex + 1) % m_Capacity;
		}
		else m_EndIndex = m_InternalBuffer.size();
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	void SlidingBuffer<T>::push(const T& element)
	{
		advance_window();

		if(!full())
			m_InternalBuffer.push_back(element);
		else
			m_InternalBuffer[m_EndIndex] = element;
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	void SlidingBuffer<T>::push(T&& element)
	{
		advance_window();

		if(!full())
			m_InternalBuffer.push_back(std::move(element));
		else
			m_InternalBuffer[m_EndIndex] = std::move(element);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	template<typename... Args>
	T& SlidingBuffer<T>::advance(Args&&... args)
	{
		// Advances the window, either emplacing a new element or re-using the element
		// being overwritten when full. This exists in order to enable user-level
		// optimisations by removing the need to use the copying push function.

		advance_window();

		if(!full())
			return m_InternalBuffer.emplace_back(args...);
		else
			return m_InternalBuffer[m_EndIndex];
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	void SlidingBuffer<T>::clear()
	{
		m_EndIndex = 0;
		m_StartIndex = 0;
		m_InternalBuffer.clear();
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	void SlidingBuffer<T>::resize(const uint32_t capacity)
	{
		LVK_ASSERT(capacity > 0);

		if(capacity == m_Capacity)
			return;

		if(!m_InternalBuffer.empty())
		{
			// Simplest way to do this is to allocate a new internal buffer
			// and copy over all the elements which fit in the correct order.
			// We always copy back to front, keeping the newest N elements
			// to conserve the temporal nature of the data.

			std::vector<T> new_buffer;
			new_buffer.reserve(capacity);

			const auto copy_count = std::min(capacity, elements());
			const auto copy_start = elements() - copy_count;
			for(uint32_t i = 0; i < copy_count; i++)
				if constexpr (std::is_move_constructible_v<T>)
					new_buffer.push_back(std::move(at(copy_start + i)));
				else
					new_buffer.push_back(at(copy_start + i));

			m_InternalBuffer = std::move(new_buffer);
		}

		m_Capacity = capacity;
		
		m_StartIndex = 0;
		m_EndIndex = std::max<uint64_t>(elements(), 1) - 1; // Clamped to [0, capacity)
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	template<typename K>
	T SlidingBuffer<T>::convolve_at(const SlidingBuffer<K>& kernel, const uint32_t index, T initial) const
	{
		LVK_ASSERT(!empty());
		LVK_ASSERT(!kernel.empty());
		LVK_ASSERT(elements() >= kernel.elements());

		uint32_t elems = 0, buffer_offset = 0, kernel_offset = 0;
		if (index <= kernel.centre_index())
		{
			kernel_offset = kernel.centre_index() - index;

			// Kernel is always equal to or smaller than buffer as per the pre-condition
			// so it will always bound the number of elements. 
			elems = kernel.elements() - kernel_offset;
		}
		else
		{
			buffer_offset = index - kernel.centre_index();
		
			// Kernel can be smaller than buffer
			elems = std::min(this->elements() - buffer_offset, kernel.elements());
		}

		for(uint32_t i = 0; i < elems; i++)
			initial = initial + at(i + buffer_offset) * kernel.at(i + kernel_offset);

		return initial;
	}
	
//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	template<typename K>
	SlidingBuffer<T> SlidingBuffer<T>::convolve(const SlidingBuffer<K>& kernel, T initial) const
	{
		LVK_ASSERT(!empty());
		LVK_ASSERT(!kernel.empty());
		LVK_ASSERT(elements() >= kernel.elements());

		SlidingBuffer<T> buffer(capacity());
		for (uint32_t i = 0; i < elements(); i++)
			buffer.push(convolve_at(kernel, i, initial));
		
		return buffer;
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	T& SlidingBuffer<T>::at(const uint32_t index)
	{
		LVK_ASSERT(index < elements());

		return m_InternalBuffer[(m_StartIndex + index) % m_Capacity];
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	T& SlidingBuffer<T>::operator[](const uint32_t index)
	{
		return at(index);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	const T& SlidingBuffer<T>::at(const uint32_t index) const
	{
		LVK_ASSERT(index < elements());

		return m_InternalBuffer[(m_StartIndex + index) % m_Capacity];
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	const T& SlidingBuffer<T>::operator[](const uint32_t index) const
	{
		return at(index);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	const T& SlidingBuffer<T>::centre() const
	{
		LVK_ASSERT(!empty());

		// NOTE: Gets lower centre for even sizing.
		return at(centre_index());
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	const T& SlidingBuffer<T>::oldest() const
	{
		LVK_ASSERT(!empty());

		return at(0);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	const T& SlidingBuffer<T>::newest() const
	{
		LVK_ASSERT(!empty());

		return at(elements() - 1);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	const T& SlidingBuffer<T>::previous() const
	{
		LVK_ASSERT(elements() > 1);

		return at(elements() - 2);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	T& SlidingBuffer<T>::centre()
	{
		LVK_ASSERT(!empty());

		// NOTE: Gets lower centre for even sizing.
		return at(centre_index());
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	T& SlidingBuffer<T>::oldest()
	{
		LVK_ASSERT(!empty());

		return at(0);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	T& SlidingBuffer<T>::newest()
	{
		LVK_ASSERT(!empty());

		return at(elements() - 1);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	T& SlidingBuffer<T>::previous()
	{
		LVK_ASSERT(elements() > 1);

		return at(elements() - 2);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	bool SlidingBuffer<T>::full() const
	{
		return elements() == capacity();
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	bool SlidingBuffer<T>::empty() const
	{
		return m_InternalBuffer.empty();
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	T SlidingBuffer<T>::average() const
	{
		LVK_ASSERT(!empty());

		// Kick start calculation with element 0 to avoid
		// requirement of a default initialisation of T.
		T avg = at(0);
		for(uint32_t i = 1; i < elements(); i++)
			avg = avg + at(i);

		return avg / elements();
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	T SlidingBuffer<T>::variance() const
	{
		LVK_ASSERT(!empty());

		// Kick start calculation with element 0 to avoid
		// requirement of a default initialisation of T.
		const T avg = average();
		T diff = at(0) - avg;
		T var = diff * diff;

		for(uint32_t i = 1; i < elements(); i++)
		{
			diff = at(i) - avg;
			var = var + diff * diff;
		}

		return var / elements();
	}


//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	uint32_t SlidingBuffer<T>::elements() const
	{
		return m_InternalBuffer.size();
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	uint32_t SlidingBuffer<T>::capacity() const
	{
		return m_Capacity;
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	uint32_t SlidingBuffer<T>::centre_index() const
	{
		LVK_ASSERT(!empty());

		// NOTE: Gets lower centre index for even sizing.
		// NOTE: This is an external 0-N index to be used with SlidingBuffer at() and [] operations.
		return (elements() - 1) / 2;
	}

//---------------------------------------------------------------------------------------------------------------------

}
