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

#include "SlidingBuffer.hpp"

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
		if (m_InternalBuffer.size() == capacity())
		{
			// If the internal buffer is full, then we are running as a circular queue.
			// However, It is still possible that the queue is not full, so ensure that
			// we increase the size if we are not overstepping on the start index. 
			m_EndIndex = (m_EndIndex + 1) % m_Capacity;
			if (m_StartIndex == m_EndIndex)
				m_StartIndex = (m_StartIndex + 1) % m_Capacity;
			else m_Size++;
		}
		else
		{
			// If the internal buffer isn't full, then we must be zero-aligned
			m_EndIndex = m_InternalBuffer.size();
			m_Size++;
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	void SlidingBuffer<T>::push(const T& element)
	{
		advance_window();

		if(m_InternalBuffer.size() != m_Capacity)
			m_InternalBuffer.push_back(element);
		else
			m_InternalBuffer[m_EndIndex] = element;
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	void SlidingBuffer<T>::push(T&& element)
	{
		advance_window();

		if(m_InternalBuffer.size() != m_Capacity)
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

		if(m_InternalBuffer.size() != m_Capacity)
			return m_InternalBuffer.emplace_back(args...);
		else
			return m_InternalBuffer[m_EndIndex];
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	T& SlidingBuffer<T>::skip()
	{
		LVK_ASSERT(!empty());

		// Advances the start pointer to pop one element from the front of the buffer.
		// This is the counter-part to advance() and does not de-allocate memory.

		T& ref = oldest();
		skip(1);
		return ref;
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	void SlidingBuffer<T>::skip(const uint32_t amount)
	{
		if (amount == 0)
			return;

		if (amount >= m_Size)
		{
			// If the skip is clearing the buffer, or the buffer is already empty
			// then reset the buffer to be empty, but do not de-allocate memory. 
			m_StartIndex = 0;
			m_EndIndex = 0;
			m_Size = 0;
		}
		else
		{
			m_StartIndex = (m_StartIndex + amount) % m_Capacity;
			m_Size = m_Size - amount;
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	void SlidingBuffer<T>::trim(const uint32_t amount)
	{
		// NOTE: trim(0) is equivalent to unravelling the circular queue into an array

		if (amount < m_Size)
		{
			const uint32_t new_size = m_Size - amount;

			// Simplest way to do this is to allocate a new internal 
			// buffer and copy over all the required elements.

			std::vector<T> new_buffer;
			new_buffer.reserve(new_size);

			for (uint32_t i = 0; i < new_size; i++)
				if constexpr (std::is_move_constructible_v<T>)
					new_buffer.push_back(std::move(at(amount + i)));
				else
					new_buffer.push_back(at(amount + i));

			m_InternalBuffer = std::move(new_buffer);
			
			m_StartIndex = 0;
			m_EndIndex = new_size - 1;
			m_Size = new_size;
		}
		else clear();
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	void SlidingBuffer<T>::resize(const uint32_t new_capacity)
	{
		LVK_ASSERT(m_Capacity > 0);

		if(new_capacity == m_Capacity)
			return;

		// If the new capacity is less than the number of elements we have, 
		// then we need to ensure we keep the newest N elements. Regardless,
		// we need to unravel the circular queue to start at index 0 so that
		// future pushes to the internal buffer are positioned in the correct
		// location. Trim will perform this for us, even if we trim nothing. 
		trim(new_capacity < elements() ? elements() - new_capacity : 0);
		m_Capacity = new_capacity;
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	void SlidingBuffer<T>::clear()
	{
		m_Size = 0;
		m_EndIndex = 0;
		m_StartIndex = 0;
		m_InternalBuffer.clear();
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
	const T& SlidingBuffer<T>::centre(const int offset) const
	{
		LVK_ASSERT(!empty());

		// NOTE: Gets lower centre for even sizing.
		return at(centre_index() + offset);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	const T& SlidingBuffer<T>::oldest(const int offset) const
	{
		LVK_ASSERT(!empty());
		LVK_ASSERT(offset >= 0);

		return at(offset);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	const T& SlidingBuffer<T>::newest(const int offset) const
	{
		LVK_ASSERT(!empty());
		LVK_ASSERT(offset <= 0);

		return at((elements() - 1) + offset);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	const T& SlidingBuffer<T>::previous() const
	{
		LVK_ASSERT(elements() > 1);

		return newest(-1);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	T& SlidingBuffer<T>::centre(const int offset)
	{
		LVK_ASSERT(!empty());

		// NOTE: Gets lower centre for even sizing.
		return at(centre_index() + offset);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	T& SlidingBuffer<T>::oldest(const int offset)
	{
		LVK_ASSERT(!empty());
		LVK_ASSERT(offset >= 0);

		return at(offset);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	T& SlidingBuffer<T>::newest(const int offset)
	{
		LVK_ASSERT(!empty());
		LVK_ASSERT(offset <= 0);

		return at((elements() - 1) + offset);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	T& SlidingBuffer<T>::previous()
	{
		LVK_ASSERT(elements() > 1);

		return newest(-1);
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
		return elements() == 0;
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
		return m_Size;
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
