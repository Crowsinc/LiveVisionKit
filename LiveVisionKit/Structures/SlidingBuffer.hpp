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

#pragma once

#include <vector>

namespace lvk
{

	template<typename T>
	class SlidingBuffer
	{
	public:

		SlidingBuffer(const uint32_t capacity = 5);

		void push(const T& element);

		void push(T&& element);

		template<typename... Args>
		T& advance(Args&&... args);

		T& skip();
		
		void skip(const uint32_t amount);

		void trim(const uint32_t amount);
		
		void resize(const uint32_t capacity);
		
		void clear();

		template<typename K>
		T convolve_at(const SlidingBuffer<K>& kernel, const uint32_t index, T initial = T()) const;
		
		template<typename K>
		SlidingBuffer<T> convolve(const SlidingBuffer<K>& kernel, T initial = T()) const;

		T& at(const uint32_t index);

		T& operator[](const uint32_t index);

		T& centre(const int offset = 0);

		T& oldest(const int offset = 0);

		T& newest(const int offset = 0);

		T& previous();

		const T& at(const uint32_t index) const;

		const T& operator[](const uint32_t index) const;

		const T& centre(const int offset = 0) const;

		const T& oldest(const int offset = 0) const;

		const T& newest(const int offset = 0) const;

		const T& previous() const;

		bool full() const;

		bool empty() const;

		T average() const;

		T variance() const;

		uint32_t elements() const;

		uint32_t capacity() const;

		uint32_t centre_index() const;

	private:

		void advance_window();

	private:

		uint32_t m_Capacity, m_Size = 0;
		std::vector<T> m_InternalBuffer;
		uint32_t m_StartIndex = 0, m_EndIndex = 0;

	};

}

#include "SlidingBuffer.tpp"
