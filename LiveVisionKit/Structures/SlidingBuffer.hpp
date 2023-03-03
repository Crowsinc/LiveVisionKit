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
#include <ostream>

namespace lvk
{

	template<typename T>
	class SlidingBuffer
	{
	public:

		explicit SlidingBuffer(const size_t capacity = 5);

		void push(const T& element);

		void push(T&& element);

		template<typename... Args>
		T& advance(Args&&... args);

		T& skip();
		
		void skip(const size_t amount);

		void trim(const size_t amount);
		
		void resize(const size_t capacity);
		
		void clear();

		T& at(const size_t index);

		T& operator[](const size_t index);

		T& centre(const int offset = 0);

		T& oldest(const int offset = 0);

		T& newest(const int offset = 0);

		T& previous();

		const T& at(const size_t index) const;

		const T& operator[](const size_t index) const;

		const T& centre(const int offset = 0) const;

		const T& oldest(const int offset = 0) const;

		const T& newest(const int offset = 0) const;

		const T& previous() const;

		bool is_full() const;

		bool is_empty() const;

		size_t size() const;

		size_t capacity() const;

		size_t centre_index() const;

		template<typename K>
		SlidingBuffer<T> convolve(const SlidingBuffer<K>& kernel) const;

		template<typename K>
		T convolve_at(const SlidingBuffer<K>& kernel, const size_t index) const;

        T average(const size_t start, const size_t count) const;

        T average() const;

        T variance(const size_t start, const size_t count) const;

        T variance() const;

        T min(const size_t start, const size_t count) const;

        T min() const;

        T max(const size_t start, const size_t count) const;

        T max() const;

        T sum(const size_t start, const size_t count) const;

        T sum() const;

	private:

		void advance_window();

	private:
		size_t m_Capacity, m_Size = 0;
		std::vector<T> m_InternalBuffer;
		size_t m_StartIndex = 0, m_EndIndex = 0;
	};


    template<typename T>
	std::ostream& operator<<(std::ostream& stream, const SlidingBuffer<T>& buffer);

}

#include "SlidingBuffer.tpp"
