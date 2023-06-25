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

#include "Iterators.hpp"

namespace lvk
{

	template<typename T>
	class StreamBuffer
	{
	public:
        using iterator = lvk::circular_iterator<T>;
        using const_iterator = lvk::const_circular_iterator<T>;
    public:

		explicit StreamBuffer(const size_t capacity);


		void push(T&& element);

		void push(const T& element);

		template<typename... Args>
		T& advance(Args&&... args);


        T&& pop_oldest();


        template<typename... Args>
        void pad_front(Args&&... args);

        template<typename... Args>
        void pad_back(Args&&... args);


		void trim(const size_t amount);

        void skip(const size_t amount = 1);


		void resize(const size_t capacity);

		void clear();


		T& at(const size_t index);

		const T& at(const size_t index) const;

        T& operator[](const size_t index);

		const T& operator[](const size_t index) const;


        T& oldest(const int offset = 0);

		const T& oldest(const int offset = 0) const;

        T& centre(const int offset = 0);

		const T& centre(const int offset = 0) const;

        T& newest(const int offset = 0);

		const T& newest(const int offset = 0) const;


		bool is_full() const;

		bool is_empty() const;

		size_t size() const;

		size_t capacity() const;

		size_t centre_index() const;


		template<typename K>
		StreamBuffer<T> convolve(const StreamBuffer<K>& kernel) const;

		template<typename K>
		T convolve_at(const StreamBuffer<K>& kernel, const size_t index) const;


        iterator begin();

        const_iterator begin() const;

        const_iterator cbegin() const;


        iterator end();

        const_iterator end() const;

        const_iterator cend() const;


	private:
		void advance_window();

		size_t m_Capacity, m_Size = 0;
		std::vector<T> m_InternalBuffer;
		size_t m_StartIndex = 0, m_EndIndex = 0;
	};


    template<typename T>
	std::ostream& operator<<(std::ostream& stream, const StreamBuffer<T>& buffer);

}

#include "StreamBuffer.tpp"
