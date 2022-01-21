#pragma once

#include <vector>
#include <stdint.h>

namespace lvk
{

	template<typename T>
	class SlidingBuffer
	{
	public:

		SlidingBuffer(const uint32_t window_size);

		void push(const T& element);

		void clear();

		void resize(const uint32_t window_size);

		template<typename K>
		T convolve(const SlidingBuffer<K>& kernel) const;

		T& at(const uint32_t index);

		T& operator[](const uint32_t index);

		T& centre();

		T& oldest();

		T& newest();

		const T& at(const uint32_t index) const;

		const T& operator[](const uint32_t index) const;

		const T& centre() const;

		const T& oldest() const;

		const T& newest() const;

		bool full() const;

		uint32_t elements() const;

		uint32_t window_size() const;

		uint32_t centre_index() const;

	private:

		std::vector<T> m_InternalBuffer;
		uint32_t m_StartIndex, m_EndIndex, m_WindowSize;
	};

}

#include "SlidingBuffer.tpp"
