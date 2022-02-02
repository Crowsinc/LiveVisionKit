
#include <stdint.h>

#include "../Diagnostics/Assert.hpp"

namespace lvk
{
	//-------------------------------------------------------------------------------------

	template<typename T>
	SlidingBuffer<T>::SlidingBuffer(const uint32_t window_size)
		: m_StartIndex(0),
		  m_EndIndex(0),
		  m_WindowSize(window_size)
	{
		LVK_ASSERT(window_size > 0);

		resize(window_size);
		clear();
	}


	//-------------------------------------------------------------------------------------

	template<typename T>
	void SlidingBuffer<T>::advance_window()
	{
		if(full())
		{
			m_EndIndex = (m_EndIndex + 1) % m_WindowSize;
			m_StartIndex = (m_StartIndex + 1) % m_WindowSize;
		}
		else m_EndIndex = m_InternalBuffer.size();
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	void SlidingBuffer<T>::push(const T& element)
	{
		advance_window();

		if(!full())
			m_InternalBuffer.push_back(element);
		else
			m_InternalBuffer[m_EndIndex] = element;
	}

	//-------------------------------------------------------------------------------------

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

	//-------------------------------------------------------------------------------------

	template<typename T>
	void SlidingBuffer<T>::clear()
	{
		m_EndIndex = 0;
		m_StartIndex = 0;
		m_InternalBuffer.clear();
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	void SlidingBuffer<T>::resize(const uint32_t window_size)
	{
		LVK_ASSERT(window_size > 0);

		if(window_size == m_WindowSize)
			return;

		// If the buffer isn't zero-alligned then we need to allign it
		// to keep element ordering correct with the new window size.
		if(!m_InternalBuffer.empty() && m_StartIndex != 0)
		{
			// Simplest way to do this is to allocate a new internal buffer
			// and copy over all the elements which fit in the correct order.
			// We always copy back to front, keeping the newest N elements
			// to conserve 'temporal consistency'.

			std::vector<T> new_buffer;
			new_buffer.reserve(window_size);

			const auto copy_count = std::min(window_size, elements());
			const auto copy_start = elements() - copy_count;
			for(uint32_t i = 0; i < copy_count; i++)
				new_buffer.push_back(this->at(copy_start + i));

			m_InternalBuffer = std::move(new_buffer);
		}

		m_StartIndex = 0;
		m_EndIndex = elements() - 1;
		m_WindowSize = window_size;
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	template<typename K>
	T SlidingBuffer<T>::filter(const SlidingBuffer<K>& kernel, T initial) const
	{
		LVK_ASSERT(!this->empty() && !kernel.empty());
		LVK_ASSERT(this->elements() == kernel.elements());

		for(uint32_t i = 0; i < elements(); i++)
			initial = initial + this->at(i) * kernel.at(i);

		return initial;
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	T& SlidingBuffer<T>::at(const uint32_t index)
	{
		LVK_ASSERT(index < this->elements());

		return m_InternalBuffer[(m_StartIndex + index) % m_WindowSize];
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	T& SlidingBuffer<T>::operator[](const uint32_t index)
	{
		return at(index);
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	const T& SlidingBuffer<T>::at(const uint32_t index) const
	{
		LVK_ASSERT(index < this->elements());

		return m_InternalBuffer[(m_StartIndex + index) % m_WindowSize];
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	const T& SlidingBuffer<T>::operator[](const uint32_t index) const
	{
		return at(index);
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	const T& SlidingBuffer<T>::centre() const
	{
		LVK_ASSERT(!this->empty());

		// NOTE: Gets lower centre for even sizing.
		// Use at() method to automatically handle wrapping.
		return at(centre_index());
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	const T& SlidingBuffer<T>::oldest() const
	{
		LVK_ASSERT(!this->empty());
		return m_InternalBuffer[m_StartIndex];
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	const T& SlidingBuffer<T>::newest() const
	{
		LVK_ASSERT(!this->empty());
		return m_InternalBuffer[m_EndIndex];
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	const T& SlidingBuffer<T>::previous() const
	{
		LVK_ASSERT(this->elements() > 1);
		return this->at(this->elements() - 2);
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	T& SlidingBuffer<T>::centre()
	{
		LVK_ASSERT(!this->empty());
		// NOTE: Gets lower centre for even sizing.
		// Use at() method to automatically handle wrapping.
		return at(centre_index());
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	T& SlidingBuffer<T>::oldest()
	{
		LVK_ASSERT(!this->empty());

		return m_InternalBuffer[m_StartIndex];
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	T& SlidingBuffer<T>::newest()
	{
		LVK_ASSERT(!this->empty());
		return m_InternalBuffer[m_EndIndex];
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	T& SlidingBuffer<T>::previous()
	{
		LVK_ASSERT(this->elements() > 1);
		return this->at(this->elements() - 2);
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	bool SlidingBuffer<T>::full() const
	{
		return elements() == window_size();
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	bool SlidingBuffer<T>::empty() const
	{
		return m_InternalBuffer.empty();
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	uint32_t SlidingBuffer<T>::elements() const
	{
		return m_InternalBuffer.size();
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	uint32_t SlidingBuffer<T>::window_size() const
	{
		return m_WindowSize;
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	uint32_t SlidingBuffer<T>::centre_index() const
	{
		LVK_ASSERT(!this->empty());

		// NOTE: Gets lower centre index for even sizing.
		// NOTE: This is an external 0-N index not an internal
		// one which wraps from start index to end index.
		return (elements() - 1) / 2;
	}

	//-------------------------------------------------------------------------------------

}
