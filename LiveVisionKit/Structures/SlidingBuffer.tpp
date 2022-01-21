
#include <stdint.h>

namespace lvk
{
	//-------------------------------------------------------------------------------------

	template<typename T>
	SlidingBuffer<T>::SlidingBuffer(const uint32_t window_size)
		: m_StartIndex(0),
		  m_EndIndex(0),
		  m_WindowSize(window_size)
	{
		resize(window_size);
		clear();
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	void SlidingBuffer<T>::push(const T& element)
	{
		if(!full())
		{
			m_InternalBuffer.push_back(element);
			m_EndIndex = m_InternalBuffer.size() - 1;
		}
		else
		{
			m_EndIndex = (m_EndIndex + 1) % m_WindowSize;
			m_StartIndex = (m_StartIndex + 1) % m_WindowSize;

			m_InternalBuffer[m_EndIndex] = element;
		}
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
		if(window_size == m_WindowSize)
			return;

		// If the buffer isn't zero-alligned then we need to allign it
		// to keep element ordering correct with the new window size.
		if(!m_InternalBuffer.empty() && m_StartIndex != 0)
		{
			// Simplest way to do this is to allocate a new internal buffer
			// And copy over all the elements which fit in the correct order.
			// We always copy back to front, in order to leave the newest N
			// elements which fit, in order to keep 'temporal' consistency.

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
	T SlidingBuffer<T>::convolve(const SlidingBuffer<K>& kernel) const
	{
		T result{};

		// NOTE: The kernel and sliding window is always centre alligned.
		// If either sizing is even, it is alligned with the lower indexed centre value.
		if(elements() > kernel.elements())
		{
			// Window is bigger than kernel, so allign kernel to window.
			const auto centre_offset = centre_index() - kernel.centre_index();
			for(uint32_t i = 0; i < kernel.elements(); i++)
				result += this->at(i + centre_offset) * kernel.at(i);
		}
		else
		{
			// Kernel is bigger (or equal) to window, so allign window to kernel
			const auto centre_offset = kernel.centre_index() - centre_index();
			for(uint32_t i = 0; i < elements(); i++)
				result += this->at(i) * kernel.at(i + centre_offset);
		}
		return result;
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	T& SlidingBuffer<T>::at(const uint32_t index)
	{
		//TODO: assert if index > WindowSize
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
		//TODO: assert if index > WindowSize
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
		// NOTE: gets lower centre for even sizing.
		return m_InternalBuffer[centre_index()];
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	const T& SlidingBuffer<T>::oldest() const
	{
		return m_InternalBuffer[m_StartIndex];
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	const T& SlidingBuffer<T>::newest() const
	{
		return m_InternalBuffer[m_EndIndex];
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	T& SlidingBuffer<T>::centre()
	{
		// NOTE: gets lower centre for even sizing.
		return m_InternalBuffer[centre_index()];
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	T& SlidingBuffer<T>::oldest()
	{
		return m_InternalBuffer[m_StartIndex];
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	T& SlidingBuffer<T>::newest()
	{
		return m_InternalBuffer[m_EndIndex];
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	bool SlidingBuffer<T>::full() const
	{
		return elements() == m_WindowSize;
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
		// NOTE: gets lower centre index for even sizing.
		return (elements() - 1) / 2;
	}

	//-------------------------------------------------------------------------------------

}
