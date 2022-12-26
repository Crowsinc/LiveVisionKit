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

#include "Stopwatch.hpp"

#include "Diagnostics/Directives.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	Stopwatch::Stopwatch(const size_t history)
		: m_Running(false),
          m_History(history),
          m_ElapsedTime(0),
          m_StartTime(0),
          m_Memory(0)
	{
		LVK_ASSERT(history > 0);
	}
	
//---------------------------------------------------------------------------------------------------------------------

	void Stopwatch::start()
	{
		m_Running = true;
		m_StartTime = Time::Now();
	}

//---------------------------------------------------------------------------------------------------------------------

	Time Stopwatch::stop()
	{
        // If running or paused, we want to stop and reset the stopwatch
        if(is_running() || is_paused())
        {
            m_ElapsedTime = pause();
            m_History.push(m_ElapsedTime);

            m_Memory = Time(0);

            return m_ElapsedTime;
        }

        return Time(0);
	}

//---------------------------------------------------------------------------------------------------------------------

    Time Stopwatch::pause()
    {
        // If paused, returns the last pause time.
        // If stopped, returns zero as memory should be reset.
        if(!is_running())
            return m_Memory;

        m_Memory += (Time::Now() - m_StartTime);
        m_ElapsedTime = m_Memory;
        m_Running = false;

        return m_Memory;
    }

//---------------------------------------------------------------------------------------------------------------------

	Time Stopwatch::restart()
	{
        auto elapsed = stop();
        start();

		return elapsed;
	}

//---------------------------------------------------------------------------------------------------------------------

    Time Stopwatch::wait_until(const Time& required_time)
    {
        if(!is_running()) start();

        // Block until elapsed time is met.
        // We yield here because it is more power efficient than busy waiting.
        // We don't sleep because it is far too imprecise to get any consistency.
        Time elapsed_time = elapsed();
        while(elapsed_time < required_time)
        {
            std::this_thread::yield();
            elapsed_time = elapsed();
        }

        return elapsed_time;
    }

//---------------------------------------------------------------------------------------------------------------------

	bool Stopwatch::is_running() const
	{
		return m_Running;
	}

//---------------------------------------------------------------------------------------------------------------------

    bool Stopwatch::is_paused() const
    {
        return !m_Running && m_Memory.nanoseconds() > 0;
    }

//---------------------------------------------------------------------------------------------------------------------

	Time Stopwatch::elapsed() const
	{
		return is_running() ? (m_Memory + (Time::Now() - m_StartTime)) : m_ElapsedTime;
	}

//---------------------------------------------------------------------------------------------------------------------

	Time Stopwatch::average() const
	{
		return m_History.is_empty() ? Time(0) : m_History.average();
	}

//---------------------------------------------------------------------------------------------------------------------

	Time Stopwatch::deviation() const
	{
		if(m_History.size() < 2)
			return Time(0);

		const Time average_time = m_History.average();

		Time total_deviation(0);
		for(auto i = 0; i < m_History.size(); i++)
		{
			const auto& current_time = m_History[i];
			if (average_time > current_time)
				total_deviation += average_time - current_time;
			else 
				total_deviation += current_time - average_time;
		}
		
		return total_deviation / static_cast<double>(m_History.size());
	}

//---------------------------------------------------------------------------------------------------------------------

	const SlidingBuffer<Time>& Stopwatch::history() const
	{
		return m_History;
	}

//---------------------------------------------------------------------------------------------------------------------

    void Stopwatch::reset_history()
    {
        m_History.clear();
    }

//---------------------------------------------------------------------------------------------------------------------

}