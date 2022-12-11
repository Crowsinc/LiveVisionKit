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
          m_Memory(0),
          m_StartTime(0),
          m_History(history)
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
            auto elapsed_time = pause();
            m_History.push(elapsed_time);

            m_Memory = Time(0);

            return elapsed_time;
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
		return is_running() ? (m_Memory + (Time::Now() - m_StartTime)) : m_Memory;
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