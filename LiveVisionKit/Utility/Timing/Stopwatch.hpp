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

#include "Time.hpp"
#include "Structures/SlidingBuffer.hpp"

namespace lvk
{

	class Stopwatch
	{
	public:

		explicit Stopwatch(const size_t history = 1);

        void start();

		Time stop();

        Time pause();

		Time restart();

        Time wait_until(const Time& required_time);

		bool is_running() const;

        bool is_paused() const;

		Time elapsed() const;
		
		Time average() const;
		
		Time deviation() const;

		const SlidingBuffer<Time>& history() const;

        void reset_history();

	private:
        bool m_Running = false;
		SlidingBuffer<Time> m_History;
		Time m_ElapsedTime, m_StartTime, m_Memory;
	};

}

