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

		Stopwatch(const size_t history = 1);

		void start();

		void stop();

		Time restart();
		
		bool is_running() const;

		Time elapsed() const;
		
		Time average() const;
		
		Time deviation() const;

		const SlidingBuffer<Time>& history() const;

	private:

		bool m_Running = false;
		Time m_StartTime, m_Elapsed;
		SlidingBuffer<Time> m_History;
	};

}

