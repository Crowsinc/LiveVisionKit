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

#include "Ticker.hpp"

namespace lvk
{
//---------------------------------------------------------------------------------------------------------------------

	Ticker::Ticker()
		: m_Counter(0)
	{}
	
//---------------------------------------------------------------------------------------------------------------------

	void Ticker::tick()
	{
		m_Counter++;
		m_DeltaTime = m_Stopwatch.restart();
	}

//---------------------------------------------------------------------------------------------------------------------

	Time Ticker::delta_time() const
	{
		return m_DeltaTime;
	}

//---------------------------------------------------------------------------------------------------------------------

	Time Ticker::elapsed_time() const
	{
		return m_Stopwatch.elapsed();
	}

//---------------------------------------------------------------------------------------------------------------------

	uint64_t Ticker::tick_count() const
	{
		return m_Counter;
	}

//---------------------------------------------------------------------------------------------------------------------

	void Ticker::reset_counter()
	{
		m_Counter = 0;
	}

//---------------------------------------------------------------------------------------------------------------------
}