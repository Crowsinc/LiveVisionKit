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

#include "TickTimer.hpp"

namespace lvk
{
//---------------------------------------------------------------------------------------------------------------------

	TickTimer::TickTimer(const uint32_t history)
		: Stopwatch(history)
	{
		LVK_ASSERT(history > 0);
	}
	
//---------------------------------------------------------------------------------------------------------------------

	Time TickTimer::tick()
	{
		m_Counter++;
		m_DeltaTime = restart();
		return m_DeltaTime;
	}

//---------------------------------------------------------------------------------------------------------------------

	Time TickTimer::tick(const Time& timestep)
	{
		wait_until(timestep);
		return tick();
	}

//---------------------------------------------------------------------------------------------------------------------

	uint64_t TickTimer::tick_count() const
	{
		return m_Counter;
	}

//---------------------------------------------------------------------------------------------------------------------

	void TickTimer::reset_counter()
	{
		m_Counter = 0;
	}

//---------------------------------------------------------------------------------------------------------------------

	Time TickTimer::delta() const
	{
		return m_DeltaTime;
	}

//---------------------------------------------------------------------------------------------------------------------

}