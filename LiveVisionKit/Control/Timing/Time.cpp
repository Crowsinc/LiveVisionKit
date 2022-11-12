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

#include "Time.hpp"

#include <ctime>
#include <time.h>
#include <sstream>
#include <iomanip>

namespace lvk
{
	
//---------------------------------------------------------------------------------------------------------------------

	// Common ratios for converting between time formats.
	// Where x_y_ratio is the number of x in y. For example
	// s_m_ratio is the number of seconds in one minute. 
	constexpr double s_h_ratio   = 3600.0;
	constexpr double s_m_ratio   = 60.0;
	constexpr double ns_s_ratio  = 1000000000.0;
	constexpr double ns_ms_ratio = 1000000.0;
	constexpr double ns_us_ratio = 1000.0;

//---------------------------------------------------------------------------------------------------------------------

	Time Time::Now()
	{
		return {std::chrono::high_resolution_clock::now()};
	}

//---------------------------------------------------------------------------------------------------------------------

	std::string Time::Timestamp(const char* format)
	{
		std::stringstream stream;

		//TODO: Replace with C++20 time calender and timezone functionality
		std::tm tm;
		std::time_t time = std::time(0);
		localtime_s(&tm, &time);

		stream << std::put_time(&tm, format);
		return stream.str();
	}
	
//---------------------------------------------------------------------------------------------------------------------

	Time Time::Hours(const double amount)
	{
		return Seconds(amount * s_h_ratio);
	}
	
//---------------------------------------------------------------------------------------------------------------------

	Time Time::Minutes(const double amount)
	{
		return Seconds(amount * s_m_ratio);
	}
	
//---------------------------------------------------------------------------------------------------------------------

	Time Time::Seconds(const double amount)
	{
		return {static_cast<uint64_t>(amount * ns_s_ratio)};
	}
	
//---------------------------------------------------------------------------------------------------------------------

	Time Time::Milliseconds(const double amount)
	{
		return {static_cast<uint64_t>(amount * ns_ms_ratio)};
	}
	
//---------------------------------------------------------------------------------------------------------------------

	Time Time::Microseconds(const double amount)
	{
		return {static_cast<uint64_t>(amount * ns_us_ratio)};
	}
	
//---------------------------------------------------------------------------------------------------------------------

	Time Time::Nanoseconds(const uint64_t amount)
	{
		return {amount};
	}
	
//---------------------------------------------------------------------------------------------------------------------

	Time::Time()
		: m_Time()
	{}
	
//---------------------------------------------------------------------------------------------------------------------

	Time::Time(const TimePoint& time)
		: m_Time(time.time_since_epoch())
	{}
	
//---------------------------------------------------------------------------------------------------------------------

	Time::Time(const uint64_t nanoseconds)
		: m_Time(nanoseconds)
	{}

//---------------------------------------------------------------------------------------------------------------------

	Time::Time(const std::chrono::nanoseconds nanoseconds)
	    : m_Time(nanoseconds)
	{}

//---------------------------------------------------------------------------------------------------------------------

	Time::Time(const Time& other)
		: m_Time(other.m_Time)
	{}
	
//---------------------------------------------------------------------------------------------------------------------

	double Time::hours() const
	{
		return seconds() * (1.0 / s_h_ratio);
	}
	
//---------------------------------------------------------------------------------------------------------------------

	double Time::minutes() const
	{
		return seconds() * (1.0 / s_m_ratio);
	}
	
//---------------------------------------------------------------------------------------------------------------------

	double Time::seconds() const
	{
		return nanoseconds() * (1.0 / ns_s_ratio);
	}
	
//---------------------------------------------------------------------------------------------------------------------

	double Time::milliseconds() const
	{
		return nanoseconds() * (1.0 / ns_ms_ratio);
	}
	
//---------------------------------------------------------------------------------------------------------------------

	double Time::microseconds() const
	{
		return nanoseconds() * (1.0 / ns_us_ratio);
	}
	
//---------------------------------------------------------------------------------------------------------------------

	double Time::nanoseconds() const
	{
		return m_Time.count();
	}
	
//---------------------------------------------------------------------------------------------------------------------

	void Time::operator=(const Time& other)
	{
		m_Time = other.m_Time;
	}
	
//---------------------------------------------------------------------------------------------------------------------

	void Time::operator+=(const Time& other)
	{
		m_Time += other.m_Time;
	}
	
//---------------------------------------------------------------------------------------------------------------------

	void Time::operator-=(const Time& other)
	{
		m_Time -= other.m_Time;
	}
	
//---------------------------------------------------------------------------------------------------------------------

	Time Time::operator+(const Time& other) const
	{
		return {m_Time + other.m_Time};
	}
	
//---------------------------------------------------------------------------------------------------------------------

	Time Time::operator-(const Time& other) const
	{
		return {m_Time - other.m_Time};
	}
	
//---------------------------------------------------------------------------------------------------------------------

	bool Time::operator==(const Time& other) const
	{
		return m_Time == other.m_Time;
	}
	
//---------------------------------------------------------------------------------------------------------------------

	bool Time::operator!=(const Time& other) const
	{
		return !(*this == other);
	}
	
//---------------------------------------------------------------------------------------------------------------------

	bool Time::operator>(const Time& other) const
	{
		return m_Time > other.m_Time;
	}
	
//---------------------------------------------------------------------------------------------------------------------

	bool Time::operator>=(const Time& other) const
	{
		return m_Time >= other.m_Time;
	}
	
//---------------------------------------------------------------------------------------------------------------------

	bool Time::operator<(const Time& other) const
	{
		return m_Time < other.m_Time;
	}
	
//---------------------------------------------------------------------------------------------------------------------

	bool Time::operator<=(const Time& other) const
	{
		return m_Time <= other.m_Time;
	}
	
//---------------------------------------------------------------------------------------------------------------------

}