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

#include <string>
#include <chrono>

namespace lvk
{
	class Time
	{
	public:
		using TimePoint = std::chrono::high_resolution_clock::time_point;

		static Time Now();

		static std::string Timestamp(const char* format = "%F %T");

		static Time Hours(const double amount);
		
		static Time Minutes(const double amount);
		
		static Time Seconds(const double amount);
		
		static Time Milliseconds(const double amount);
		
		static Time Microseconds(const double amount);
		
		static Time Nanoseconds(const uint64_t amount);

		Time();

		Time(const TimePoint& time);

		Time(const uint64_t nanoseconds);

		Time(const std::chrono::nanoseconds nanoseconds);

		Time(const Time& other);

		double hours() const;

		double minutes() const;

		double seconds() const;
		
		double milliseconds() const;
		
		double microseconds() const;
		
		double nanoseconds() const;
	
		void operator=(const Time& other);

		void operator+=(const Time& other);

		void operator-=(const Time& other);
		
		Time operator+(const Time& other) const;

		Time operator-(const Time& other) const;

		bool operator==(const Time& other) const;

		bool operator!=(const Time& other) const;

		bool operator>(const Time& other) const;
		
		bool operator>=(const Time& other) const;
	
		bool operator<(const Time& other) const;
		
		bool operator<=(const Time& other) const;

		Time operator*(const double multiplier) const;
		
		Time operator/(const double divisor) const;

	private:
		std::chrono::nanoseconds m_Time;
	};

}

