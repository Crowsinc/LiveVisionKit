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
#include "Diagnostics/Directives.hpp"

#include <ctime>
#include <iomanip>
#include <opencv2/opencv.hpp>

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
		return Time(std::chrono::high_resolution_clock::now());
	}

//---------------------------------------------------------------------------------------------------------------------

	std::string Time::Timestamp(const char* format)
	{
		std::stringstream stream;

		//TODO: Replace with C++20 time calender and timezone functionality
		std::tm tm = {};
		std::time_t time = std::time(nullptr);
#ifdef WIN32
		localtime_s(&tm, &time);
#else
        localtime_r(&time, &tm);
#endif
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
		return Time(static_cast<uint64_t>(amount * ns_s_ratio));
	}
	
//---------------------------------------------------------------------------------------------------------------------

	Time Time::Milliseconds(const double amount)
	{
		return Time(static_cast<uint64_t>(amount * ns_ms_ratio));
	}
	
//---------------------------------------------------------------------------------------------------------------------

	Time Time::Microseconds(const double amount)
	{
		return Time(static_cast<uint64_t>(amount * ns_us_ratio));
	}
	
//---------------------------------------------------------------------------------------------------------------------

	Time Time::Nanoseconds(const uint64_t amount)
	{
		return Time(amount);
	}

//---------------------------------------------------------------------------------------------------------------------

    Time Time::Timestep(const double frequency)
    {
        LVK_ASSERT(frequency > 0.0);

        return Time::Seconds(1.0 / frequency);
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
		return static_cast<double>(m_Time.count());
	}

//---------------------------------------------------------------------------------------------------------------------

    double Time::frequency() const
    {
        return 1.0 / seconds();
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
		return Time(m_Time + other.m_Time);
	}
	
//---------------------------------------------------------------------------------------------------------------------

	Time Time::operator-(const Time& other) const
	{
		return Time(m_Time - other.m_Time);
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

	Time Time::operator*(const double multiplier) const
	{
		return Time(static_cast<uint64_t>((m_Time * multiplier).count()));
	}

//---------------------------------------------------------------------------------------------------------------------

	Time Time::operator/(const double divisor) const
	{
        return Time(static_cast<uint64_t>((m_Time / divisor).count()));
	}

//---------------------------------------------------------------------------------------------------------------------

    std::string Time::hms() const
    {
        const auto s = static_cast<uint64_t>(seconds());
        const auto m = s / 60;
        const auto h = m / 60;

        // TODO: switch to std::format when widely available for GCC
        return cv::format("%02lu:%02lu:%02lu", h, m % 60, s % 60);
    }

//---------------------------------------------------------------------------------------------------------------------

    bool Time::is_zero() const
    {
        return m_Time.count() == 0;
    }

//---------------------------------------------------------------------------------------------------------------------

}