//    *************************** LiveVisionKit ****************************
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

#include "Logger.hpp"

#include "Diagnostics/Directives.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	// TODO: don't really like that this is a reference, its possible 
	// for the ostream to have a shorter lifetime than the logger. 
	// However the reference has its benefits too. 
	Logger::Logger(std::ostream& target)
		: m_Stream(target),
		  m_BaseFormat(nullptr)
	{
		LVK_ASSERT(target.good());
		m_BaseFormat.copyfmt(target);

		begin_log(m_Stream);
	}

//---------------------------------------------------------------------------------------------------------------------

	Logger::~Logger()
	{
		end_log(m_Stream);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	Logger& Logger::write(const T& object)
	{
		if (!m_HoldInputs)
		{
			if(m_NewRecord)
			{
				begin_record(m_Stream);
				m_NewRecord = false;
			}

			begin_object(m_Stream);
			m_Stream << object;
			end_object(m_Stream);
		}
		return *this;
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	Logger& Logger::operator<<(const T& object)
	{
		return write(object);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	Logger& Logger::append(const T& object)
	{
		if(!m_HoldInputs)
			m_Stream << object;
		
		return *this;
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	void Logger::operator+=(const T& object)
	{
		append(object);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	Logger& Logger::operator+(const T& object)
	{
		return append(object);
	}

//---------------------------------------------------------------------------------------------------------------------

	std::ostream& Logger::raw()
	{
		return m_Stream;
	}

//---------------------------------------------------------------------------------------------------------------------

	void Logger::next() 
	{
		if (!m_HoldRecord)
		{
			end_record(m_Stream);
			m_NewRecord = true;
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void Logger::flush()
	{
		m_Stream.flush();
	}

//---------------------------------------------------------------------------------------------------------------------

	void Logger::hold(const bool all_inputs)
	{
		m_HoldRecord = true;
		m_HoldInputs = all_inputs;
	}

//---------------------------------------------------------------------------------------------------------------------

	void Logger::resume()
	{
		m_HoldInputs = false;
		m_HoldRecord = false;
	}

//---------------------------------------------------------------------------------------------------------------------

	void Logger::reformat() 
	{
		m_Stream.copyfmt(base_format());
	}

//---------------------------------------------------------------------------------------------------------------------

	const std::ios& Logger::base_format() const
	{
		return m_BaseFormat;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool Logger::has_error() const
	{
		return m_Stream.fail();
	}

//---------------------------------------------------------------------------------------------------------------------

	void Logger::begin_log(std::ostream& stream) {}

//---------------------------------------------------------------------------------------------------------------------

	void Logger::end_log(std::ostream& stream) {}

//---------------------------------------------------------------------------------------------------------------------

	void Logger::begin_object(std::ostream& stream) {}

//---------------------------------------------------------------------------------------------------------------------

	void Logger::end_object(std::ostream& stream) {}

//---------------------------------------------------------------------------------------------------------------------

	void Logger::begin_record(std::ostream& stream) {}

//---------------------------------------------------------------------------------------------------------------------

	void Logger::end_record(std::ostream& stream) 
	{ 
		stream << "\n";
	}

//---------------------------------------------------------------------------------------------------------------------
}
