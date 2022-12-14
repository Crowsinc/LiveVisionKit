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

#pragma once

#include <ostream>
#include <iomanip>
#include <iostream>

namespace lvk
{

	class Logger
	{
        struct NextSignal {};
	public:

        const static NextSignal Next;

		explicit Logger(std::ostream& target = std::cout);

		virtual ~Logger() = default;

		template<typename T>
		Logger& write(const T& object);

		template<typename T>
		Logger& operator<<(const T& object);

        Logger& operator<<(const NextSignal& signal);

		template<typename T>
		Logger& append(const T& object);

		template<typename T>
		void operator+=(const T& object);
		
		template<typename T>
		Logger& operator+(const T& object);

		std::ostream& raw();

		void next();

		virtual void flush();

		void hold(const bool all_inputs = false);
		
		void resume();
		
		virtual void reformat();
		
		bool has_error() const;

        bool has_started() const;

	protected:
		
		bool is_new_record() const;

		const std::ios& base_format() const;

		virtual void begin_record(std::ostream& stream);

		virtual void end_record(std::ostream& stream);

		virtual void begin_object(std::ostream& stream);

		virtual void end_object(std::ostream& stream);

	private:
		std::ostream& m_Stream;
		std::ios m_BaseFormat;

        bool m_Started = false;
		bool m_NewRecord = true;
		bool m_HoldRecord = false;
		bool m_HoldInputs = false;
	};

}

#include "Logger.tpp"
