//     *************************** LiveVisionKit ****************************
//     Copyright (C) 2022  Sebastian Di Marco (crowsinc.dev@gmail.com)
//
//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the GNU General Public License
//     along with this program.  If not, see <https://www.gnu.org/licenses/>.
//     **********************************************************************

#include "ConsoleLogger.hpp"

#ifdef WIN32
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#endif

namespace clt
{
//---------------------------------------------------------------------------------------------------------------------

    ConsoleLogger::ConsoleLogger()
        : lvk::Logger(std::cout)
    {
#ifdef WIN32
        // If we are in Windows, we need to put the console in virtual terminal mode
        // so that it is capable of understanding ANSI codes and is cross-platform.
        auto handle = GetStdHandle(STD_OUTPUT_HANDLE);
        SetConsoleMode(
            handle,
            ENABLE_PROCESSED_OUTPUT | ENABLE_VIRTUAL_TERMINAL_PROCESSING | DISABLE_NEWLINE_AUTO_RETURN
        );
#endif

        std::cout << "\033[?25l" // Disable cursor
                  << "\033[=7l"; // Disable line wrapping (non-windows)
    }

//---------------------------------------------------------------------------------------------------------------------

    ConsoleLogger::~ConsoleLogger() noexcept
    {
        // TODO: restore windows console mode

        std::cout << "\033[?25h" // Enable cursor
                  << "\033[=7h"; // Enable line wrapping (non-windows)
    }

//---------------------------------------------------------------------------------------------------------------------

    void ConsoleLogger::end_record(std::ostream& stream)
    {
        m_LineCount++;
        stream << "\n";
    }

//---------------------------------------------------------------------------------------------------------------------

    void ConsoleLogger::clear()
    {
        if(m_LineCount > 0)
            std::cout << "\033[" << (m_LineCount) << 'A'; // Move cursor up to beginning of log


        std::cout << "\033[0G" // Move cursor to start of line
                  << "\033[0J"; // Delete everything after and including the cursor

        m_LineCount = 0;
    }

//---------------------------------------------------------------------------------------------------------------------
}