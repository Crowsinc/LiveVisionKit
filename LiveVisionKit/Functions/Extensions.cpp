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

#include "Extensions.hpp"

namespace lvk
{
//---------------------------------------------------------------------------------------------------------------------

    cv::Size2f operator*(const cv::Size2f& v1, const cv::Size2f& v2)
    {
        return cv::Size2f(v1.width * v2.width, v1.height * v2.height);
    }

//---------------------------------------------------------------------------------------------------------------------

    cv::Size2f operator/(const cv::Size2f& v1, const cv::Size2f& v2)
    {
        return cv::Size2f(v1.width / v2.width, v1.height / v2.height);
    }

//---------------------------------------------------------------------------------------------------------------------

    cv::Size operator*(const cv::Size& v1, const cv::Size& v2)
    {
        return cv::Size(v1.width * v2.width, v1.height * v2.height);
    }

//---------------------------------------------------------------------------------------------------------------------

    cv::Size operator/(const cv::Size& v1, const cv::Size& v2)
    {
        return cv::Size(v1.width / v2.width, v1.height / v2.height);
    }

//---------------------------------------------------------------------------------------------------------------------
}
