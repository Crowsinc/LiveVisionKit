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

#pragma once

#include <opencv2/opencv.hpp>

namespace lvk
{
    struct Frame
    {
        cv::UMat data;
        uint64_t timestamp;

        static Frame Wrap(cv::UMat& frame, const uint64_t timestamp = 0);

        explicit Frame(const uint64_t timestamp = 0);

        // NOTE: copies the frame, use Wrap() to reference
        explicit Frame(const cv::UMat& frame, const uint64_t timestamp = 0);

        Frame(
            const cv::Size& size,
            const int type,
            const uint64_t timestamp = 0
        );

        Frame(
            const uint32_t width,
            const uint32_t height,
            const int type,
            const uint64_t timestamp = 0
        );

        Frame(const Frame& frame);

        Frame(Frame&& frame) noexcept;

        virtual ~Frame() = default;

        Frame& operator=(Frame&& frame) noexcept;

        void default_to(const cv::Size& size, const int type);

        void default_to(const uint32_t width, const uint32_t height, const int type);

        void allocate(const cv::Size& size, const int type);

        void allocate(const uint32_t width, const uint32_t height, const int type);

        void copy(const cv::UMat& src);

        void copy(const Frame& src);

        Frame clone() const;

        uint32_t width() const;

        uint32_t height() const;

        cv::Size size() const;

        bool is_empty() const;

        int type() const;

    };
}