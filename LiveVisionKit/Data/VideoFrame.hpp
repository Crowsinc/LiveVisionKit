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
    struct VideoFrame : public cv::UMat
    {
        enum Format {BGR, RGB, YUV, GRAY, UNKNOWN};

        uint64_t timestamp = 0;
        Format format = UNKNOWN;
        int& width = cols; int& height = rows;

    public:

        VideoFrame();

        VideoFrame(const VideoFrame& frame);

        VideoFrame(VideoFrame&& frame) noexcept;

        explicit VideoFrame(const uint64_t timestamp);

        VideoFrame(const cv::UMat& frame, const uint64_t timestamp, const Format format = UNKNOWN);

        VideoFrame(cv::UMat&& frame, const uint64_t timestamp, const Format format = UNKNOWN) noexcept;


        virtual ~VideoFrame() = default;


        VideoFrame& operator=(VideoFrame&& frame) noexcept;

        VideoFrame& operator=(const VideoFrame& frame) noexcept;


        VideoFrame clone() const; /* override */

        void copyTo(VideoFrame& dst) const; /* override */

        void copyTo(VideoFrame& dst, cv::InputArray mask) const; /* override */

        void copyTo(cv::OutputArray dst) const; /* override */

        void copyTo(cv::OutputArray dst, cv::InputArray mask) const; /* override */


        void reformat(const Format new_format);

        void reformatTo(VideoFrame& dst, const Format new_format) const;
    };

    typedef VideoFrame Frame;

//    struct Frame
//    {
//        cv::UMat data;
//        uint64_t timestamp;
//
//    public:
//
//        Frame();
//
//        explicit Frame(const uint64_t timestamp);
//
//        explicit Frame(cv::UMat&& frame, const uint64_t timestamp = 0);
//
//        explicit Frame(const cv::UMat& frame, const uint64_t timestamp = 0);
//
//        Frame(const cv::Size& size, const int type, const uint64_t timestamp = 0);
//
//        Frame(const uint32_t width, const uint32_t height, const int type, const uint64_t timestamp = 0);
//
//        Frame(Frame&& frame) noexcept;
//
//        Frame(const Frame& frame);
//
//        virtual ~Frame() = default;
//
//        Frame& operator=(Frame&& frame) noexcept;
//
//        Frame& operator=(const Frame& frame) noexcept;
//
//        void default_to(const cv::Size& size, const int type);
//
//        void default_to(const uint32_t width, const uint32_t height, const int type);
//
//        void allocate(const cv::Size& size, const int type);
//
//        void allocate(const uint32_t width, const uint32_t height, const int type);
//
//        void copy(const cv::UMat& src);
//
//        void copy(const Frame& src);
//
//        Frame clone() const;
//
//        void release();
//
//        uint32_t width() const;
//
//        uint32_t height() const;
//
//        cv::Size size() const;
//
//        bool is_empty() const;
//
//        int type() const;
//
//    };
}