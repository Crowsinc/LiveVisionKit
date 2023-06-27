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
    // NOTE: use camelCase to match the cv::UMat API.
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

        explicit VideoFrame(const cv::UMat& frame, const uint64_t timestamp = 0, const Format format = UNKNOWN);

        explicit VideoFrame(cv::UMat&& frame, const uint64_t timestamp = 0, const Format format = UNKNOWN) noexcept;


        virtual ~VideoFrame() = default;


        VideoFrame& operator=(VideoFrame&& frame) noexcept;

        VideoFrame& operator=(const VideoFrame& frame) noexcept;


        VideoFrame clone() const; /* override */

        void copyTo(VideoFrame& dst) const; /* override */

        void copyTo(VideoFrame& dst, cv::InputArray mask) const; /* override */

        void copyTo(cv::OutputArray dst) const; /* override */

        void copyTo(cv::OutputArray dst, cv::InputArray mask) const; /* override */


        VideoFrame operator()(const cv::Rect& roi) const; /* override */


        bool has_known_format() const;

        void reformat(const Format new_format);

        void reformatTo(VideoFrame& dst, const Format new_format) const;

        // NOTE: Ownership of the view is undefined and should not be modified.
        void viewAsFormat(VideoFrame& view, const Format new_format) const;

    };

    typedef VideoFrame Frame;

}