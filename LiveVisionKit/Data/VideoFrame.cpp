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

#include "VideoFrame.hpp"

#include "Directives.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

    VideoFrame::VideoFrame()
        : cv::UMat(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY)
    {}

//---------------------------------------------------------------------------------------------------------------------

    VideoFrame::VideoFrame(const uint64_t timestamp)
        : cv::UMat(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
          timestamp(timestamp)
    {}

//---------------------------------------------------------------------------------------------------------------------

    VideoFrame::VideoFrame(const VideoFrame& frame)
        : cv::UMat(frame),
          timestamp(frame.timestamp),
          format(frame.format)
    {}

//---------------------------------------------------------------------------------------------------------------------

    VideoFrame::VideoFrame(VideoFrame&& frame) noexcept
        : cv::UMat(std::move(frame)),
          timestamp(frame.timestamp),
          format(frame.format)
    {}

//---------------------------------------------------------------------------------------------------------------------

    VideoFrame::VideoFrame(const cv::UMat& frame, const uint64_t timestamp, const Format format)
        : cv::UMat(frame),
          timestamp(timestamp),
          format(format)
    {}

//---------------------------------------------------------------------------------------------------------------------

    VideoFrame::VideoFrame(cv::UMat&& frame, const uint64_t timestamp, const Format format) noexcept
        : cv::UMat(std::move(frame)),
          timestamp(timestamp),
          format(format)
    {}

//---------------------------------------------------------------------------------------------------------------------

    VideoFrame& VideoFrame::operator=(VideoFrame&& frame) noexcept
    {
        format = frame.format;
        timestamp = frame.timestamp;
        cv::UMat::operator=(std::move(frame));

        return *this;
    }

//---------------------------------------------------------------------------------------------------------------------

    VideoFrame& VideoFrame::operator=(const VideoFrame& frame) noexcept
    {
        format = frame.format;
        timestamp = frame.timestamp;
        cv::UMat::operator=(frame);

        return *this;
    }

//---------------------------------------------------------------------------------------------------------------------

    VideoFrame VideoFrame::clone() const /* override */
    {
        return VideoFrame(
            std::move(cv::UMat::clone()),
            timestamp,
            format
        );
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoFrame::copyTo(VideoFrame& dst) const /* override */
    {
        cv::UMat::copyTo(dst);
        dst.timestamp = timestamp;
        dst.format = format;
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoFrame::copyTo(VideoFrame& dst, cv::InputArray mask) const /* override */
    {
        cv::UMat::copyTo(dst, mask);
        dst.timestamp = timestamp;
        dst.format = format;
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoFrame::copyTo(cv::OutputArray dst) const /* override */
    {
        cv::UMat::copyTo(dst);
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoFrame::copyTo(cv::OutputArray dst, cv::InputArray mask) const /* override */
    {
        cv::UMat::copyTo(dst, mask);
    }

//---------------------------------------------------------------------------------------------------------------------

    VideoFrame VideoFrame::operator()(const cv::Rect& roi) const /* override */
    {
        return VideoFrame(
            std::move(cv::UMat::operator()(roi)),
            timestamp,
            format
        );
    }

//---------------------------------------------------------------------------------------------------------------------

    bool VideoFrame::has_known_format() const
    {
        return format != UNKNOWN;
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoFrame::reformat(const VideoFrame::Format new_format)
    {
        LVK_ASSERT(new_format != UNKNOWN);
        LVK_ASSERT(format != UNKNOWN);

        if(new_format != format)
        {
            thread_local VideoFrame format_buffer;
            reformatTo(format_buffer, new_format);
            std::swap(*this, format_buffer);
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoFrame::reformatTo(VideoFrame& dst, const VideoFrame::Format new_format) const
    {
        LVK_ASSERT(new_format != UNKNOWN);
        LVK_ASSERT(format != UNKNOWN);
        LVK_ASSERT(u != dst.u);

        // Copy if no format change is required.
        if(new_format == format)
        {
            copyTo(dst);
            return;
        }

        // Convert the old format into the new format.
        switch(format)
        {
            case Format::BGR:
            {
                // BGR to ...
                switch(new_format)
                {
                    case Format::GRAY: cv::cvtColor(*this, dst, cv::COLOR_BGR2GRAY); break;
                    case Format::RGB: cv::cvtColor(*this, dst, cv::COLOR_BGR2RGB); break;
                    case Format::YUV: cv::cvtColor(*this, dst, cv::COLOR_BGR2YUV); break;
                    default: LVK_ASSERT("Unsupported BGR conversion" && false);
                }
                break;
            }
            case Format::RGB:
            {
                // RGB to ...
                switch(new_format)
                {
                    case Format::GRAY: cv::cvtColor(*this, dst, cv::COLOR_RGB2GRAY); break;
                    case Format::BGR: cv::cvtColor(*this, dst, cv::COLOR_RGB2BGR); break;
                    case Format::YUV: cv::cvtColor(*this, dst, cv::COLOR_RGB2YUV); break;
                    default: LVK_ASSERT("Unsupported RGB conversion" && false);
                }
                break;
            }
            case Format::YUV:
            {
                // YUV to ...
                switch(new_format)
                {
                    case Format::GRAY: cv::extractChannel(*this, dst, 0); break;
                    case Format::BGR: cv::cvtColor(*this, dst, cv::COLOR_YUV2BGR); break;
                    case Format::RGB: cv::cvtColor(*this, dst, cv::COLOR_YUV2RGB); break;
                    default: LVK_ASSERT("Unsupported YUV conversion" && false);
                }
                break;
            }
            case Format::GRAY:
            {
                // Gray to ...
                switch(new_format)
                {
                    case Format::RGB: cv::cvtColor(*this, dst, cv::COLOR_GRAY2RGB); break;
                    case Format::BGR: cv::cvtColor(*this, dst, cv::COLOR_GRAY2BGR); break;
                    case Format::YUV:
                    {
                        // The Y plane is equal to OpenCV's grayscale format. The other planes are zero.
                        thread_local cv::UMat zero_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
                        if(zero_buffer.empty() || zero_buffer.size() != size())
                        {
                            zero_buffer = zeros(size(), type());
                        }
                        cv::merge(std::vector<cv::UMat>{*this, zero_buffer, zero_buffer}, dst);
                        break;
                    }
                    default: LVK_ASSERT("Unsupported GRAY conversion" && false);
                }
                break;
            }
            default: LVK_ASSERT("Unsupported conversion" && false);
        }

        // Update metadata.
        dst.timestamp = timestamp;
        dst.format = new_format;
    }

//---------------------------------------------------------------------------------------------------------------------
}