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

#include <functional>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include "VideoFrame.hpp"
#include "Utility/Properties/Unique.hpp"
#include "Utility/Timing/TickTimer.hpp"

namespace lvk
{

	class VideoFilter : public Unique<VideoFilter>
	{
	public:

		explicit VideoFilter(const std::string& filter_name = "Identity Filter");

		virtual ~VideoFilter() = default;

        // NOTE: standard colour format is YUV.
        void process(
            const Frame& input,
            Frame& output,
            bool debug = false
        );

        void process(
            cv::VideoCapture& input_stream,
            const std::function<bool(VideoFilter&, Frame&)>& callback,
            const bool debug = false
        );

        void render(
            const Frame& input,
            bool debug = false
        );

        void render(
            cv::VideoCapture& input_stream,
            const uint32_t target_fps = 0,
            const bool debug = false
        );

        void set_timing_samples(const uint32_t samples);

        const Stopwatch& timings() const;

		const std::string& alias() const;

    protected:

        virtual void filter(
            const Frame& input,
            Frame& output,
            Stopwatch& timer,
            const bool debug
        );

    private:
        Frame m_FrameBuffer;
        Stopwatch m_FrameTimer;
		const std::string m_Alias;
	};

    // Default VideoFilter is essentially an identity filter.
    typedef VideoFilter IdentityFilter;
}
