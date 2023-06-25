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

#include "Utility/Unique.hpp"
#include "Data/VideoFrame.hpp"
#include "Timing/Stopwatch.hpp"

namespace lvk
{

    // NOTE: standard colour format is YUV.
	class VideoFilter : public Unique<VideoFilter>
	{
	public:

		explicit VideoFilter(const std::string& filter_name = "Identity Filter");

		virtual ~VideoFilter() = default;

		const std::string& alias() const;


        void apply(Frame&& input, Frame& output, const bool profile = false);

        void apply(const Frame& input, Frame& output, const bool profile = false);

        void stream(cv::VideoCapture& input, const std::function<bool(Frame&)>& callback, const bool profile = false);


        void set_timing_samples(const size_t samples);

        const Stopwatch& timings() const;

    protected:

        virtual void filter(Frame&& input, Frame& output);

    private:
        Stopwatch m_FrameTimer;
		const std::string m_Alias;
	};

    // Default VideoFilter is an identity filter.
    typedef VideoFilter IdentityFilter;
}
