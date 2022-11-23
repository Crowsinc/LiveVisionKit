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

#include "Utility/Properties/Unique.hpp"
#include "Utility/Timing/TickTimer.hpp"
#include "Diagnostics/Logging/Logger.hpp"

namespace lvk
{

	// TODO: Deal with format changes in video processing for CLI 
	// That is, videos may be of different formats (BGR, YUV, etc.)

	struct Frame
	{
		cv::UMat data;
		uint64_t timestamp;

		static Frame&& Allocate(
			const cv::Size& size,
			const int type,
			const uint32_t timestamp = 0
		);

		static Frame&& Allocate(
			const uint32_t width,
			const uint32_t height,
			const int type,
			const uint32_t timestamp = 0
		);

		static Frame&& Wrap(
			cv::UMat& frame,
			const uint32_t timestamp = 0
		);

		static Frame&& Copy(
			const cv::UMat& frame,
			const uint32_t timestamp = 0
		);

		Frame();

		Frame(Frame&& frame);

		virtual ~Frame() = default;

		void operator=(Frame&& frame);

		void try_allocate(const cv::Size& size, const int type);
		
		void try_allocate(const uint32_t width, const uint32_t height, const int type);

		void copy_from(const cv::UMat& src);
		
		void copy_from(const Frame& src);
		
		Frame&& copy() const;

		uint32_t width() const;

		uint32_t height() const;

		cv::Size size() const;

		bool empty() const;

		int type() const;

	};



	class VideoFilter : public Unique<VideoFilter>
	{
	public:

		VideoFilter(const std::string filter_name);

		virtual ~VideoFilter() = default;

		virtual void process(
			const Frame& input,
			Frame& output,
			const bool debug = false
		) = 0;

		void process(
			cv::VideoCapture& input_stream,
			cv::VideoWriter& output_stream,
			const bool debug = false,
			const std::function<void(VideoFilter&, Frame&)> callback = [](auto&, auto&) {}
		);
		
		virtual void profile(
			const Frame& input,
			Frame& output,
			Stopwatch& timer,
			const bool debug = false
		);

		void profile(
			cv::VideoCapture& input_stream,
			cv::VideoWriter& output_stream,
			Stopwatch& timer,
			const bool debug = false,
			const std::function<void(VideoFilter&, Frame&, Stopwatch&)> callback = [](auto&, auto&, auto&) {}
		);

		const std::string& alias() const;

	private:
		const std::string m_Alias;
	};

}
