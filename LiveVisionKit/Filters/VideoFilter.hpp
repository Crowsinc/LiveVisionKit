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

#include "Utility/Data/Unique.hpp"
#include "Utility/Timing/TickTimer.hpp"
#include "Diagnostics/Logging/Logger.hpp"

namespace lvk
{

	class VideoFilter : public Unique<VideoFilter>
	{
	public:

		VideoFilter(const std::string filter_name, const uint32_t timing_samples = 300);

		virtual ~VideoFilter() = default;

		void process(
			cv::UMat& frame,
			const bool debug = false
		);
		
		void process(
			cv::VideoCapture& input_stream,
			cv::VideoWriter& output_stream,
			const bool debug = false,
			const std::function<void(VideoFilter&,cv::UMat&)> callback = [](auto&,auto&) {}
		);

		void process(
			cv::UMat& frame,
			Logger& logger,
			const bool debug = false
		);

		void process(
			cv::VideoCapture& input_stream,
			cv::VideoWriter& output_stream,
			Logger& logger,
			const bool debug = false,
			const std::function<void(VideoFilter&, cv::UMat&, Logger&)> callback = [](auto&, auto&, auto&) {}
		);
	
		Time runtime() const;

		Time runtime_average() const;

		Time runtime_deviation() const;

		const std::string& alias() const;

	protected:

		virtual void filter(cv::UMat& frame, const bool debug) = 0;

		virtual void write_log(Logger& logger);

	private:
		Stopwatch m_FrameTimer;
		std::string m_Alias;
	};

}
