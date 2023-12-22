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

#include "IngestTestFilter.hpp"

#include "Utility/Logging.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	constexpr std::array<video_format, 16> TEST_FORMATS = {
        VIDEO_FORMAT_I420,
        VIDEO_FORMAT_NV12,
        VIDEO_FORMAT_YVYU,
        VIDEO_FORMAT_YUY2,
        VIDEO_FORMAT_UYVY,
        VIDEO_FORMAT_RGBA,
        VIDEO_FORMAT_BGRA,
        VIDEO_FORMAT_BGRX,
        VIDEO_FORMAT_Y800,
        VIDEO_FORMAT_I444,
        VIDEO_FORMAT_BGR3,
        VIDEO_FORMAT_I422,
        VIDEO_FORMAT_I40A,
        VIDEO_FORMAT_I42A,
        VIDEO_FORMAT_YUVA,
        VIDEO_FORMAT_AYUV
    };
    constexpr size_t TEST_FORMAT_DURATION = 30;

//---------------------------------------------------------------------------------------------------------------------

	IngestTestFilter::IngestTestFilter(obs_source_t* context)
		: VisionFilter(context),
		  m_Context(context)
	{
		LVK_ASSERT(context != nullptr);
	}

//---------------------------------------------------------------------------------------------------------------------

	void IngestTestFilter::filter(OBSFrame& frame)
	{
        auto format = TEST_FORMATS[m_FormatIndex / TEST_FORMAT_DURATION];
        const std::string format_name = get_video_format_name(format);

        log::print("Starting ingest test for %s...", format_name.c_str());

        // Create OBS frame of the desired test format.
        auto test_frame = obs_source_frame_create(format, frame.cols, frame.rows);

        // Convert to the new format and back, timing both conversions.
        m_DownloadTimer.sync_gpu().start();
        frame.to_obs_frame(test_frame);
        m_DownloadTimer.stop();

        m_UploadTimer.sync_gpu().start();
        frame.from_obs_frame(test_frame);
        m_UploadTimer.stop();

        // Clean up the obs frame
        obs_source_frame_destroy(test_frame);

        // Output debug info
        blog(LOG_INFO,
            "    OCL Upload Time: %.2fms\n"
            "    OCL Download Time: %.2fms",
            m_UploadTimer.elapsed().milliseconds(),
            m_DownloadTimer.elapsed().milliseconds()
        );
        log::print("%s Test Completed!", format_name.c_str());

        draw_text(frame, format_name, cv::Point(20,50), yuv::MAGENTA, 1);

        m_FormatIndex = (m_FormatIndex + 1) % (TEST_FORMATS.size() * TEST_FORMAT_DURATION);
    }

//---------------------------------------------------------------------------------------------------------------------

}
