#include "FrameIngest.hpp"

#include <obs/obs-module.h>
#include <obs/obs-source.h>
#include <obs/obs.h>


namespace lvk
{

	//-------------------------------------------------------------------------------------




	//-------------------------------------------------------------------------------------

	void ingest(const obs_source_frame& frame, cv::Mat& dst, bool clone)
	{
//		switch(frame.format)
//		{
//			case video_format::VIDEO_FORMAT_I420:
//			{
//
//
//				break;
//			}
//
//
//
//
//
//
//		}

		if(clone)
			dst.copyTo(dst);
	}

	//-------------------------------------------------------------------------------------

	void operator<<(cv::Mat& dst, const obs_source_frame& frame)
	{
		// NOTE: we never clone when using the << operator
		ingest(frame, dst, false);
	}

	//-------------------------------------------------------------------------------------

}
