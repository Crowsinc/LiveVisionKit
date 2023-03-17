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

#include "CameraCalibrator.hpp"

#include "Diagnostics/Directives.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	CameraCalibrator::CameraCalibrator(const cv::Size& pattern_size)
		: m_PatternSize(pattern_size),
		  m_DetectionFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY)
	{
		LVK_ASSERT(!pattern_size.empty());

		reset();
	}

//---------------------------------------------------------------------------------------------------------------------

	bool CameraCalibrator::
    feed(cv::UMat& frame, const bool draw_corners)
	{
		if(m_ImageSize.empty())
			m_ImageSize = frame.size();

		LVK_ASSERT(frame.size() == m_ImageSize);

		// Extract Y plane for detection.
		cv::extractChannel(frame, m_DetectionFrame, 0);

		std::vector<cv::Point2f> corners;
		const bool found = cv::findChessboardCorners(
			m_DetectionFrame,
			m_PatternSize,
			corners,
			cv::CALIB_CB_ADAPTIVE_THRESH
		);

		if(found)
		{
			cv::cornerSubPix(
				m_DetectionFrame,
				corners,
				cv::Size(11,11),
				cv::Size(-1,-1),
				cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001)
			);

			m_ImagePoints.emplace_back(corners);
		}

		if(draw_corners)
			cv::drawChessboardCorners(frame, m_PatternSize, corners, found);

		return found;
	}

//---------------------------------------------------------------------------------------------------------------------

	CameraParameters CameraCalibrator::calibrate(const uint32_t square_size) const
	{
		LVK_ASSERT(calibration_frames() > 0);
		LVK_ASSERT(square_size > 0);

		std::vector<std::vector<cv::Point3f>> object_points;
		for(uint32_t i = 0; i < m_ImagePoints.size(); i++)
		{
			auto& points = object_points.emplace_back();

			for(int r = 0; r < m_PatternSize.height; r++)
				for(int c = 0; c < m_PatternSize.width; c++)
					points.emplace_back(c * square_size, r * square_size, 0);
		}

		CameraParameters parameters;
		cv::calibrateCamera(
			object_points,
			m_ImagePoints,
			m_ImageSize,
			parameters.camera_matrix,
			parameters.distortion_coefficients,
			cv::noArray(),
			cv::noArray()
		);

		return parameters;
	}

//---------------------------------------------------------------------------------------------------------------------

	void CameraCalibrator::reset()
	{
		m_ImageSize = cv::Size(0, 0);
		m_ImagePoints.clear();

		m_DetectionFrame.release();
	}

//---------------------------------------------------------------------------------------------------------------------

	uint32_t CameraCalibrator::calibration_frames() const
	{
		return m_ImagePoints.size();
	}

//---------------------------------------------------------------------------------------------------------------------

}
