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

#include "PathStabilizer.hpp"
#include "Math/Math.hpp"

#include "Diagnostics/Logging/CSVLogger.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	PathStabilizer::PathStabilizer(const PathStabilizerSettings& settings)
	{
		this->configure(settings);
	}

//---------------------------------------------------------------------------------------------------------------------

    void PathStabilizer::configure(const PathStabilizerSettings& settings)
    {
        LVK_ASSERT(between(settings.smoothing_coefficient, 0.0f, 1.0f));
        LVK_ASSERT(between_strict(settings.scene_margins, 0.0f, 1.0f));
        LVK_ASSERT(settings.drift_coefficient > 0.0f);

        m_Settings = settings;
    }

//---------------------------------------------------------------------------------------------------------------------

    Frame PathStabilizer::next(const Frame& frame, const WarpField& motion)
    {
        return next(std::move(frame.clone()), motion);
    }

//---------------------------------------------------------------------------------------------------------------------

    Frame PathStabilizer::next(Frame&& frame, const WarpField& motion)
	{
        LVK_ASSERT(!frame.is_empty());

        // We must ensure the motion resolution stays consistent.
        if(motion.size() != m_Trace.size())
        {
            m_Trace.resize(motion.size());
            m_Position.resize(motion.size());
        }

        m_Margins = crop(frame.size(), m_Settings.scene_margins);
        const cv::Point2f corrective_limits(m_Margins.tl());

        // Update the path position
        m_Position += motion;

        const WarpField drift = m_Position - m_Trace;
        const auto drift_limits = m_Settings.drift_coefficient * corrective_limits;

        // Calculate max drift error from original path
        float max_drift_error = 0.0f;
        drift.read([&](const cv::Point2f& drift, const cv::Point& coord){
            max_drift_error = std::max(max_drift_error, std::abs(drift.x) / drift_limits.x);
            max_drift_error = std::max(max_drift_error, std::abs(drift.y) / drift_limits.y);
        }, false);
        max_drift_error = std::min(max_drift_error, 1.0f);


        // Trace over the original path via an exponential moving average in order
        // to create a smoothed path with minimal frame delay. To achieve acceptable
        // results, the influence of the next position sample is adjusted adaptively,
        // based on the maximum drift error of the trace from the position.
        m_Trace.blend(regulate_influence(max_drift_error), m_Position);

        // Find the corrective motion to move the frame onto the new path.
        auto path_correction = m_Trace - m_Position;
        path_correction.clamp(corrective_limits);

        if(m_Settings.force_rigid_output)
        {
            path_correction.undistort(m_Settings.rigidity_tolerance);
        }

        // NOTE: we perform a swap between the resulting warp frame
        // and the original frame data to ensure zero de-allocations.
        path_correction.warp(frame.data, m_WarpFrame);
        std::swap(m_WarpFrame, frame.data);

        return std::move(frame);
	}

//---------------------------------------------------------------------------------------------------------------------

    float PathStabilizer::regulate_influence(const float drift_error) const
    {
        LVK_ASSERT(between(drift_error, 0.0f, 1.0f));

        // NOTE: Lower influence results in higher smoothing.
        //
        // The influence is regulated via an exponential function which balances the amount
        // smoothing or path catch up that is necessary for the trace based on the drift error.
        //
        // The curve is given by: https://www.desmos.com/calculator/sg34zskyh2
        //
        // When the drift error is small, the path velocity is assumed to be minimal, usually
        // consisting of small high frequency noise such as vibrations. Here, the influence is
        // reduced to increase the smoothing applied to the trace.
        //
        // When the drift error is large, the path velocity is assumed to be large, usually due
        // to fast camera motions that can't be smoothed without distortions. The influence of
        // the motion is increased in this case so that the trace catches up to the original path.
        // This also plays a large role in keeping the trace within the scene margins to avoid
        // distortions caused by clamping the path correction to respect the user's margins.

        // This controls the boundary between the smoothing and catch up sections of the curve,
        // with respect to the drift error. Higher values increase the amount of drift error
        // that is required before the trace starts to catch up to the path again.
        constexpr float catch_up_delay = 15.0f;

        const float min_influence = 1.0f - m_Settings.smoothing_coefficient;
        return (1.0f - min_influence) * std::exp(catch_up_delay * (drift_error - 1.0f)) + min_influence;
    }

//---------------------------------------------------------------------------------------------------------------------

	void PathStabilizer::restart()
	{
		m_Trace.set_identity();
        m_Position.set_identity();
	}

//---------------------------------------------------------------------------------------------------------------------

    WarpField PathStabilizer::raw_position() const
    {
        return m_Position;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField PathStabilizer::smooth_position() const
    {
        return m_Trace;
    }

//---------------------------------------------------------------------------------------------------------------------

	const cv::Rect& PathStabilizer::stable_region() const
	{
		return m_Margins;
	}

//---------------------------------------------------------------------------------------------------------------------

}