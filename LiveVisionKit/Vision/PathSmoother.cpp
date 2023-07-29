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

#include "PathSmoother.hpp"

#include "Functions/Math.hpp"
#include "Logging/CSVLogger.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	PathSmoother::PathSmoother(const PathSmootherSettings& settings)
	{
		configure(settings);
	}

//---------------------------------------------------------------------------------------------------------------------

    void PathSmoother::configure(const PathSmootherSettings& settings)
    {
        LVK_ASSERT(settings.motion_resolution.height >= WarpField::MinimumSize.height);
        LVK_ASSERT(settings.motion_resolution.width >= WarpField::MinimumSize.width);
        LVK_ASSERT_01(settings.path_correction_limits.height);
        LVK_ASSERT_01(settings.path_correction_limits.width);
        LVK_ASSERT(settings.path_prediction_samples > 0);
        LVK_ASSERT(settings.max_smoothing_range > 0.0f);
        LVK_ASSERT(settings.min_smoothing_sigma > 0.0f);
        LVK_ASSERT_01(settings.response_rate);

        m_Settings = settings;

        // The trajectory is held in a circular buffer representing a windowed view on the
        // full path. The size of the window is based on the number of predictive samples
        // and is symmetrical with the center element, representing the current position
        // in time. To achieve predictive smoothing the trajectory is implicitly delayed.
        m_Trajectory.resize(2 * settings.path_prediction_samples + 1);

        if(m_Position.size() != settings.motion_resolution)
        {
            m_Trajectory.clear();

            // Re-create all motion fields with the new resolution.
            m_Trajectory.pad_back(settings.motion_resolution);
            m_Position = WarpField(settings.motion_resolution);
            m_Trace = WarpField(settings.motion_resolution);
        }
        else
        {
            // NOTE: Trajectory is always kept full to avoid edge cases. We also
            // always pad from the front so that the existing data stays current.
            m_Trajectory.pad_front(settings.motion_resolution);
        }

        m_SceneMargins = crop<float>({1,1}, settings.path_correction_limits);
        m_SceneCrop = WarpField(settings.motion_resolution);
        m_SceneCrop.crop_in(m_SceneMargins);
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField PathSmoother::next(const WarpField& motion)
    {
        LVK_ASSERT(motion.size() == m_Settings.motion_resolution);

        // Update the path's current state.
        m_Position -= m_Trajectory.oldest();
        m_Trajectory.push(motion);
        m_Position += m_Trajectory.centre();


        // Determine how much our smoothed path trace has drifted away from the path,
        // as a percentage of the corrective limits (1.0+ => out of scene bounds).
        cv::absdiff(m_Trace, m_Position, m_Trace);
        m_Trace /= m_SceneMargins.tl();

        double min_drift_error, max_drift_error;
        cv::minMaxIdx(m_Trace, &min_drift_error, &max_drift_error);
        max_drift_error = std::min(max_drift_error, 1.0);


        // Adapt the smoothing factor based on the max drift error. If the trace
        // is close to the original path, the smoothing coefficient is raised to
        // maximise the smoothing applied. If the trace starts drifting away from
        // the path and closer to the corrective limits, the smoothing is lowered
        // to bring the trace back towards the path.
        m_SmoothingFactor = exp_moving_average(
            m_SmoothingFactor,
            m_Settings.max_smoothing_range * (1.0 - max_drift_error) + m_Settings.min_smoothing_sigma,
            m_Settings.response_rate
        );
        const cv::Mat smoothing_filter = cv::getGaussianKernel(
            static_cast<int>(m_Trajectory.capacity()),
            m_SmoothingFactor,
            CV_32F
        );

        // Apply the filter to get the current smooth trace position.
        float weight = 1.0f;
        m_Trace = m_Trajectory.oldest();
        for(size_t i = 1; i < m_Trajectory.size(); i++)
        {
            weight -= smoothing_filter.at<float>(static_cast<int>(i) - 1);
            m_Trace.combine(m_Trajectory[i], weight);
        }

        // Find the corrective motion for smoothing.
        auto path_correction = m_Trace - m_Position;
        path_correction.clamp(m_SceneMargins.tl());

        return std::move(path_correction);
    }

//---------------------------------------------------------------------------------------------------------------------

    void PathSmoother::restart()
    {
        // Trajectory should always be full, so we can just clear it.
        for(auto& motion : m_Trajectory) motion.set_identity();
        m_Position.set_identity();
        m_Trace.set_identity();
    }

//---------------------------------------------------------------------------------------------------------------------

    size_t PathSmoother::time_delay() const
    {
        return m_Settings.path_prediction_samples;
    }

//---------------------------------------------------------------------------------------------------------------------

    const WarpField& PathSmoother::scene_crop() const
    {
        return m_SceneCrop;
    }

//---------------------------------------------------------------------------------------------------------------------

    const cv::Rect2f& PathSmoother::scene_margins() const
    {
        return m_SceneMargins;
    }

//---------------------------------------------------------------------------------------------------------------------

}
