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
#include "Functions/Logic.hpp"
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
        LVK_ASSERT(settings.motion_resolution.height >= WarpMesh::MinimumSize.height);
        LVK_ASSERT(settings.motion_resolution.width >= WarpMesh::MinimumSize.width);
        LVK_ASSERT(settings.predictive_samples > 0);
        LVK_ASSERT(settings.smoothing_steps > 0.0f);
        LVK_ASSERT_01(settings.corrective_limit);
        LVK_ASSERT_01(settings.response_rate);

        // Update motion resolution.
        if(m_Position.size() != settings.motion_resolution)
        {
            m_Trajectory.fill(settings.motion_resolution);
            m_Trace = WarpMesh(settings.motion_resolution);
            m_Position = WarpMesh(settings.motion_resolution);
        }

        // Update trajectory sizing.
        if(const auto window_size = 2 * settings.predictive_samples + 1; m_Trajectory.size() != window_size)
        {
            // The trajectory is held in a circular buffer representing a windowed view on the
            // full path. The size of the window is based on the number of predictive samples
            // and is symmetrical with the center element, representing the current position.
            // When resizing, always pad the front to avoid invalid time-shifts in the data.
            m_Trajectory.resize(2 * settings.predictive_samples + 1);
            m_Trajectory.pad_front(settings.motion_resolution);

            // Reset the current position tracker.
            m_Position = m_Trajectory.oldest();
            for(size_t i = 1; i <= m_Trajectory.centre_index(); i++)
            {
                m_Position += m_Trajectory[i];
            }

            // Adjust the base factor to stay consistent with different sample counts.
            m_BaseSmoothingFactor = static_cast<double>(m_Trajectory.capacity()) / 12.0;
        }

        m_SceneMargins = crop<float>({1,1}, settings.corrective_limit);
        m_SceneCrop = WarpMesh(settings.motion_resolution);
        m_SceneCrop.crop_in(m_SceneMargins);

        m_Settings = settings;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh PathSmoother::next(const WarpMesh& motion)
    {
        LVK_ASSERT(motion.size() == m_Settings.motion_resolution);

        // Update the path's current state.
        m_Position -= m_Trajectory.oldest();
        m_Trajectory.push(motion);
        m_Position += m_Trajectory.centre();

        // Generate adaptive smoothing filter.
        const cv::Mat filter = cv::getGaussianKernel(
            static_cast<int>(m_Trajectory.capacity()),
            m_BaseSmoothingFactor + m_SmoothingFactor,
            CV_32F
        );

        // Apply the filter to get smooth path correction.
        float weight = 1.0f;
        m_Trace = m_Trajectory.oldest();
        for(size_t i = 1; i < m_Trajectory.size(); i++)
        {
            weight -= filter.at<float>(static_cast<int>(i) - 1);
            m_Trace.combine(m_Trajectory[i], weight);
        }
        auto path_correction = m_Trace - m_Position;

        // Determine how much our smoothed path trace has drifted away from the path,
        // as a percentage of the corrective limits (1.0+ => out of scene bounds).
        float max_drift_error = 0.0f;
        path_correction.read([&](const cv::Point2f& drift, const cv::Point& coord){
            const auto x_drift = std::abs(drift.x) / m_SceneMargins.x;
            const auto y_drift = std::abs(drift.y) / m_SceneMargins.y;
            max_drift_error = std::max(max_drift_error, x_drift);
            max_drift_error = std::max(max_drift_error, y_drift);
        }, false);

        // Clamp drift within the corrective limits
        if(max_drift_error > 1.0f)
        {
            path_correction.clamp(m_SceneMargins.tl());
            max_drift_error = 1.0f;
        }

        // Adapt the smoothing factor to target a drift of 0.5.
        m_SmoothingFactor = exp_moving_average(
            m_SmoothingFactor,
            hysteresis<double>(max_drift_error, 0.3, m_Settings.smoothing_steps, 0.7, 0.0),
            m_Settings.response_rate
        );

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
        return m_Settings.predictive_samples;
    }

//---------------------------------------------------------------------------------------------------------------------

    const WarpMesh& PathSmoother::scene_crop() const
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
