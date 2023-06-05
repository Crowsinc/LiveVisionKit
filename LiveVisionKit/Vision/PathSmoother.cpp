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

    constexpr double MIN_FILTER_SIGMA = 3.0f;
    constexpr double MAX_FILTER_SIGMA = 13.0f;
    constexpr double SIGMA_RESPONSE_RATE = 0.08f;

//---------------------------------------------------------------------------------------------------------------------

	PathSmoother::PathSmoother(const PathSmootherSettings& settings)
	{
		configure(settings);
	}

//---------------------------------------------------------------------------------------------------------------------

    void PathSmoother::configure(const PathSmootherSettings& settings)
    {
        LVK_ASSERT(settings.path_prediction_samples > 0);

        m_Settings = settings;

        // The path is held in a circular buffer representing a windowed view on the
        // full path. The size of the window is based on the number of predictive samples
        // and is symmetrical with the center position representing the current position
        // in time. To achieve predictive smoothing the path is implicitely delayed.
        m_Path.resize(2 * m_Settings.path_prediction_samples + 1);

        // Pad the path to keep the buffer full and ensure the existing path is still recent.
        m_Path.pad_front(m_Path.is_empty() ? WarpField::MinimumSize : m_Path.newest().size());
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField PathSmoother::next(const WarpField& motion, const cv::Size2f& limits)
    {
        LVK_ASSERT(limits.width > 0 && limits.height > 0);

        // Resize past motions if a new size is given.
        if(motion.size() != m_Trace.size())
            resize_fields(motion.size());

        // Update the path's current state.
        m_Path.push(m_Path.newest() + motion);

        // Grab the current path position in time.
        auto& position = m_Path.centre();

        // Determine how much our smoothed path trace has drifted away from the path,
        // as a percentage of the corrective limits (1.0+ => out of scene bounds).
        cv::absdiff(m_Trace.offsets(), position.offsets(), m_Trace.offsets());
        m_Trace /= limits;

        double min_drift_error, max_drift_error;
        cv::minMaxIdx(m_Trace.offsets(), &min_drift_error, &max_drift_error);
        max_drift_error = std::min(max_drift_error, 1.0);


        // Adapt the smoothing kernel based on the max drift error. If the trace
        // is close to the original path, the smoothing coefficient is raised to
        // maximise the smoothing applied. If the trace starts drifting away from
        // the path and closer to the corrective limits, the smoothing is lowered
        // to bring the trace back towards the path.
        m_SmoothingFactor = exp_moving_average(
            m_SmoothingFactor,
            (MAX_FILTER_SIGMA - MIN_FILTER_SIGMA) * (1.0f - max_drift_error) + MIN_FILTER_SIGMA,
            SIGMA_RESPONSE_RATE
        );

        const auto smoothing_kernel = cv::getGaussianKernel(
            static_cast<int>(m_Path.capacity()), m_SmoothingFactor, CV_32F
        );

        // Apply the filter to get the current smooth trace position.
        m_Trace.set_identity();
        auto weight = smoothing_kernel.ptr<float>();
        for(size_t i = 0; i < m_Path.size(); i++, weight++)
        {
            m_Trace.combine(m_Path[i], *weight);
        }

        // Find the corrective motion for smoothing.
        auto path_correction = m_Trace - position;
        if(max_drift_error == 1.0) path_correction.clamp(limits);
        if(m_Settings.force_spatial_stability) path_correction.undistort(m_Settings.instability_tolerance);

        return std::move(path_correction);
    }

//---------------------------------------------------------------------------------------------------------------------

    size_t PathSmoother::time_delay() const
    {
        return m_Settings.path_prediction_samples;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField PathSmoother::position() const
    {
        // NOTE: the path will never be empty.
        return m_Path.centre();
    }

//---------------------------------------------------------------------------------------------------------------------

    void PathSmoother::restart()
    {
        m_Path.clear();

        // Pre-fill the path to avoid edge cases.
        m_Path.pad_back(WarpField::MinimumSize);
    }

//---------------------------------------------------------------------------------------------------------------------

    void PathSmoother::resize_fields(const cv::Size& new_size)
    {
        m_Trace.resize(new_size);
        for(auto& position : m_Path)
        {
            position.resize(new_size);
        }
    }

//---------------------------------------------------------------------------------------------------------------------

}
