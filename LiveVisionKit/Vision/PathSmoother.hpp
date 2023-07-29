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

#pragma once

#include <opencv2/opencv.hpp>

#include "Math/WarpField.hpp"
#include "Data/StreamBuffer.hpp"
#include "Utility/Configurable.hpp"

namespace lvk
{

    struct PathSmootherSettings
    {
        cv::Size motion_resolution = {2, 2};

        // NOTE: introduces time delay.
        size_t path_prediction_samples = 10;
        cv::Size2f path_correction_limits = {0.1f, 0.1f};

        float max_smoothing_range = 10.0f;
        float min_smoothing_factor = 3.0f;
        float response_rate = 0.08f;
    };

    class PathSmoother final : public Configurable<PathSmootherSettings>
    {
    public:

        explicit PathSmoother(const PathSmootherSettings& settings = {});

        void configure(const PathSmootherSettings& settings) override;

        WarpField next(const WarpField& motion);

        void restart();

        size_t time_delay() const;

        const WarpField& scene_crop() const;

        const cv::Rect2f& scene_margins() const;

    private:
        double m_SmoothingFactor = 0.0f;
        StreamBuffer<WarpField> m_Trajectory{1};
        WarpField m_Trace{WarpField::MinimumSize};
        WarpField m_Position{WarpField::MinimumSize};

        cv::Rect2f m_SceneMargins{0,0,0,0};
        WarpField m_SceneCrop{WarpField::MinimumSize};
    };


}