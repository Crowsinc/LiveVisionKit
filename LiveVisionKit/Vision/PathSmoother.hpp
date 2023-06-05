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
#include "Filters/VideoFrame.hpp"
#include "Utility/Configurable.hpp"
#include "Structures/StreamBuffer.hpp"

namespace lvk
{

    struct PathSmootherSettings
    {
        // NOTE: frame delay is proportional to prediction frames
        size_t path_prediction_samples = 10;

        bool force_spatial_stability = true;
        float instability_tolerance = 5.0f;
    };

    class PathSmoother final : public Configurable<PathSmootherSettings>
    {
    public:

        explicit PathSmoother(const PathSmootherSettings& settings = {});

        void configure(const PathSmootherSettings& settings) override;

        WarpField next(const WarpField& motion, const cv::Size2f& limits);

        WarpField position() const;

        size_t time_delay() const;

        void restart();

    private:

        void resize_fields(const cv::Size& new_size);

    private:
        double m_SmoothingFactor = 0.0;
        StreamBuffer<WarpField> m_Path{1};
        WarpField m_Trace{WarpField::MinimumSize};
    };

}