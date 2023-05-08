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

    struct PathStabilizerSettings
    {
        // NOTE: frame delay is proportional to smoothing samples
        size_t path_prediction_frames = 10;

        float scene_margins = 0.1f;
        bool clamp_path_to_margins = true;

        float rigidity_tolerance = 0.2f;
        bool force_output_rigidity = true;
    };

    class PathStabilizer final : public Configurable<PathStabilizerSettings>
    {
    public:

        explicit PathStabilizer(const PathStabilizerSettings& settings = {});

        void configure(const PathStabilizerSettings& settings) override;

        Frame next(const Frame& frame, const WarpField& motion);

        Frame next(Frame&& frame, const WarpField& motion);

        void restart();

        bool ready() const;

        size_t frame_delay() const;

        WarpField position() const;

        const cv::Rect& stable_region() const;

    private:

        void configure_buffers();

        void resize_fields(const cv::Size& new_size);

    private:
        double m_SmoothingFactor = 0.0;
        StreamBuffer<WarpField> m_Path;
        WarpField m_Trace{WarpField::MinimumSize};

        cv::Rect m_Margins{0,0,0,0};
        StreamBuffer<Frame> m_FrameQueue;
        cv::UMat m_WarpFrame{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
    };

}