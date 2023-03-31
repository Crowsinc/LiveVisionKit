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

#include <memory>

#include "VideoFilter.hpp"
#include "Utility/Configurable.hpp"

namespace lvk
{

    struct CompositeFilterSettings
    {
        std::vector<std::shared_ptr<lvk::VideoFilter>> filter_chain;
        bool save_outputs = false;

        // TODO: add pipelineing option
    };

    class CompositeFilter final : public VideoFilter, public Configurable<CompositeFilterSettings>
    {
    public:

        explicit CompositeFilter(const CompositeFilterSettings& settings = {});

        CompositeFilter(
            const std::initializer_list<std::shared_ptr<lvk::VideoFilter>>& filter_chain,
            const CompositeFilterSettings& settings = {}
        );

        void configure(const CompositeFilterSettings& settings) override;

        const std::vector<std::shared_ptr<lvk::VideoFilter>>& filters() const;

        std::shared_ptr<lvk::VideoFilter> filters(const size_t index);

        const std::vector<Frame>& outputs() const;

        const Frame& outputs(const size_t index);

        bool is_filter_enabled(const size_t index);

        void disable_filter(const size_t index);

        void enable_filter(const size_t index);

        void enable_all_filters();

        size_t filter_count() const;

    private:

        void filter(
            Frame&& input,
            Frame& output,
            Stopwatch& timer,
            const bool debug
        ) override;

        std::vector<bool> m_FilterRunState;
        std::vector<Frame> m_FilterOutputs;
    };

}

