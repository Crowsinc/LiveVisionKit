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

#pragma once

#include <type_traits>
#include <string>

#include <obs-module.h>
#include <opencv2/opencv.hpp>

namespace lvk
{

	template<typename E, typename... Args>
	class OBSEffect
	{
	public:

		static E& Instance();

		static bool Render(
			obs_source_t* source,
			const cv::Size& render_size,
			Args... args
		);

		static bool Render(
			obs_source_t* source,
			Args... args
		);

		static bool Render(
			gs_texture_t* texture,
			const cv::Size& render_size,
			Args... args
		);

		static bool Render(
			gs_texture_t* texture,
			Args... args
		);

		static bool IsCompiled();

	public:

        OBSEffect(OBSEffect& effect) = delete;

        OBSEffect(OBSEffect&& effect) = delete;

		bool render(
			obs_source_t* source,
			const cv::Size& render_size,
			Args... args
		);

		bool render(
			obs_source_t* source,
			Args... args
		);

		bool render(
			gs_texture_t* texture,
			const cv::Size& render_size,
			Args... args
		);

		bool render(
			gs_texture_t* texture,
			Args... args
		);

		bool is_compiled();

	protected:

		explicit OBSEffect(const std::string& name);

		explicit OBSEffect(gs_effect_t* handle);

		virtual ~OBSEffect() = default;

		virtual const char* configure(
			const cv::Size& source_size,
			const cv::Size& render_size,
			Args... args
		);

		virtual bool should_skip(
			const cv::Size& source_size,
			const cv::Size& render_size,
			Args... args
		) const;

		virtual bool validate() const;

        gs_eparam_t* load_param(const char* name);

        gs_effect_t* handle();

	private:

		bool is_renderable(
			obs_source_t* source,
			const cv::Size& source_size,
			const cv::Size& render_size,
			Args... args
		);

	private:
		// NOTE: the gs_effect_t* is automatically released by OBS when destroying context
		gs_effect_t* m_Handle = nullptr;
	};

}

#include "OBSEffect.tpp"
