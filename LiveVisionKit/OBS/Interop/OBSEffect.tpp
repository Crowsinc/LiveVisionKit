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

namespace lvk
{
//---------------------------------------------------------------------------------------------------------------------
	
	// Render source
	template<typename E, typename...Args>
	bool OBSEffect<E, Args...>::Render(
		obs_source_t* source,
		const cv::Size render_size,
		Args... args
	)
	{
		LVK_ASSERT(source != nullptr);

		auto& effect = E::Instance();

		const auto target = obs_filter_get_target(source);
		const cv::Size source_size(
			obs_source_get_base_width(target),
			obs_source_get_base_height(target)
		);

		if (!is_render_valid(source, source_size, render_size, effect, args...))
			return false;

		// Perform normal rendering of source filter
		if (obs_source_process_filter_begin(source, GS_RGBA, OBS_ALLOW_DIRECT_RENDERING))
		{
			obs_source_process_filter_tech_end(
				source,
				effect.handle(),
				render_size.width,
				render_size.height,
				effect.configure(source_size, render_size, args...)
			);

			return true;
		}
		return false;
	}

//---------------------------------------------------------------------------------------------------------------------

	// Render texture to source
	template<typename E, typename...Args>
	bool OBSEffect<E, Args...>::Render(
		gs_texture_t* texture,
		const cv::Size render_size,
		Args... args
	)
	{
		LVK_ASSERT(texture != nullptr);
		
		auto& effect = E::Instance();

		const cv::Size source_size(
			gs_texture_get_width(texture),
			gs_texture_get_height(texture)
		);

		if (!is_render_valid(nullptr, source_size, render_size, effect, args...))
			return false;

		const bool use_srgb = gs_get_linear_srgb();
		const bool prev_srgb = gs_framebuffer_srgb_enabled();

		gs_enable_framebuffer_srgb(use_srgb);

		auto image_param = gs_effect_get_param_by_name(effect.handle(), "image");
		if (use_srgb)
			gs_effect_set_texture_srgb(image_param, texture);
		else
			gs_effect_set_texture(image_param, texture);

		auto technique = effect.configure(source_size, render_size, args...);
		while (gs_effect_loop(effect.handle(), technique))
			gs_draw_sprite(texture, false, render_size.width, render_size.height);

		gs_enable_framebuffer_srgb(prev_srgb);

		return true;
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename E, typename...Args>
	bool OBSEffect<E, Args...>::Validate()
	{
		auto& effect = E::Instance();
		return effect.handle() != nullptr && effect.validate();
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename E, typename...Args>
	E& OBSEffect<E,Args...>::Instance()
	{
		static E effect;
		return effect;
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename E, typename...Args>
	OBSEffect<E, Args...>::OBSEffect(const std::string& name)
		: m_Handle(nullptr),
		  m_Owner(true)
	{
		char* effect_path = obs_module_file(
			("effects/" + name + ".effect").c_str()
		);

		LVK_ASSERT(effect_path != nullptr);

		if (effect_path != nullptr)
		{
			obs_enter_graphics();

			m_Handle = gs_effect_create_from_file(effect_path, nullptr);
			bfree(effect_path);

			obs_leave_graphics();
		}
	}
	
//---------------------------------------------------------------------------------------------------------------------

	template<typename E, typename...Args>
	OBSEffect<E, Args...>::OBSEffect(gs_effect_t* handle)
		: m_Handle(handle),
		  m_Owner(false)
	{}

//---------------------------------------------------------------------------------------------------------------------

	template<typename E, typename...Args>
	OBSEffect<E, Args...>::~OBSEffect()
	{
		if (m_Owner && m_Handle != nullptr)
		{
			obs_enter_graphics();
			gs_effect_destroy(m_Handle);
			obs_leave_graphics();
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename E, typename...Args>
	bool OBSEffect<E, Args...>::is_render_valid(
		obs_source_t* source,
		const cv::Size source_size,
		const cv::Size render_size,
		E& effect,
		Args... args
	)
	{
		return (source == nullptr || obs_filter_get_parent(source) != nullptr)
			&& (source == nullptr || obs_filter_get_target(source) != nullptr)
			&& source_size.width > 0 && source_size.height > 0
			&& render_size.width > 0 && render_size.height > 0
			&& !effect.should_skip(source_size, render_size, args...);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename E, typename...Args>
	gs_effect_t* OBSEffect<E, Args...>::handle()
	{
		return m_Handle;
	}

//---------------------------------------------------------------------------------------------------------------------
	
	template<typename E, typename...Args>
	gs_eparam_t* OBSEffect<E, Args...>::load_param(const char* name)
	{
		return gs_effect_get_param_by_name(handle(), name);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename E, typename...Args>
	bool OBSEffect<E, Args...>::validate() const
	{
		return true;
	}

//---------------------------------------------------------------------------------------------------------------------
	
	template<typename E, typename...Args>
	const char* OBSEffect<E, Args...>::configure(
		const cv::Size input_size,
		const cv::Size output_size,
		Args... args
	)
	{
		return "Draw";
	}

//---------------------------------------------------------------------------------------------------------------------
	
	template<typename E, typename...Args>
	bool OBSEffect<E, Args...>::should_skip(
		const cv::Size input_size,
		const cv::Size output_size,
		Args... args
	) const
	{
		return false;
	}

//---------------------------------------------------------------------------------------------------------------------

}
