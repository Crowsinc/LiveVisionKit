#pragma once

#include <obs/obs.h>

namespace lvk
{

	class CASFilter
	{
	public:

		static obs_properties_t* Properties();

		static void LoadDefaults(obs_data_t* settings);

		static CASFilter* Create(obs_source_t* context);

	public:

		~CASFilter();

		void configure(obs_data_t* settings);

		void render();

		uint32_t width() const;

		uint32_t height() const;

	private:

		obs_source_t* m_Context;
		gs_effect_t* m_Shader;

		vec4 m_CASConst1;
		gs_eparam_t* m_CASConstParam1;
		gs_eparam_t* m_OutputSizeParam;
		vec2 m_OutputSize;

		CASFilter(obs_source_t* context);

		bool validate() const;

	};

}
