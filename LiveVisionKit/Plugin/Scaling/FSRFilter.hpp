#pragma once

#include <obs/obs.h>

namespace lvk
{

	class FSRFilter
	{
	public:

		static obs_properties_t* Properties();

		static void LoadDefaults(obs_data_t* settings);

		static FSRFilter* Create(obs_source_t* context);

	public:

		~FSRFilter();

		void configure(obs_data_t* settings);

		void render();

		uint32_t width() const;

		uint32_t height() const;

	private:

		obs_source_t* m_Context;
		gs_effect_t* m_Shader;

		bool m_EASUMatchCanvas;
		vec2 m_InputSize, m_OutputSize;
		vec2 m_NewOutputSize;

		vec4 m_EASUConst0, m_EASUConst1;
		vec4 m_EASUConst2, m_EASUConst3;
		vec4 m_RCASConst0;

		gs_eparam_t* m_OutputSizeParam;
		gs_eparam_t* m_EASUConstParam0;
		gs_eparam_t* m_EASUConstParam1;
		gs_eparam_t* m_EASUConstParam2;
		gs_eparam_t* m_EASUConstParam3;
		gs_eparam_t* m_RCASConstParam0;
		gs_texrender_t* m_EASURenderTarget;

		FSRFilter(obs_source_t* context);

		void prepare_scaling();

		bool validate() const;
	};

}
