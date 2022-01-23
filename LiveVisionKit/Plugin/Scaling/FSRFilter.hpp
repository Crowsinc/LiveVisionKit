#pragma once

#include <obs/obs.h>

#include <cmath>
#include <cstdint>

// OBS doesn't 'officially' support two separate render passes per filter.
// It works but sometimes leads to unexpected behaviour and potential
// future problems as OBS updates. As a result, the RCAS pass has been
// disabled in favour of implementing a dedicated FidelityFX CAS filter
// to run after FSR in the filter stack.
#define DISABLE_RCAS

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

		void tick();

		void render() const;

		uint32_t width() const;

		uint32_t height() const;

	private:

		obs_source_t* m_Context;
		gs_effect_t* m_Shader;

		bool m_BypassEASU;
		bool m_EASUMatchSource;
		bool m_EASUMatchCanvas;

		vec2 m_NewOutputSize;
		vec2 m_InputSize, m_OutputSize;
		vec4 m_EASUConst0, m_EASUConst1;
		vec4 m_EASUConst2, m_EASUConst3;

		gs_eparam_t* m_OutputSizeParam;
		gs_eparam_t* m_EASUConstParam0;
		gs_eparam_t* m_EASUConstParam1;
		gs_eparam_t* m_EASUConstParam2;
		gs_eparam_t* m_EASUConstParam3;

#ifndef DISABLE_RCAS
		vec4 m_RCASConst0;
		gs_eparam_t* m_RCASConstParam0;
		gs_texture_t* m_EASURenderTarget;

		void prepare_easu_render_target();
#endif

		FSRFilter(obs_source_t* context);

		bool validate() const;

	};

}
