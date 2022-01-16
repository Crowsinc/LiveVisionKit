#pragma once

#include <obs/obs.h>

#define A_CPU 1
#include <cmath>
#include <cstdint>
#include "Effects/ffx_a.h"
#include "Effects/ffx_fsr1.h"

namespace lvk
{

	class FSRFilter
	{
	public:

		static obs_properties_t* Properties();

		static void LoadDefaults(obs_data_t* settings);

		static FSRFilter* Create(obs_source_t* context);

		static const char* Name();

	public:

		~FSRFilter();

		void configure(obs_data_t* settings);

		void tick();

		void render() const;

		uint32_t width() const;

		uint32_t height() const;

	private:

		uint32_t* m_DummyAlloc;
		obs_source_t* m_Context;

		gs_effect_t* m_Shader;
		gs_texture_t* m_EASURender;

		bool m_BypassEASU, m_BypassRCAS;
		bool m_EASUMatchSource, m_EASUMatchCanvas;

		vec2 m_InputSize, m_OutputSize;
		vec4 m_EASUConst0, m_EASUConst1;
		vec4 m_EASUConst2, m_EASUConst3;
		vec4 m_RCASConst0;



		gs_eparam_t* m_OutputSizeParam;
		gs_eparam_t* m_EASUConstParam0;
		gs_eparam_t* m_EASUConstParam1;
		gs_eparam_t* m_EASUConstParam2;
		gs_eparam_t* m_EASUConstParam3;
		gs_eparam_t* m_RCASConstParam0;

		FSRFilter(obs_source_t* context);

		bool validate() const;
	};

}
