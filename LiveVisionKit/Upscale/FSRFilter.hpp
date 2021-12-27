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

		//TODO:
		static void LoadDefaults(obs_data_t* settings);

		//TODO:
		static obs_properties_t* Properties();

		static FSRFilter* Create(obs_source_t* context);

		static const char* Name();

	public:

		~FSRFilter();

		void configure(obs_data_t* settings);

		void tick();

		void render();

		uint32_t width();

		uint32_t height();

	private:

		static constexpr const char* c_FilterName = "LVK - FSR Upscaler";
		static constexpr uint32_t c_DefaultOutputWidth = 1920;
		static constexpr uint32_t c_DefaultOutputHeight = 1080;
		static constexpr float c_DefaultSharpness = 0.2f;

		uint32_t* m_DummyAlloc;
		obs_source_t* m_Context;
		gs_effect_t* m_Shader;

		bool m_EASUOutdated;
		AU1 m_InputWidth, m_InputHeight;
		AU1 m_OutputWidth, m_OutputHeight;
		gs_eparam_t* m_OutputSizeParam;
		gs_eparam_t* m_EASUConstParam0;
		gs_eparam_t* m_EASUConstParam1;
		gs_eparam_t* m_EASUConstParam2;
		gs_eparam_t* m_EASUConstParam3;
		gs_eparam_t* m_RCASConstParam0;
		varAU4(m_EASUConst0);
		varAU4(m_EASUConst1);
		varAU4(m_EASUConst2);
		varAU4(m_EASUConst3);
		varAU4(m_RCASConst0);

		FSRFilter(obs_source_t* context);

		bool validate() const;
	};

}
