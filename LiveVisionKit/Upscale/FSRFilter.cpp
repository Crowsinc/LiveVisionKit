#include "FSRFilter.hpp"

#include <filesystem>

namespace lvk
{
	//-------------------------------------------------------------------------------------

	//TODO: move to some common util file
	AF1 reinterpret_float(AU1& val)
	{
		// Re-interprets the bits of a uint32_t as that of a float instead.
		// Don't use this unless you understand exactly what all the consequences are.
		return *reinterpret_cast<AF1*>(&val);
	}

	//-------------------------------------------------------------------------------------

	const char* FSRFilter::Name()
	{
		return FSRFilter::c_FilterName;
	}

	//-------------------------------------------------------------------------------------

	FSRFilter* FSRFilter::Create(obs_source_t* context)
	{
		auto filter = new FSRFilter(context);

		// Validate that filter creation was performed
		// successfully before we return anything. This
		// check also ensures that we don't have to do
		// any checks later as we guarantee that any existing
		// filter instantiation must be fully operational.
		if(!filter->validate())
		{
			delete filter;
			return nullptr;
		}

		return filter;
	}

	//-------------------------------------------------------------------------------------

	FSRFilter::FSRFilter(obs_source_t* context)
		: m_Context(context),
		  m_EASUOutdated(true)
	{
		// Since we are using a C++ class, the FSRFilter should be allocated
		// using the 'new' keyword. Doing so will bypass OBS' memory leak
		// detection, which requires the use of bmalloc/bfree. In order to
		// keep consistency with the rest of OBS, we make a dummy allocation
		// to allow memory leak detection for this class.
		m_DummyAlloc = static_cast<uint32_t*>(bzalloc(sizeof(uint32_t)));

		// Load the FSR shader
		//TODO: use obs_module_file("fsr.effect") instead of absolute path.
		std::filesystem::path shader_path("/home/sdm/Projects/C++/LiveVisionKit/LiveVisionKit/Upscale/Effects/fsr.effect");


		obs_enter_graphics();

		m_Shader = gs_effect_create_from_file(shader_path.c_str(), NULL);
		if(m_Shader)
		{
			// Load all shader uniform/parameter locations
			m_OutputSizeParam = gs_effect_get_param_by_name(m_Shader, "output_size");
			m_EASUConstParam0 = gs_effect_get_param_by_name(m_Shader, "easu_const_0");
			m_EASUConstParam1 = gs_effect_get_param_by_name(m_Shader, "easu_const_1");
			m_EASUConstParam2 = gs_effect_get_param_by_name(m_Shader, "easu_const_2");
			m_EASUConstParam3 = gs_effect_get_param_by_name(m_Shader, "easu_const_3");
			m_RCASConstParam0 = gs_effect_get_param_by_name(m_Shader, "rcas_const_0");
		}

		obs_leave_graphics();


		// Set output size to 1080p. If properly implemented, this
		// shouldn't be necessary as it would get configured to the
		// user's preferred setting before the render function is called.
		vec2_set(&m_OutputSize, c_DefaultOutputWidth, c_DefaultOutputHeight);

		// We set the input to {-1,-1} to force the first render
		// call to update the EASU constants for the shader.
		vec2_set(&m_InputSize, -1.0f, -1.0f);
	}

	//-------------------------------------------------------------------------------------

	FSRFilter::~FSRFilter()
	{
		bfree(m_DummyAlloc);

		// Need this check since we may deallocate a failed filter instantiation
		if(m_Shader)
		{
			obs_enter_graphics();
			gs_effect_destroy(m_Shader);
			obs_leave_graphics();
		}
	}

	//-------------------------------------------------------------------------------------

	void FSRFilter::configure(obs_data_t* settings)
	{
		//TODO: implement properly

		varAU4(con0);
		FsrRcasCon(con0, c_DefaultSharpness); // @suppress("Invalid arguments")

		// Although con0 is defined as being a 32bit unsigned vector for use with FSR.
		// Their bits ultimately re-interpreted as a float in the FSR shader. The re-interpretation
		// of bits functionality is not supported by OBS's shader parser. So we do it here instead.
		// See ffx_fsr1_mod.h for more information.
		vec4_set(&m_RCASConst0, reinterpret_float(con0[0]), reinterpret_float(con0[1]),
				reinterpret_float(con0[2]), reinterpret_float(con0[3]));

	}

	//-------------------------------------------------------------------------------------

	void FSRFilter::tick()
	{
		//TODO: handle downscaling

		auto filter_target = obs_filter_get_target(m_Context);
		const float input_width = obs_source_get_base_width(filter_target);
		const float input_height = obs_source_get_base_height(filter_target);

		// If the EASU constant is flagged as out-dated, or the input source size
		// has changed. Then we need to re-compute the EASU constants.
		if(m_EASUOutdated || input_width != m_InputSize.x || input_height != m_InputSize.y)
		{
			vec2_set(&m_InputSize, input_width, input_height);
			m_EASUOutdated = false;

			// NOTE: the constants should be defined by varAU4, but we skip
			// the macro and define it ourselves for brevity.
			AU1 con0[4], con1[4], con2[4], con3[4];
			FsrEasuCon(con0, con1, con2, con3,
					m_InputSize.x, m_InputSize.y, m_InputSize.x, m_InputSize.y,
					m_OutputSize.x, m_OutputSize.y);

			// Although con0-con3 are defined as being uint32_t vectors for use with FSR.
			// Their bits ultimately re-interpreted as a float in the FSR shader. The re-interpretation
			// of bits functionality is not supported by OBS's shader parser. So we do it here instead.
			// See ffx_fsr1_mod.h for more information.
			vec4_set(&m_EASUConst0, reinterpret_float(con0[0]), reinterpret_float(con0[1]),
					reinterpret_float(con0[2]), reinterpret_float(con0[3]));
			vec4_set(&m_EASUConst1, reinterpret_float(con1[0]), reinterpret_float(con1[1]),
					reinterpret_float(con1[2]), reinterpret_float(con1[3]));
			vec4_set(&m_EASUConst2, reinterpret_float(con2[0]), reinterpret_float(con2[1]),
					reinterpret_float(con2[2]), reinterpret_float(con2[3]));
			vec4_set(&m_EASUConst3, reinterpret_float(con3[0]), reinterpret_float(con3[1]),
					reinterpret_float(con3[2]), reinterpret_float(con3[3]));
		}
	}

	//-------------------------------------------------------------------------------------

	void FSRFilter::render()
	{
		//TODO: allow direct rendering??
		if(!obs_source_process_filter_begin(m_Context, GS_RGBA, OBS_NO_DIRECT_RENDERING))
			return;

		// Update all shader parameters.
		// No need to check if they changed, OBS handles that.
		gs_effect_set_vec2(m_OutputSizeParam, &m_OutputSize);
		gs_effect_set_vec4(m_EASUConstParam0, &m_EASUConst0);
		gs_effect_set_vec4(m_EASUConstParam1, &m_EASUConst1);
		gs_effect_set_vec4(m_EASUConstParam2, &m_EASUConst2);
		gs_effect_set_vec4(m_EASUConstParam3, &m_EASUConst3);
		gs_effect_set_vec4(m_RCASConstParam0, &m_RCASConst0);

		obs_source_process_filter_end(m_Context, m_Shader, m_OutputSize.x, m_OutputSize.y);
	}

	//-------------------------------------------------------------------------------------

	uint32_t FSRFilter::width()
	{
		return m_OutputSize.x;
	}

	//-------------------------------------------------------------------------------------

	uint32_t FSRFilter::height()
	{
		return m_OutputSize.y;
	}


	//-------------------------------------------------------------------------------------

	bool FSRFilter::validate() const
	{
		// Ensure we have no nulls for key filter members
		return m_Context
				&& m_Shader
				&& m_OutputSizeParam
				&& m_EASUConstParam0
				&& m_EASUConstParam1
				&& m_EASUConstParam2
				&& m_EASUConstParam3
				&& m_RCASConstParam0;
	}

	//-------------------------------------------------------------------------------------
}
