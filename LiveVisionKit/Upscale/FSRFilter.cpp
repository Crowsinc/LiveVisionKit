#include "FSRFilter.hpp"



#include <filesystem>

namespace lvk
{
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
		m_OutputWidth = c_DefaultOutputWidth;
		m_OutputHeight = c_DefaultOutputHeight;

		// We set the input to {-1,-1} to force the first render
		// call to update the EASU constants for the shader.
		m_InputWidth = -1;
		m_InputHeight = -1;
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

		FsrRcasCon(m_RCASConst0, c_DefaultSharpness); // @suppress("Invalid arguments")
	}

	//-------------------------------------------------------------------------------------

	void FSRFilter::tick()
	{
		//TODO: handle downscaling

		auto filter_target = obs_filter_get_target(m_Context);
		const uint32_t input_width = obs_source_get_base_width(filter_target);
		const uint32_t input_height = obs_source_get_base_height(filter_target);

		// If the EASU constant is flagged as out-dated, or the input source size
		// has changed. Then we need to re-compute the EASU constants.
		if(m_EASUOutdated || input_width != m_InputWidth || input_height != m_InputHeight)
		{
			m_EASUOutdated = false;
			m_InputWidth = input_width;
			m_InputHeight = input_height;

			FsrEasuCon(m_EASUConst0, m_EASUConst1, m_EASUConst2, m_EASUConst3, // @suppress("Invalid arguments")
					m_InputWidth, m_InputHeight, m_InputWidth, m_InputHeight,
					m_OutputWidth, m_OutputHeight);
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
		gs_effect_set_val(m_EASUConstParam0, m_EASUConst0, sizeof(m_EASUConst0));
		gs_effect_set_val(m_EASUConstParam0, m_EASUConst1, sizeof(m_EASUConst1));
		gs_effect_set_val(m_EASUConstParam0, m_EASUConst2, sizeof(m_EASUConst2));
		gs_effect_set_val(m_EASUConstParam0, m_EASUConst3, sizeof(m_EASUConst3));
		gs_effect_set_val(m_EASUConstParam0, m_RCASConst0, sizeof(m_RCASConst0));

		obs_source_process_filter_end(m_Context, m_Shader, m_OutputWidth, m_OutputHeight);
	}

	//-------------------------------------------------------------------------------------

	uint32_t FSRFilter::width()
	{
		return m_OutputWidth;
	}

	//-------------------------------------------------------------------------------------

	uint32_t FSRFilter::height()
	{
		return m_OutputHeight;
	}


	//-------------------------------------------------------------------------------------

	bool FSRFilter::validate() const
	{
		// Ensure we have no nulls for key filter members
		return m_Context
				&& m_Shader
				&& m_EASUConstParam0
				&& m_EASUConstParam1
				&& m_EASUConstParam2
				&& m_EASUConstParam3
				&& m_RCASConstParam0;
	}

	//-------------------------------------------------------------------------------------
}
