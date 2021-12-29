#include "../Scaling/FSRFilter.hpp"

#include <obs/obs-module.h>
#include <filesystem>
#include <string>

namespace lvk
{

	//=====================================================================================
	//		CONSTANT PROPERTIES/SETTINGS
	//=====================================================================================

	static constexpr auto FILTER_NAME = "(LVK) FidelityFX Super Resolution 1.0";

	static constexpr auto PROP_SHARPNESS = "OUTPUT_SHARPNESS";

	static constexpr auto PROP_OUTPUT_SIZE   = "OUTPUT_SIZE";
	static constexpr auto OUTPUT_SIZE_CANVAS = "SIZE_CANVAS";
	static constexpr auto OUTPUT_SIZE_2160P  = "SIZE_2160P";
	static constexpr auto OUTPUT_SIZE_1440P  = "SIZE_1440P";
	static constexpr auto OUTPUT_SIZE_1080P  = "SIZE_1080P";
	static constexpr auto OUTPUT_SIZE_720P   = "SIZE_720P";
	static constexpr auto OUTPUT_SIZE_NONE   = "SIZE_NONE";

	static constexpr auto SHARPNESS_DEFAULT = 0.9f;
	static constexpr auto OUTPUT_SIZE_DEFAULT = OUTPUT_SIZE_CANVAS;

	//=====================================================================================
	//		MISC FUNCTIONS
	//=====================================================================================

	static inline AF1 interpret_float(AU1& val)
	{
		// Re-interprets the bits of a uint32_t as that of a float instead.
		// Don't use this unless you understand exactly what all the consequences are.
		return *reinterpret_cast<AF1*>(&val);
	}

	//=====================================================================================
	//		FILTER IMPLEMENTATION
	//=====================================================================================

	const char* FSRFilter::Name()
	{
		return FILTER_NAME;
	}

	//-------------------------------------------------------------------------------------

	obs_properties_t* FSRFilter::Properties()
	{
		obs_properties_t* properties = obs_properties_create();
		obs_property_t* property;

		// Create list for selecting output resolution
		property = obs_properties_add_list(
				properties,
				PROP_OUTPUT_SIZE,
				"Output Size",
				OBS_COMBO_TYPE_LIST,
				OBS_COMBO_FORMAT_STRING
		);

		obs_property_list_add_string(property, "Canvas Size", OUTPUT_SIZE_CANVAS);
		obs_property_list_add_string(property, "3840x2160   (2160p)", OUTPUT_SIZE_2160P);
		obs_property_list_add_string(property, "2560x1440   (1440p)", OUTPUT_SIZE_1440P);
		obs_property_list_add_string(property, "1920x1080   (1080p)", OUTPUT_SIZE_1080P);
		obs_property_list_add_string(property, "1280x720     (720p)", OUTPUT_SIZE_720P);
		obs_property_list_add_string(property, "Source Size  (No Scaling)", OUTPUT_SIZE_NONE);

		// Create slider for selecting sharpness
		// NOTE: the sharpness is presented as a value from 0-1 with 1 at max sharpness.
		// However, we internally interpret it as 0-2 with 0 being max sharpness.
		property = obs_properties_add_float_slider(
				properties,
				PROP_SHARPNESS,
				"Sharpness",
				0,
				1,
				0.05
		);

		return properties;
	}

	//-------------------------------------------------------------------------------------

	void FSRFilter::LoadDefaults(obs_data_t* settings)
	{
		obs_data_set_default_string(settings, PROP_OUTPUT_SIZE, OUTPUT_SIZE_DEFAULT);
		obs_data_set_default_double(settings, PROP_SHARPNESS, SHARPNESS_DEFAULT);
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
		  m_EASUOutdated(true),
		  m_BypassEASU(false),
		  m_BypassRCAS(false),
		  m_EASUMatchSource(false),
		  m_EASUMatchCanvas(false)
	{
		// Since we are using a C++ class, the FSRFilter should be allocated
		// using the 'new' keyword. Doing so will bypass OBS' memory leak
		// detection, which requires the use of bmalloc/bfree. In order to
		// keep consistency with the rest of OBS, we make a dummy allocation
		// to allow memory leak detection for this class.
		m_DummyAlloc = static_cast<uint32_t*>(bzalloc(sizeof(uint32_t)));

		// Load the FSR shader
		std::filesystem::path shader_path(obs_module_file("effects/fsr.effect"));

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


		// We set the output size to {-1,-1}. This is invalid,
		// but should be updated before the first render when
		// the filter properties settings are configured.
		vec2_set(&m_OutputSize, -1.0f, -1.0f);

		// We set the input size to {-1,-1} to force the first render
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
		// Reset any render pass bypassing
		m_BypassEASU = m_BypassRCAS = m_EASUMatchCanvas = m_EASUMatchSource = false;

		// Convert output size setting to numeric value
		const uint32_t old_output_width = m_OutputSize.x;
		const uint32_t old_output_height = m_OutputSize.y;
		std::string output_size = obs_data_get_string(settings, PROP_OUTPUT_SIZE);
		if(output_size == OUTPUT_SIZE_CANVAS)
			m_EASUMatchCanvas = true; // Match canvas size on tick()
		else if(output_size == OUTPUT_SIZE_NONE)
			m_EASUMatchSource = true; // Match source size on tick()
		else if(output_size == OUTPUT_SIZE_2160P)
			vec2_set(&m_OutputSize, 3840, 2160);
		else if(output_size == OUTPUT_SIZE_1440P)
			vec2_set(&m_OutputSize, 2560, 1440);
		else if(output_size == OUTPUT_SIZE_1080P)
			vec2_set(&m_OutputSize, 1920, 1080);
		else if(output_size == OUTPUT_SIZE_720P)
			vec2_set(&m_OutputSize, 1280, 720);

		// If the output size has changed, then we need to flag the EASU constants
		// as out-dated. So that they get forcefully updated next tick() call.
		m_EASUOutdated = old_output_width != static_cast<uint32_t>(m_OutputSize.x)
				|| old_output_height != static_cast<uint32_t>(m_OutputSize.y);

		// NOTE: the sharpness is presented as a value from 0-1 with 1 at max sharpness.
		// However, we internally interpret it as 0-2 with 0 being max sharpness.
		const float sharpness = 2.0 * (1.0 - obs_data_get_double(settings, PROP_SHARPNESS));

		// At a minimum sharpness of 2, we disable RCAS from running
		if(sharpness >= 2.0)
			m_BypassRCAS = true;
		else
		{
			// Otherwise, set up the RCAS parameters for this sharpness.

			varAU4(con0);
			FsrRcasCon(con0, sharpness);

			// Although con0 is defined as being a uint32_t vector for use with FSR, its bits
			// are ultimately re-interpreted as a float in the FSR shader. The re-interpretation of
			// bits functionality is not supported by OBS's shader parser. So we do it here instead.
			// See ffx_fsr1_mod.h for more information.
			vec4_set(&m_RCASConst0, interpret_float(con0[0]), interpret_float(con0[1]),
					interpret_float(con0[2]), interpret_float(con0[3]));
		}

	}

	//-------------------------------------------------------------------------------------

	void FSRFilter::tick()
	{
		// If EASU is set to match the canvas size, then update the output size to match
		if(m_EASUMatchCanvas)
		{
			obs_video_info video_info;
			obs_get_video_info(&video_info);

			// If the size is different, then we make sure the
			// out-dated flag is asserted so that the constants are updated
			m_EASUOutdated = m_EASUOutdated
					|| video_info.base_width != static_cast<uint32_t>(m_OutputSize.x)
					|| video_info.base_height != static_cast<uint32_t>(m_OutputSize.y);


			vec2_set(&m_OutputSize, video_info.base_width, video_info.base_height);
		}

		auto filter_target = obs_filter_get_target(m_Context);
		const uint32_t input_width = obs_source_get_base_width(filter_target);
		const uint32_t input_height = obs_source_get_base_height(filter_target);

		// If EASU is set to match the source size, then update the output size to match
		if(m_EASUMatchSource)
		{
			vec2_set(&m_OutputSize, input_width, input_height);

			// Although the source matching flag inherently causes EASU to be bypassed
			// we will still set the out-dated flag for the constants to be updated.
			m_EASUOutdated = true;
		}

		// If the input size matches the output size, then we will bypass EASU next render
		m_BypassEASU = m_BypassEASU
				|| m_EASUMatchSource
				|| (input_width == static_cast<uint32_t>(m_OutputSize.x)
				&& input_height == static_cast<uint32_t>(m_OutputSize.y));


		// If the EASU constants are flagged as out-dated, or the input size has changed
		// then we need to re-calculate the constants. Note that we still calculate them
		// even if we are bypassing EASU. This is to avoid issues with the constants
		// losing synchronization with the input/output sizes because we bypassed.
		if(m_EASUOutdated || input_width != m_InputSize.x || input_height != m_InputSize.y)
		{
			vec2_set(&m_InputSize, input_width, input_height);

			// NOTE: the constants should be defined by varAU4, but
			// we skip the macro and define it ourselves for brevity.
			AU1 con0[4], con1[4], con2[4], con3[4];
			FsrEasuCon(con0, con1, con2, con3,
					m_InputSize.x, m_InputSize.y, m_InputSize.x, m_InputSize.y,
					m_OutputSize.x, m_OutputSize.y);

			// Although con0-con3 are defined as being uint32_t vectors for use with FSR, their bits
			// are ultimately re-interpreted as a float in the FSR shader. The re-interpretation of
			// bits functionality is not supported by OBS's shader parser. So we do it here instead.
			// See ffx_fsr1_mod.h for more information.
			vec4_set(&m_EASUConst0, interpret_float(con0[0]), interpret_float(con0[1]),
					interpret_float(con0[2]), interpret_float(con0[3]));
			vec4_set(&m_EASUConst1, interpret_float(con1[0]), interpret_float(con1[1]),
					interpret_float(con1[2]), interpret_float(con1[3]));
			vec4_set(&m_EASUConst2, interpret_float(con2[0]), interpret_float(con2[1]),
					interpret_float(con2[2]), interpret_float(con2[3]));
			vec4_set(&m_EASUConst3, interpret_float(con3[0]), interpret_float(con3[1]),
					interpret_float(con3[2]), interpret_float(con3[3]));


			m_EASUOutdated = false;
		}
	}

	//-------------------------------------------------------------------------------------

	//TODO: fix RCAS issue with moving the source location
	// Probably to do with the render target being used and drawn to at the same time
	void FSRFilter::render() const
	{

		// AMD's FSR shader needs to be ran in two passes. The first performing edge adaptive
		// spatial upscaling (EASU), and the second performing robust contrast adaptive
		// sharpening (RCAS).
		//
		// To do this we first use OBS's provided source process filter rendering functionality,
		// allowing us to perform the EASU pass on the source video frame, automatically provided
		// in the 'image' parameter of the shader. This renders the up-scaled frame to the OBS bound
		// render target.
		//
		// The RCAS pass then needs to operate on the up-scaled frame. So we bind the render
		// target used for EASU to the shader 'image' parameter, and manually execute a rendering
		// pass of RCAS. Note that the original EASU render target remains as the target for RCAS
		// so we are technically reading and writing to it at the same time.
		//
		// TODO: Test ^ for issues. We can always add a third pass where we render the EASU render
		// target to a separate texture then render that texture back to the original render target.


		// EASU PASS
		// ================================================================================

		// Only perform the EASU pass if has not been flagged to be skipped
		if(!m_BypassEASU)
		{
			// NOTE: returns false if rendering of the filter should be bypassed.
			if(!obs_source_process_filter_begin(m_Context, GS_RGBA, OBS_NO_DIRECT_RENDERING))
				return;

			// Update all EASU shader parameters.
			// OBS requires us to perform this for every render
			gs_effect_set_vec2(m_OutputSizeParam, &m_OutputSize);
			gs_effect_set_vec4(m_EASUConstParam0, &m_EASUConst0);
			gs_effect_set_vec4(m_EASUConstParam1, &m_EASUConst1);
			gs_effect_set_vec4(m_EASUConstParam2, &m_EASUConst2);
			gs_effect_set_vec4(m_EASUConstParam3, &m_EASUConst3);

			obs_source_process_filter_tech_end(m_Context, m_Shader, m_OutputSize.x, m_OutputSize.y, "EASU");
		}

		// RCAS PASS
		// ================================================================================

		// Only perform the RCAS pass if it has not been flagged to be skipped
		if(!m_BypassRCAS)
		{
			// We need to switch how we handle the RCAS pass based on whether we performed EASU
			// or not. If we didn't then we render it normally as expected by OBS. Otherwise, we
			// need to perform the multi-pass rendering techniques discussed in previous comments.
			if(m_BypassEASU)
			{
				// NOTE: returns false if rendering of the filter should be bypassed.
				if(!obs_source_process_filter_begin(m_Context, GS_RGBA, OBS_NO_DIRECT_RENDERING))
					return;

				// Update all RCAS shader parameters
				// OBS requires us to perform this for every render
				gs_effect_set_vec2(m_OutputSizeParam, &m_OutputSize);
				gs_effect_set_vec4(m_RCASConstParam0, &m_RCASConst0);

				obs_source_process_filter_tech_end(m_Context, m_Shader, m_OutputSize.x, m_OutputSize.y, "RCAS");
			}
			else
			{
				auto easu_texture = gs_get_render_target();
				auto rcas_technique = gs_effect_get_technique(m_Shader, "RCAS");

				gs_technique_begin(rcas_technique);
				gs_technique_begin_pass(rcas_technique, 0);

				// Update all RCAS shader parameters
				// OBS requires us to perform this for every render
				gs_effect_set_vec2(m_OutputSizeParam, &m_OutputSize);
				gs_effect_set_vec4(m_RCASConstParam0, &m_RCASConst0);

				obs_source_draw(easu_texture, 0, 0, m_OutputSize.x, m_OutputSize.y, false);

				gs_technique_end_pass(rcas_technique);
				gs_technique_end(rcas_technique);
			}
		}

		// If both RCAS and EASU were bypassed, then completely skip the filter
		if(m_BypassRCAS && m_BypassEASU)
		{
			obs_source_skip_video_filter(m_Context);
		}
	}

	//-------------------------------------------------------------------------------------

	uint32_t FSRFilter::width() const
	{
		return m_OutputSize.x;
	}

	//-------------------------------------------------------------------------------------

	uint32_t FSRFilter::height() const
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
