#include "../Scaling/FSRFilter.hpp"

#include <obs/obs-module.h>
#include <filesystem>
#include <string>

#define A_CPU 1
#include "../Effects/ffx_a.h"
#include "../Effects/ffx_fsr1.h"


namespace lvk
{

	//===================================================================================
	//		CONSTANT PROPERTIES/SETTINGS
	//===================================================================================

	static constexpr auto PROP_SHARPNESS = "OUTPUT_SHARPNESS";
	static constexpr auto SHARPNESS_DEFAULT = 0.9f;

	static constexpr auto PROP_OUTPUT_SIZE   = "OUTPUT_SIZE";
	static constexpr auto OUTPUT_SIZE_CANVAS = "CANVAS";
	static constexpr auto OUTPUT_SIZE_2160P  = "2160P";
	static constexpr auto OUTPUT_SIZE_1440P  = "1440P";
	static constexpr auto OUTPUT_SIZE_1080P  = "1080P";
	static constexpr auto OUTPUT_SIZE_720P   = "720P";
	static constexpr auto OUTPUT_SIZE_DEFAULT = OUTPUT_SIZE_CANVAS;


	//===================================================================================
	//		FILTER IMPLEMENTATION
	//===================================================================================

	obs_properties_t* FSRFilter::Properties()
	{
		obs_properties_t* properties = obs_properties_create();

		// Output resolution list
		auto property = obs_properties_add_list(
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

		// Slider for selecting sharpness (0.0 to 1.0)
		obs_properties_add_float_slider(
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
		  m_EASUMatchCanvas(false),
		  m_OutputSizeParam(nullptr),
		  m_EASUConstParam0(nullptr),
		  m_EASUConstParam1(nullptr),
		  m_EASUConstParam2(nullptr),
		  m_EASUConstParam3(nullptr),
		  m_RCASConstParam0(nullptr)

	{
		obs_video_info video_info;
		obs_get_video_info(&video_info);
		const std::string graphics_api = video_info.graphics_module;

		// Due to inadequate HLSL to GLSL conversion by the OBS shader parser along with
		// being constrained to GLSL version 330, we must use a different FSR shader whenever
		// OBS is using the OpenGL API for rendering.
		char* shader_path = nullptr;
		if(graphics_api.find("opengl") != std::string::npos)
			shader_path = obs_module_file("effects/fsr_glsl.effect");
		else
			shader_path = obs_module_file("effects/fsr.effect");

		// Load FSR shader
		if(shader_path != nullptr)
		{
			obs_enter_graphics();

			m_Shader = gs_effect_create_from_file(shader_path, nullptr);
			bfree(shader_path);

			if(m_Shader)
			{
				m_OutputSizeParam = gs_effect_get_param_by_name(m_Shader, "output_size");
				m_EASUConstParam0 = gs_effect_get_param_by_name(m_Shader, "easu_const_0");
				m_EASUConstParam1 = gs_effect_get_param_by_name(m_Shader, "easu_const_1");
				m_EASUConstParam2 = gs_effect_get_param_by_name(m_Shader, "easu_const_2");
				m_EASUConstParam3 = gs_effect_get_param_by_name(m_Shader, "easu_const_3");
				m_RCASConstParam0 = gs_effect_get_param_by_name(m_Shader, "rcas_const_0");
			}

			m_EASURenderTarget = gs_texrender_create(GS_RGBA, GS_ZS_NONE);

			obs_leave_graphics();
		}

		// These should get updated to their proper values before the first render.
		vec2_zero(&m_NewOutputSize);
		vec2_zero(&m_OutputSize);
		vec2_zero(&m_InputSize);
	}

	//-------------------------------------------------------------------------------------

	FSRFilter::~FSRFilter()
	{
		obs_enter_graphics();

		if(m_Shader != nullptr)
			gs_effect_destroy(m_Shader);

		if(m_EASURenderTarget != nullptr)
			gs_texrender_destroy(m_EASURenderTarget);

		obs_leave_graphics();
	}

	//-------------------------------------------------------------------------------------

	void FSRFilter::configure(obs_data_t* settings)
	{
		m_EASUMatchCanvas = false;

		std::string output_size = obs_data_get_string(settings, PROP_OUTPUT_SIZE);
		if(output_size == OUTPUT_SIZE_CANVAS)
			m_EASUMatchCanvas = true; // Match canvas size on tick()
		else if(output_size == OUTPUT_SIZE_2160P)
			vec2_set(&m_NewOutputSize, 3840, 2160);
		else if(output_size == OUTPUT_SIZE_1440P)
			vec2_set(&m_NewOutputSize, 2560, 1440);
		else if(output_size == OUTPUT_SIZE_1080P)
			vec2_set(&m_NewOutputSize, 1920, 1080);
		else if(output_size == OUTPUT_SIZE_720P)
			vec2_set(&m_NewOutputSize, 1280, 720);

		// NOTE: the sharpness is presented as a value from 0-1 with 1 at max sharpness.
		// However, we internally interpret it as 0-2 with 0 being max sharpness.
		const float sharpness = 2.0 * (1.0 - obs_data_get_double(settings, PROP_SHARPNESS));

		// The RCAS constant is a vector of four uint32_t but its bits actually represent floats.
		// Normally this conversion happens in the FSR shader. However due to compatibility issues,
		// we perform the conversion on the CPU instead. So here we pass in float pointers, casted
		// to uint32_t pointers to facilitate the uint32_t to float re-interpretation.
		FsrRcasCon((AU1*)m_RCASConst0.ptr, sharpness);
	}

	//-------------------------------------------------------------------------------------

	void FSRFilter::tick()
	{
		auto filter_target = obs_filter_get_target(m_Context);
		const uint32_t input_width = obs_source_get_base_width(filter_target);
		const uint32_t input_height = obs_source_get_base_height(filter_target);

		// Update EASU output size if necessary
		if(m_EASUMatchCanvas)
		{
			obs_video_info video_info;
			obs_get_video_info(&video_info);

			vec2_set(&m_NewOutputSize, video_info.base_width, video_info.base_height);
		}

		const bool output_size_changed = m_OutputSize.x != m_NewOutputSize.x
									  || m_OutputSize.y != m_NewOutputSize.y;

		// Update EASU constants, we do this regardless of if EASU is bypassed or not
		if(output_size_changed || input_width != m_InputSize.x || input_height != m_InputSize.y)
		{
			vec2_set(&m_InputSize, input_width, input_height);
			m_OutputSize = m_NewOutputSize;

			// The EASU constants are a vector of four uint32_t but their bits actually represent floats.
			// Normally this conversion happens in the FSR shader. However due to compatibility issues,
			// we perform the conversion on the CPU instead. So here we pass in float pointers, casted
			// to uint32_t pointers to facilitate the uint32_t to float re-interpretation.
			FsrEasuCon((AU1*)m_EASUConst0.ptr, (AU1*)m_EASUConst1.ptr, (AU1*)m_EASUConst2.ptr, (AU1*)m_EASUConst3.ptr,
					m_InputSize.x, m_InputSize.y, m_InputSize.x, m_InputSize.y,
					m_OutputSize.x, m_OutputSize.y);
		}
	}

	//-------------------------------------------------------------------------------------

	void FSRFilter::render() const
	{
		gs_texrender_reset(m_EASURenderTarget);
		if(gs_texrender_begin(m_EASURenderTarget, m_OutputSize.x, m_OutputSize.y))
		{
			if(obs_source_process_filter_begin(m_Context, GS_RGBA, OBS_NO_DIRECT_RENDERING))
			{
				gs_effect_set_vec2(m_OutputSizeParam, &m_OutputSize);
				gs_effect_set_vec4(m_EASUConstParam0, &m_EASUConst0);
				gs_effect_set_vec4(m_EASUConstParam1, &m_EASUConst1);
				gs_effect_set_vec4(m_EASUConstParam2, &m_EASUConst2);
				gs_effect_set_vec4(m_EASUConstParam3, &m_EASUConst3);

				obs_source_process_filter_tech_end(m_Context, m_Shader, m_OutputSize.x, m_OutputSize.y, "EASU");
			}
			else obs_source_skip_video_filter(m_Context);


			gs_texrender_end(m_EASURenderTarget);


			auto easu_render_texture = gs_texrender_get_texture(m_EASURenderTarget);
			while(gs_effect_loop(m_Shader,"RCAS"))
			{
				gs_effect_set_vec2(m_OutputSizeParam, &m_OutputSize);
				gs_effect_set_vec4(m_RCASConstParam0, &m_RCASConst0);

				obs_source_draw(easu_render_texture, 0, 0, m_OutputSize.x, m_OutputSize.y, false);
			}

		}
		else obs_source_skip_video_filter(m_Context);
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
		return m_Context != nullptr
				&& m_Shader  != nullptr
				&& m_EASURenderTarget != nullptr
				&& m_OutputSizeParam != nullptr
				&& m_RCASConstParam0 != nullptr
				&& m_EASUConstParam0 != nullptr
				&& m_EASUConstParam1 != nullptr
				&& m_EASUConstParam2 != nullptr
				&& m_EASUConstParam3 != nullptr;
	}

	//-------------------------------------------------------------------------------------
}
