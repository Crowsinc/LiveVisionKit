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

#ifndef DISABLE_RCAS
	static constexpr auto PROP_SHARPNESS = "OUTPUT_SHARPNESS";
	static constexpr auto SHARPNESS_DEFAULT = 0.9f;
#endif

	static constexpr auto PROP_OUTPUT_SIZE   = "OUTPUT_SIZE";
	static constexpr auto OUTPUT_SIZE_CANVAS = "CANVAS";
	static constexpr auto OUTPUT_SIZE_2160P  = "2160P";
	static constexpr auto OUTPUT_SIZE_1440P  = "1440P";
	static constexpr auto OUTPUT_SIZE_1080P  = "1080P";
	static constexpr auto OUTPUT_SIZE_720P   = "720P";
	static constexpr auto OUTPUT_SIZE_NONE   = "NONE";
	static constexpr auto OUTPUT_SIZE_DEFAULT = OUTPUT_SIZE_CANVAS;


	//===================================================================================
	//		FILTER IMPLEMENTATION
	//===================================================================================

	obs_properties_t* FSRFilter::Properties()
	{
		obs_properties_t* properties = obs_properties_create();
		obs_property_t* property;

		// Output resolution list
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

#ifndef DISABLE_RCAS
		// Slider for selecting sharpness (0.0 to 1.0)
		property = obs_properties_add_float_slider(
				properties,
				PROP_SHARPNESS,
				"Sharpness",
				0,
				1,
				0.05
		);
#endif
		return properties;
	}

	//-------------------------------------------------------------------------------------

	void FSRFilter::LoadDefaults(obs_data_t* settings)
	{
		obs_data_set_default_string(settings, PROP_OUTPUT_SIZE, OUTPUT_SIZE_DEFAULT);

#ifndef DISABLE_RCAS
		obs_data_set_default_double(settings, PROP_SHARPNESS, SHARPNESS_DEFAULT);
#endif
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
		  m_BypassEASU(false),
		  m_EASUMatchSource(false),
		  m_EASUMatchCanvas(false),
		  m_OutputSizeParam(nullptr),
		  m_EASUConstParam0(nullptr),
		  m_EASUConstParam1(nullptr),
		  m_EASUConstParam2(nullptr),
		  m_EASUConstParam3(nullptr)
	{
		obs_video_info video_info;
		obs_get_video_info(&video_info);
		const std::string graphics_api = video_info.graphics_module;

		// Due to inadequate HLSL to GLSL conversion by the OBS shader parser along with
		// being constrained to GLSL version 330, we must use a different FSR shader whenever
		// OBS is using the OpenGL API for rendering.
		const char* shader_path = nullptr;
		if(graphics_api.find("opengl") != std::string::npos)
			shader_path = obs_module_file("effects/fsr_glsl.effect");
		else
			shader_path = obs_module_file("effects/fsr.effect");

		// Load FSR shader
		if(shader_path != nullptr)
		{
			obs_enter_graphics();

			m_Shader = gs_effect_create_from_file(shader_path, NULL);

			if(m_Shader)
			{
				m_OutputSizeParam = gs_effect_get_param_by_name(m_Shader, "output_size");
				m_EASUConstParam0 = gs_effect_get_param_by_name(m_Shader, "easu_const_0");
				m_EASUConstParam1 = gs_effect_get_param_by_name(m_Shader, "easu_const_1");
				m_EASUConstParam2 = gs_effect_get_param_by_name(m_Shader, "easu_const_2");
				m_EASUConstParam3 = gs_effect_get_param_by_name(m_Shader, "easu_const_3");
#ifndef DISABLE_RCAS
				m_RCASConstParam0 = gs_effect_get_param_by_name(m_Shader, "rcas_const_0");
#endif
			}

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
		if(m_Shader != nullptr)
		{
			obs_enter_graphics();
			gs_effect_destroy(m_Shader);
#ifndef DISABLE_RCAS
			gs_texture_destroy(m_EASURenderTarget);
#endif
			obs_leave_graphics();
		}
	}

	//-------------------------------------------------------------------------------------

	void FSRFilter::configure(obs_data_t* settings)
	{
		m_BypassEASU = m_EASUMatchCanvas = m_EASUMatchSource = false;

		std::string output_size = obs_data_get_string(settings, PROP_OUTPUT_SIZE);
		if(output_size == OUTPUT_SIZE_CANVAS)
			m_EASUMatchCanvas = true; // Match canvas size on tick()
		else if(output_size == OUTPUT_SIZE_NONE)
			m_EASUMatchSource = true; // Match source size on tick()
		else if(output_size == OUTPUT_SIZE_2160P)
			vec2_set(&m_NewOutputSize, 3840, 2160);
		else if(output_size == OUTPUT_SIZE_1440P)
			vec2_set(&m_NewOutputSize, 2560, 1440);
		else if(output_size == OUTPUT_SIZE_1080P)
			vec2_set(&m_NewOutputSize, 1920, 1080);
		else if(output_size == OUTPUT_SIZE_720P)
			vec2_set(&m_NewOutputSize, 1280, 720);

#ifndef DISABLE_RCAS
		// NOTE: the sharpness is presented as a value from 0-1 with 1 at max sharpness.
		// However, we internally interpret it as 0-2 with 0 being max sharpness.
		const float sharpness = 2.0 * (1.0 - obs_data_get_double(settings, PROP_SHARPNESS));

		// The RCAS constant is a vector of four uint32_t but its bits actually represent floats.
		// Normally this conversion happens in the FSR shader. However due to compatibility issues,
		// we perform the conversion on the CPU instead. So here we pass in float pointers, casted
		// to uint32_t pointers to facilitate the uint32_t to float re-interpretation.
		FsrRcasCon((AU1*)m_RCASConst0.ptr, sharpness);
#endif
	}

	//-------------------------------------------------------------------------------------

#ifndef DISABLE_RCAS
	void FSRFilter::prepare_easu_render_target()
	{
		obs_enter_graphics();

		// NOTE: we are relying on short-circuit logic here in case m_EASURenderTarget is nullptr
		const bool outdated = m_EASURenderTarget == nullptr
				|| gs_texture_get_width(m_EASURenderTarget) != static_cast<uint32_t>(m_OutputSize.x)
				|| gs_texture_get_height(m_EASURenderTarget) != static_cast<uint32_t>(m_OutputSize.y);

		if(outdated)
		{
			if(m_EASURenderTarget != nullptr)
				gs_texture_destroy(m_EASURenderTarget);

			m_EASURenderTarget = gs_texture_create(m_OutputSize.x, m_OutputSize.y,  GS_RGBA, 1, NULL, GS_RENDER_TARGET);
		}

		obs_leave_graphics();
	}
#endif


	//-------------------------------------------------------------------------------------

	void FSRFilter::update()
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
		else if(m_EASUMatchSource)
		{
			vec2_set(&m_NewOutputSize, input_width, input_height);
		}

		const bool output_size_changed = m_OutputSize.x != m_NewOutputSize.x
									  || m_OutputSize.y != m_NewOutputSize.y;

		m_OutputSize = m_NewOutputSize;

#ifndef DISABLE_RCAS
		// Make sure the EASU render target matches the output size
		prepare_easu_render_target();
#endif

		// Update EASU constants, we do this regardless of if EASU is bypassed or not
		if(output_size_changed || input_width != m_InputSize.x || input_height != m_InputSize.y)
		{
			vec2_set(&m_InputSize, input_width, input_height);

			// The EASU constants are a vector of four uint32_t but their bits actually represent floats.
			// Normally this conversion happens in the FSR shader. However due to compatibility issues,
			// we perform the conversion on the CPU instead. So here we pass in float pointers, casted
			// to uint32_t pointers to facilitate the uint32_t to float re-interpretation.
			FsrEasuCon((AU1*)m_EASUConst0.ptr, (AU1*)m_EASUConst1.ptr, (AU1*)m_EASUConst2.ptr, (AU1*)m_EASUConst3.ptr,
					m_InputSize.x, m_InputSize.y, m_InputSize.x, m_InputSize.y,
					m_OutputSize.x, m_OutputSize.y);
		}

		// Don't perform EASU If the input size matches the output size
		m_BypassEASU = m_EASUMatchSource
				|| (input_width == static_cast<uint32_t>(m_OutputSize.x)
				&& input_height == static_cast<uint32_t>(m_OutputSize.y));
	}

	//-------------------------------------------------------------------------------------

	void FSRFilter::render() const
	{
		if(m_BypassEASU)
			obs_source_skip_video_filter(m_Context);

		// AMD's FSR shader needs to be ran in two passes. The first performing edge adaptive
		// spatial up-scaling (EASU), and the second performing robust contrast adaptive
		// sharpening (RCAS).
		//
		// To do this we first use OBS's provided source process filter rendering functionality,
		// allowing us to perform the EASU pass on the source video frame, automatically provided
		// through the 'image' parameter of the shader. However, we remember the old render target
		// and instead render to our own EASU render texture.
		//
		// The RCAS pass then needs to operate on the up-scaled frame. So we bind the EASU render
		// texture to the shader 'image' parameter, and manually execute a rendering pass of RCAS.
		// The new render target should be the one which was originally bound for EASU.


		// EASU PASS
		// ================================================================================

		if(!obs_source_process_filter_begin(m_Context, GS_RGBA, OBS_NO_DIRECT_RENDERING))
			return;

		gs_effect_set_vec2(m_OutputSizeParam, &m_OutputSize);
		gs_effect_set_vec4(m_EASUConstParam0, &m_EASUConst0);
		gs_effect_set_vec4(m_EASUConstParam1, &m_EASUConst1);
		gs_effect_set_vec4(m_EASUConstParam2, &m_EASUConst2);
		gs_effect_set_vec4(m_EASUConstParam3, &m_EASUConst3);

#ifndef DISABLE_RCAS
		// Only change render target if we are doing RCAS
		gs_texture_t* original_target = gs_get_render_target();
		gs_zstencil_t* original_zstencil = gs_get_zstencil_target();
		gs_set_render_target(m_EASURenderTarget, NULL);
#endif

		obs_source_process_filter_tech_end(m_Context, m_Shader, m_OutputSize.x, m_OutputSize.y, "EASU");


#ifndef DISABLE_RCAS
		// RCAS PASS
		// ================================================================================

		// Branch between single pass or multi-pass RCAS render
		auto rcas_technique = gs_effect_get_technique(m_Shader, "RCAS");

		gs_set_render_target(original_target, original_zstencil);

		gs_technique_begin(rcas_technique);
		gs_technique_begin_pass(rcas_technique, 0);

		gs_effect_set_vec2(m_OutputSizeParam, &m_OutputSize);
		gs_effect_set_vec4(m_RCASConstParam0, &m_RCASConst0);

		obs_source_draw(m_EASURenderTarget, 0, 0, m_OutputSize.x, m_OutputSize.y, false);

		gs_technique_end_pass(rcas_technique);
		gs_technique_end(rcas_technique);
#endif

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
				&& m_OutputSizeParam != nullptr
#ifndef DISABLE_RCAS
				&& m_RCASConstParam0 != nullptr
#endif
				&& m_EASUConstParam0 != nullptr
				&& m_EASUConstParam1 != nullptr
				&& m_EASUConstParam2 != nullptr
				&& m_EASUConstParam3 != nullptr;
	}

	//-------------------------------------------------------------------------------------
}
