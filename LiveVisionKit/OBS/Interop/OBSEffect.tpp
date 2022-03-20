//    *************************** LiveVisionKit ****************************
//    Copyright (C) 2022  Sebastian Di Marco (crowsinc.dev@gmail.com)
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.
// 	  **********************************************************************

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	template<typename E>
	E& OBSEffect<E>::Get()
	{
		thread_local E effect;
		return effect;
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename E>
	bool OBSEffect<E>::Validate()
	{
		auto& effect = OBSEffect<E>::Get();

		return effect.handle() != nullptr && effect.validate();
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename E>
	OBSEffect<E>::OBSEffect(const std::string& name)
		: m_Handle(nullptr)
	{
		char* effect_path = obs_module_file(
			("effects/" + name + ".effect").c_str()
		);

		LVK_ASSERT(effect_path != nullptr);

		if(effect_path != nullptr)
		{
			obs_enter_graphics();
			m_Handle = gs_effect_create_from_file(effect_path, nullptr);
			bfree(effect_path);

			obs_leave_graphics();
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename E>
	OBSEffect<E>::~OBSEffect()
	{
		LVK_ASSERT(m_Handle != nullptr);

		obs_enter_graphics();
		gs_effect_destroy(m_Handle);
		obs_leave_graphics();
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename E>
	gs_effect_t* OBSEffect<E>::handle()
	{
		return m_Handle;
	}

//---------------------------------------------------------------------------------------------------------------------

}
