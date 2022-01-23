
#include <string>
#include <obs/obs.h>

namespace lvk
{

	//-------------------------------------------------------------------------------------

	template<typename... T>
	void _assert(const bool condition, const char* file, const char* func, const char* format, T... vars)
	{
		if(!condition)
		{
			const auto message = std::string("LVK ASSERTION FROM ") + file + "@" + func + ": " + format;
			bcrash(message.c_str(), vars...);
		}
	}

	//-------------------------------------------------------------------------------------

}
