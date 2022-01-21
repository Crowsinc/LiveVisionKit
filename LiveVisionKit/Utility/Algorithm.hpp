#include <vector>

namespace lvk
{

	template<typename T>
	void fast_erase(std::vector<T>& data, const size_t index);

	template<typename T, typename P>
	void fast_filter(std::vector<T>& data, const std::vector<P>& keep);

	template<typename T, typename P>
	void fast_filter(std::vector<T>& data_1, std::vector<T>& data_2, const std::vector<P>& keep);

	template<typename T, typename P>
	void filter(std::vector<T>& data, const std::vector<P>& keep);

	template<typename T>
	int signum(const T value);

}

#include "Algorithm.tpp"
