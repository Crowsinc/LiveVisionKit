
#include <vector>
#include <algorithm>

namespace lvk
{

	//-------------------------------------------------------------------------------------

	template<typename T>
	void fast_erase(std::vector<T>& data, const size_t index)
	{
		// Removes an element by first swapping it with the final element to avoid re-shuffling.
		// NOTE: Changes ordering of the vector data so should be used with caution.
		std::swap(data[index], data.back());
		data.pop_back();
	}

	//-------------------------------------------------------------------------------------

	template<typename T, typename P>
	void fast_filter(std::vector<T>& data, const std::vector<P>& keep)
	{
		// Generic filter using fast erase for the removal of data
		// NOTE: Changes ordering of the vector data so should be used with caution.

		// We need to filter in reverse so that the fast erase doesn't affect the
		// vector element correspondence of unprocessed elements.
		for(auto k = keep.size() - 1; k >= 0; k--)
			if(!keep[k])
				fast_erase(data, k);
	}

	//-------------------------------------------------------------------------------------

	template<typename T, typename P>
	void fast_filter(std::vector<T>& data_1, std::vector<T>& data_2, const std::vector<P>& keep)
	{
		// Generic filter using fast erase for the removal of data
		// NOTE: Changes ordering of the vector data so should be used with caution.


		// We need to filter in reverse so that the fast erase doesn't affect the
		// vector element correspondence of unprocessed elements.
		for(auto k = keep.size() - 1; k >= 0; k--)
			if(!keep[k])
			{
				fast_erase(data_1, k);
				fast_erase(data_2, k);
			}
	}

	//-------------------------------------------------------------------------------------

	template<typename T, typename P>
	void filter(std::vector<T>& data, const std::vector<P>& keep)
	{
		// https://stackoverflow.com/a/33494518
		const auto predicate = [&data, &keep](const T& value) {
			return !keep.at(&value - &data[0]);
		};

		data.erase(std::remove_if(data.begin(), data.end(), predicate), data.end());
	}

	//-------------------------------------------------------------------------------------

	template<typename T>
	int signum(const T value)
	{
		// https://stackoverflow.com/a/4609795
		return (T(0) < value) - (value < T(0));
	}

	//-------------------------------------------------------------------------------------

}
