
#include <vector>
#include <algorithm>

#include <obs/obs.h>

#include "../Diagnostics/Assert.hpp"

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
		LVK_ASSERT_FMT(data.size() == keep.size(),
				"Inputs are different sizes, data.size()=%d, keep.size()=%d.", data.size(), keep.size());

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
		LVK_ASSERT_FMT(data_1.size() == keep.size(),
				"Inputs are different sizes, data_1.size()=%d, keep.size()=%d.", data_1.size(), keep.size());
		LVK_ASSERT_FMT(data_2.size() == keep.size(),
				"Inputs are different sizes, data_2.size()=%d, keep.size()=%d.", data_2.size(), keep.size());


		// Generic filter using fast erase for the removal of data
		// NOTE: Changes ordering of the vector data so should be used with caution.


		// We need to filter in reverse so that the fast erase doesn't affect the
		// vector element correspondence of unprocessed elements.
		for(int k = keep.size() - 1; k >= 0; k--)
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
		LVK_ASSERT_FMT(data.size() == keep.size(),
				"Inputs are different sizes, data.size()=%d, keep.size()=%d.", data.size(), keep.size());

		// https://stackoverflow.com/a/33494518
		const auto predicate = [&data, &keep](const T& value) {
			return !keep.at(&value - &data[0]);
		};

		data.erase(std::remove_if(data.begin(), data.end(), predicate), data.end());
	}

	//-------------------------------------------------------------------------------------

}
