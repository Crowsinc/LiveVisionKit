#pragma once


//#define DISABLE_ASSERTS

#ifndef DISABLE_ASSERTS

#define LVK_ASSERT_FMT(assertion, format, ...) lvk::_assert(assertion, __FILE__, __func__, format, __VA_ARGS__)

#define LVK_ASSERT(assertion, format) lvk::_assert(assertion, __FILE__, __func__, format)

#else

#define LVK_ASSERT_FMT(assertion, format, ...)

#define LVK_ASSERT(assertion, format)

#endif


namespace lvk
{

	template<typename... T>
	void _assert(const bool condition, const char* file, const char* func, const char* format, T... vars);

}

#include "Assert.tpp"
