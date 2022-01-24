#pragma once

#include <obs/obs.h>

//#define DISABLE_ASSERTS

#ifndef DISABLE_ASSERTS

#define LVK_ASSERT(assertion) if(!(assertion)){bcrash("\e[1;31m" "LVK@" __FILE__ "@%s(..) FAILED " #assertion "\n \e[0m" , __func__);}

#else

#define LVK_ASSERT(assertion)

#endif


