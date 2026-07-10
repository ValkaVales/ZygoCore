#pragma once


#if defined(_MSC_VER)
#define ZgDebugBreak() __debugbreak()
#else
#include <csignal>
#define ZgDebugBreak() std::raise( SIGTRAP )
#endif


#ifdef _DEBUG
#define ZgAssert( expr ) do { if ( !(expr) ) ZgDebugBreak(); } while (false)
#else
#define ZgAssert( expr )
#endif

#define ZgAssertRelease( expr ) do { if ( !(expr) ) ZgDebugBreak(); } while (false)

