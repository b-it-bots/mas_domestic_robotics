#include "timer.h"
#include <iostream>
using namespace std;
using namespace fair;

/**
 * windows specific time measurement info
 */
#ifdef WIN32
static LONG64 g_ldTicksPerSecond = 0;
#endif

CTimer::CTimer()
{
#ifdef WIN32
	if(!g_ldTicksPerSecond) QueryPerformanceFrequency((LARGE_INTEGER*)&g_ldTicksPerSecond);
#endif
	_ldStartTime = getCurrentTime();
}

CTimer::~CTimer()
{
}

long double CTimer::reset()
{
	_ldCurrentTime = getCurrentTime();
	long double ldTimeDiff = _ldCurrentTime - _ldStartTime;
	_ldStartTime = _ldCurrentTime;
	return ldTimeDiff;
}

long double CTimer::getTime()
{
	_ldCurrentTime = getCurrentTime();
	return (_ldCurrentTime - _ldStartTime);
}

long double CTimer::getCurrentTime()
{
#ifdef WIN32
	LARGE_INTEGER ldTick;
	QueryPerformanceCounter(&ldTick);
	ldTick.QuadPart = ldTick.QuadPart * 1000 / g_ldTicksPerSecond;
	return ldTick.u.LowPart;
#else
	static struct timeval tv;
	gettimeofday(&tv, NULL);
	return (long double) (tv.tv_sec * 1000.0 + tv.tv_usec / 1000.0);
#endif
}
