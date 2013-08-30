#ifndef CTIMER_H__
#define CTIMER_H__

#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#include <time.h>
#include <stdio.h>

/**
 * @namespace fair
 */
namespace fair
{

	/**
	 * @class CTimer
	 * @brief This utility provides functions for precise timing measurements.
	 * @author Stefan May, Christopher Loerken and Dirk Holz
	 */
	class CTimer
	{
 	public:
		/**
		 * Default constructor. Starts directly time measurement after instanciation.
		 */
		CTimer();

		/**
		 *  Default destructor
		 */
		~CTimer();

		/**
		 * Function resets the timer. '_ldStartTime' is set to the current system time.
		 * @return elapsed time in ms.µs since construction of timer or last reset call
		 */
		long double reset();

		/**
		 * Retrieve the elapsed time (in ms.µs) since
		 * @return elapsed time in ms.µs since construction of timer or last reset call
		 */
		long double getTime();

 	private:

		/**
		 * Holds the system time at construction of timer of last reset()-function call
		 */
		long double _ldStartTime;

		/**
		 *  Holds current system time
		 */
		long double _ldCurrentTime;

		/**
		 * Function retrieves the current time from operating system
		 * using system specific standard functions. The Current Time
		 * in ms.µs is stored in _ldCurrentTime.
		 * @return elapsed time in ms.µs since construction of timer or last reset call
		 */
		long double getCurrentTime();

		/**
		 * Windows specific time measurement information
		 */
#ifdef WIN32
		LARGE_INTEGER _ldTicksPerSecond;
#endif
	};

} /*namespace*/

#endif /*CTIMER_H__*/
