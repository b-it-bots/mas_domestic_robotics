/*
 *
 * libFAIR, Fraunhofer Autonomous Intelligent Robotics Library
 *
 * Copyright Fraunhofer Gesellschaft e. V., Munich, Germany
 *
 * The FAIR library [both binary and source code (if released)] is intellectual
 * property owned by Fraunhofer Gesellschaft and is protected by copyright;
 * the ownership remains with Fraunhofer Gesellschaft.
 *
 */

/**
 * Common inclusion header for all fair modules
 * This header file defines basic routines for Windows and Linux
 * @author Stefan May
 */

// COMMON_H is a common name and oftenly used, e.g. by ffmpeg
// so avoid using that define!
#ifndef FAIR_COMMON_H_
#define FAIR_COMMON_H_

#include <stdio.h>
#include <sys/types.h>

#include <stdlib.h>
#include <assert.h>

/**
 * @brief system independend (windows/linux) sleep macro with ms granularity
 */
#ifdef WIN32
//#	include <windows.h>
#	define FAIR_SLEEP(x) Sleep(x);
#	define snprintf _snprintf
#else
#ifndef FAIR_SLEEP
#	include <unistd.h>
#	define FAIR_SLEEP(x) usleep((x)*1000);
#endif
#endif

#ifndef M_PI
#	define M_PI        3.14159265358979323846
#endif

/**
 * @brief helper macro to support extern c declaration for c++ usage
 */
#ifdef __cplusplus
#	define BEGIN_C_DECLS	extern "C" {
#	define END_C_DECLS		}
#else
#	define BEGIN_C_DECLS
#	define END_C_DECLS
#endif

#ifndef FAIR_EXIT_SUCCESS
/**
 * @brief common success return flag 
 */
#	define FAIR_EXIT_SUCCESS 1
/**
 * @brief common failure return flag
 */
#	define FAIR_EXIT_FAILURE 0
#endif

#ifndef FAIR_INVALID_HANDLE
#	define FAIR_INVALID_HANDLE NULL
#endif

/**
 * @brief assertion macro for debugging purposes. Only active with DEBUG preprocessor flag.
 */
#if defined( DEBUG )
#	define fairAssert(a,b) assert( a && b && __LINE__ && __FILE__)
#else
#	define fairAssert(a,b)
#endif /*_DEBUG*/

#endif /* !FAIR_COMMON_H_ */
