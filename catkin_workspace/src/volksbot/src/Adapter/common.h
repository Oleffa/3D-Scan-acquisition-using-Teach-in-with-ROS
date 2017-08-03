/**
 * Common inclusion header for all fair modules
 * This header file defines basic routines for Windows and Linux
 * @author Stefan May
 */

#ifndef COMMON_H
#define COMMON_H

#include <stdio.h>
#include <sys/types.h>

#include <stdlib.h>
#include <assert.h>

#ifdef WIN32
#	include <windows.h>
#	define SLEEP(x) Sleep(x);
#	define snprintf _snprintf
#	ifndef M_PI
#		define M_PI        3.14159265358979323846
#	endif
#else
#	include <unistd.h>
#	define SLEEP(x) usleep(x*1000);
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

#ifndef EXIT_SUCCESS
/**
 * @brief common success return flag 
 */
#	define EXIT_SUCCESS 0
/**
 * @brief common failure return flag
 */
#	define EXIT_FAILURE 1
#endif

/**
 * @brief assertion macro for debugging purposes. Only active with DEBUG preprocessor flag.
 */
#if defined( DEBUG )
#	define Assert(a,b) assert( a && b && __LINE__ && __FILE__)
#else
#	define Assert(a,b)
#endif /*_DEBUG*/

/**
 * @brief system independend (windows/linux) sleep macro with ms granularity
 */
#ifdef WIN32
#	define SLEEP(x) Sleep(x);
#else
#	define SLEEP(x) usleep(x*1000);
#endif

#endif /* !COMMON_H */
