#ifndef CORE_DEFINES_H_
#define CORE_DEFINES_H_

#include <math.h>
#include <limits.h>
#include <cfloat>

#define BUFFER_COLUMN_X 		0
#define BUFFER_COLUMN_Y 		1
#define BUFFER_COLUMN_Z 		2
#define BUFFER_COLUMN_DISTANCE 	3
#define BUFFER_COLUMN_ROLL 		4
#define BUFFER_COLUMN_PITCH 	5
#define BUFFER_COLUMN_YAW 		6
#define BUFFER_NUMBER_COLUMNS   7

#define ROBOCUP_SCANNER_MAX_ANGLE 1.41

#define ROBOCUP_MAX_SCAN_POINTS 2000

#ifndef FAIR_CONSOLE_OUTPUT
#define FAIR_CONSOLE_OUTPUT(s) printf("%s: %s", __PRETTY_FUNCTION__, s)
#endif

#ifndef MIN
/** Minimum of two values a and b */
#define MIN(a,b)	(((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
/** Maxmimum of two values a and b */
#define MAX(a,b)	(((a) > (b)) ? (a) : (b))
#endif

#ifndef M_PI
#define M_PI           3.14159265358979323846  /* pi */
#endif
#ifndef M_PI_2
#define M_PI_2         1.57079632679489661923  /* pi/2 */
#endif
#ifndef M_PI_4
#define M_PI_4         0.78539816339744830962  /* pi/4 */
#endif
#ifndef TWO_M_PI
#define TWO_M_PI 6.28318530717958647692
#endif
#ifndef M_PI_180
#define M_PI_180 0.01745329251994329576
#endif
#ifndef M_PI_360
#define M_PI_360 0.00872664625997164788
#endif
#ifndef M_PI_720
#define M_PI_720 0.00436332312998582388
#endif

#define ROBOCUP_SQRT_2 1.41421356

#endif /*CORE_DEFINES_H_*/
