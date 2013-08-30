/**
 * Mathbase defines basic math routines with no dependencies to other modules.
 * This header file collects methods for conviency. Some methods are adopted from other libraries (see comments below).
 * @author Stefan May
 * @date 16.01.2007
 */

#ifndef FAIRMATHBASE_H
#define FAIRMATHBASE_H

#include <math.h>
#include <algorithm>
#include "common.h"

#ifdef WIN32
#	ifndef M_PI
#		define M_PI        3.14159265358979323846
#	endif
#	ifdef max
#		undef max
#	endif
#	ifdef min
#		undef min
#	endif
#	define NOMINMAX
#endif

#define fairROUND(f)			( (f) < 0 ? (int)((f) - 0.5f) : (int)((f) + 0.5f) );
#define fairFIX(x,n) 	      	(int)((x)*(1 << (n)) + 0.5)
#define fairDESCALE(x,n) 		( ( (x) + ( 1 << ((n) - 1) ) ) >> (n))

#define fairSCALE_F2I_8BIT(x) 		fairFIX(x, 8)
#define fairSCALE_F2I_16BIT(x) 		fairFIX(x,16)
#define fairSCALE_F2I_24BIT(x) 		fairFIX(x,24)
#define fairSCALE_F2I_32BIT(x) 		fairFIX(x,32)

#define fairSCALE_I_8BIT(x) 		( x <<  8 )
#define fairSCALE_I_16BIT(x) 		( x << 16 )
#define fairSCALE_I_24BIT(x) 		( x << 24 )
#define fairSCALE_I_32BIT(x) 		( x << 32 )

#define fairDESCALE_8BIT(x)		fairDESCALE(x,8)
#define fairDESCALE_16BIT(x) 	fairDESCALE(x,16)
#define fairDESCALE_24BIT(x) 	fairDESCALE(x,24)

#define fairCAST_8U(t)  (unsigned char)(!((t) & ~255) ? (t) : (t) > 0 ? 255 : 0)
#define fairCAST_16U(t) (unsigned short)(!((t) & ~65535) ? (t) : (t) > 0 ? 65535 : 0)

using std::swap;

/**
 * euclidean distance between two points in 2D space
 **/
#define DIST_2D_SQR(dP1,dP2) sqr(dP1[0] - dP2[0]) + sqr(dP1[1] - dP2[1])

/**
 * euclidean distance between two points in 3D space
 **/
#define DIST_3D_SQR(dP1,dP2) sqr(dP1[0] - dP2[0]) + sqr(dP1[1] - dP2[1]) + sqr(dP1[2] - dP2[2])

namespace fair
{

	/**
	 * @function max
	 * @brief maximum template function
	 * @return maximum of a or b
	 **/
	template<class T>
	static inline T max(const T a, const T b)
	{
		return ((a >= b) ? a : b);
	}

	/**
	 * @function min
	 * @brief minimumn template function
	 * @return minimum of a or b
	 **/
	template<class T>
	static inline T min(const T a, const T b)
	{
		return ((a <= b) ? a : b);
	}

	/**
	 * @function sqr
	 * @brief sqare template function
	 * @return square of argument x
	 **/
	template<class T>
	static inline T sqr(const T &x)
	{
		return x * x;
	}

	/**
	 * @function Dist2_2D
	 * @return Squared distance between two points in 2D space
	 **/
	template<class T>
	static inline T Dist2_2D(const T *x1, const T *x2)
	{
		T dx = x2[0] - x1[0];
		T dy = x2[1] - x1[1];

		return sqr(dx) + sqr(dy);
	}

	/**
	 * @function Dist2
	 * @return Squared distance between two points in 3D space
	 **/
	template<class T>
	static inline T Dist2(const T *x1, const T *x2)
	{
		T dx = x2[0] - x1[0];
		T dy = x2[1] - x1[1];
		T dz = x2[2] - x1[2];

		return sqr(dx) + sqr(dy) + sqr(dz);
	}

	/**
	 * @function rad
	 * @brief degree to rad conversion
	 * @param deg degree value
	 * @return rad rad value
	 **/
	static inline double deg2rad(const double deg)
	{
		return ((M_PI * deg) / 180.0);
	}

	/**
	 * @function deg
	 * @brief rad to degree conversion
	 * @param rad rad value
	 * @return deg degree value
	 **/
	static inline double rad2deg(const double rad)
	{
		return ((rad * 180.0) / M_PI);
	}

	/**
	 * Shift angle to [-PI, PI] range
	 * @param dAngle rad angle with range [-2*PI, 2*PI]
	 * @return rad angle with valid range
	 */
	static inline double getPiCorrectedAngle(const double dAngle)
	{
		double dCorrectedAngle = dAngle;
		if (dCorrectedAngle < M_PI)
			dCorrectedAngle += 2 * M_PI;
		else if (dCorrectedAngle > M_PI)
			dCorrectedAngle -= 2 * M_PI;
		return dCorrectedAngle;
	}

	/**
	 * Create lookup table for sqare root function (speedup purpose)
	 * @param ptLut pointer to lookup table (must already be instanciated)
	 * @param tSize size of lookup table
	 */
	template<class T>
	void fairCreateSqRtLut(T* ptLut, T tSize)
	{
		for (T t = 0; t < tSize; ++t)
			ptLut[t] = (T) (sqrt((float) t) + 0.5);
	}

// CubeRoot implementation from http://www.worldserver.com/turk/computergraphics/CubeRoot.pdf
// The OpenCV improvements are included.
// Note: Determine, if this function is faster than pow! Some compiler make better code for the pow function.
	typedef union fair32suf
	{
		int i;
		unsigned u;
		float f;
	} fair32suf;

	inline float fairCbRt24bit(float value)
	{
		float fr;
		fair32suf v, m;
		int ix, s;
		int ex, shx;

		v.f = value;
		ix = v.i & 0x7fffffff;
		s = v.i & 0x80000000;
		ex = (ix >> 23) - 127;
		shx = ex % 3;
		shx -= shx >= 0 ? 3 : 0;
		ex = (ex - shx) / 3; /* exponent of cube root */
		v.i = (ix & ((1 << 23) - 1)) | ((shx + 127) << 23);
		fr = v.f;

		/* 0.125 <= fr < 1.0 */
		/* Use quartic rational polynomial with error < 2^(-24) */
		fr = ((((45.2548339756803022511987494 * fr + 192.2798368355061050458134625) * fr + 119.1654824285581628956914143) * fr + 13.43250139086239872172837314)
		        * fr + 0.1636161226585754240958355063)
		        / ((((14.80884093219134573786480845 * fr + 151.9714051044435648658557668) * fr + 168.5254414101568283957668343) * fr
		                + 33.9905941350215598754191872) * fr + 1.0);

		/* fr *= 2^ex * sign */
		m.f = value;
		v.f = fr;
		v.i = (v.i + (ex << 23) + s) & (m.i * 2 != 0 ? -1 : 0);
		return v.f;
	}

	inline float fairCbRt6bit(float value)
	{
		float fr;
		fair32suf v, m;
		int ix, s;
		int ex, shx;

		v.f = value;
		ix = v.i & 0x7fffffff;
		s = v.i & 0x80000000;
		ex = (ix >> 23) - 127;
		shx = ex % 3;
		shx -= shx >= 0 ? 3 : 0;
		ex = (ex - shx) / 3; /* exponent of cube root */
		v.i = (ix & ((1 << 23) - 1)) | ((shx + 127) << 23);
		fr = v.f;

		/* 0.125 <= fr < 1.0 */
		/* Use polynomial with error < 2^(-6) */
		fr = -0.46946116 * fr * fr + 1.072302 * fr + 0.3812513;

		/* fr *= 2^ex * sign */
		m.f = value;
		v.f = fr;
		v.i = (v.i + (ex << 23) + s) & (m.i * 2 != 0 ? -1 : 0);
		return v.f;
	}

	/**
	 * Cube root calculation based on an approximation (see upper)
	 * @param value input value
	 * @return cube root of input value
	 */
	inline float fairCbRt(float value)
	{
		return fairCbRt24bit(value);
	}

	/**
	 * Calculate the squared euklidian distance between two n-dimensional points
	 * @param pfCoords1 first coordinate vector
	 * @param pfCoords2 second coordinate vector
	 * @param nSize size of coordinate vectors (must both be equally sized)
	 * @return squared euklidean distance
	 */
	inline float fairEuklidianDistanceSqr(float* pfCoords1, float* pfCoords2, int nSize)
	{
		float fSqr = 0.0f;
		for (int i = 0; i < nSize; i++)
		{
			float fTmp = pfCoords1[i] - pfCoords2[i];
			fSqr += fTmp * fTmp;
		}
		return fSqr;
	}

	/**
	 * Calculate the euklidian distance between two n-dimensional points
	 * @param pfCoords1 first coordinate vector
	 * @param pfCoords2 second coordinate vector
	 * @param nSize size of coordinate vectors (must both be equally sized)
	 * @return euklidean distance
	 */
	inline float fairEuklidianDistance(float* pfCoords1, float* pfCoords2, int nSize)
	{
		float fSqr = 0.0f;
		for (int i = 0; i < nSize; i++)
		{
			float fTmp = pfCoords1[i] - pfCoords2[i];
			fSqr += fTmp * fTmp;
		}
		return sqrt(fSqr);
	}

	/**
	 * Calculate the L1 distance between two n-dimensional points
	 * @param pfCoords1 first coordinate vector
	 * @param pfCoords2 second coordinate vector
	 * @param nSize size of coordinate vectors (must both be equally sized)
	 * @return L1 distance
	 */
	inline float fairL1Distance(float* pfCoords1, float* pfCoords2, int nSize)
	{
		float fDist = 0.0f;
		for (int i = 0; i < nSize; i++)
		{
			float fTmp = pfCoords1[i] - pfCoords2[i];
			fDist += fabs(fTmp);
		}
		return fDist;
	}

	/**
	 * Calculates the empirical correlation coefficient of two data vectors
	 * @param pfFeatures1 first data vector
	 * @param pfFeatures2 second data vector
	 * @param nSize size of data vectors (must be be equally sized)
	 * @return empirical correlation coefficient
	 */
	inline float fairEmpiricalCorrelation(float* pfFeatures1, float* pfFeatures2, int nSize)
	{
		if (nSize < 1)
			return 0.0f;
		if (nSize == 1)
		{
			if ((pfFeatures2[0] == 0.0f) && (pfFeatures1[0] == 0.0f))
				return 1.0f;

			if ((pfFeatures2[0] == 0.0f) || (pfFeatures1[0] == 0.0f))
				return 0.0f;

			float fRatio = pfFeatures1[0] / pfFeatures2[0];
			if (fRatio > 1.0f)
				fRatio = 1.0f / fRatio;
			return fRatio;
		}

		// calculate the empirical correlation coefficient between the feature vectors of both pixels
		float fX = 0.0f;
		float fY = 0.0f;
		float fXX = 0.0f;
		float fYY = 0.0f;
		float fXY = 0.0f;

		float fSize = (float) nSize;

		for (int i = 0; i < nSize; ++i)
		{
			float fXTmp = pfFeatures1[i];
			float fYTmp = pfFeatures2[i];
			;
			// Zero values are not significant
			if ((fXTmp > 0.0f) || (fYTmp > 0.0f))
			{
				fX += fXTmp;
				fY += fYTmp;
				fXX += fXTmp * fXTmp;
				fYY += fYTmp * fYTmp;
				fXY += fXTmp * fYTmp;
			}
			else
				fSize -= 1.0f;
		}

		// No significant values passed
		if (fSize < 1.0f)
			return 0.0f;

		float fCorr;
		if (fSize == 1.0f)
		{
			if ((fY == 0.0f) && (fX == 0.0f))
				return 1.0f;

			if ((fY == 0.0f) || (fX == 0.0f))
				return 0.0f;

			fCorr = fX / fY;

			if (fCorr > 1.0f)
				fCorr = 1.0f / fCorr;
		}
		else
		{
			fCorr = fXY - 1.0f / fSize * fX * fY;
			float fArg = (fXX - 1.0f / fSize * fX * fX) * (fYY - 1.0f / fSize * fY * fY);

			fairAssert(fArg>=0.0f, "Argument for sqrt operation is negative");

			if (fArg < 0.1)
				fCorr = 0.0f;
			else
				fCorr /= sqrt(fArg);
		}

		return fCorr;
	}

}  // namespace

#endif //FAIRMATHBASE_H
