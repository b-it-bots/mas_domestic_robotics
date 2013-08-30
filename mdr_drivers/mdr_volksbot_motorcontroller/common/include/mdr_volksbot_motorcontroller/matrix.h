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

#ifndef MATRIX_H_
#define MATRIX_H_

#include "mathbase.h"
#include "vector.h"

#include <stdio.h>

//#pragma warning (disable:981)
//#pragma warning (disable:383)

/**
 * @namespace fair
 */
namespace fair
{

	static inline float DegToRad(float a)
	{
		return a * 0.01745329252f;
	}
	static inline float RadToDeg(float a)
	{
		return a * 57.29577951f;
	}

	class CMatrix33;
	class CMatrix44;

	/**
	 * @class CMatrix33
	 * @brief Class encapsules 3x3 matrix representations and operations
	 * @author Stefan May
	 **/
	class CMatrix33
	{
 	public:
		/**
		 *  Members
		 */
		CVector3 col[3];

 	public:
		/**
		 *  Constructors
		 */
		CMatrix33()
		{
		}
		;
		/**
		 *  Constructor with initializing value
		 */
		CMatrix33(float v)
		{
			col[0].set(v, v, v);
			col[1].set(v, v, v);
			col[2].set(v, v, v);
		}
		/**
		 *  Constructor with initializing CMatrix33
		 */
		CMatrix33(const CMatrix33 &m)
		{
			col[0] = m[0];
			col[1] = m[1];
			col[2] = m[2];
		}
		/**
		 *  Constructor with initializing CVector3's
		 */
		CMatrix33(const CVector3 &v0, const CVector3 &v1, const CVector3 &v2)
		{
			col[0] = v0;
			col[1] = v1;
			col[2] = v2;
		}

 	public:
		// Operators
		/**
		 *  Array indexing
		 */
		CVector3 &operator [](unsigned int i)
		{
			assert(i < 3);
			return (CVector3&) col[i];
		}
		/**
		 *  Array indexing
		 */
		const CVector3 &operator [](unsigned int i) const
		{
			assert(i < 3);
			return (CVector3&) col[i];
		}
		/**
		 *  Assign
		 */
		CMatrix33 &operator =(const CMatrix33 &m)
		{
			col[0] = m[0];
			col[1] = m[1];
			col[2] = m[2];
			return *this;
		}
		/**
		 *  Add a CMatrix33 to this one
		 */
		CMatrix33 &operator +=(const CMatrix33 &m)
		{
			col[0] += m[0];
			col[1] += m[1];
			col[2] += m[2];
			return *this;
		}
		/**
		 *  Subtract a CMatrix33 from this one
		 */
		CMatrix33 &operator -=(const CMatrix33 &m)
		{
			col[0] -= m[0];
			col[1] -= m[1];
			col[2] -= m[2];
			return *this;
		}
		/**
		 *  Multiply the CMatrix33 by another CMatrix33
		 */
		CMatrix33 &operator *=(const CMatrix33 &m);
		/**
		 *  Multiply the CMatrix33 by a float
		 */
		CMatrix33 &operator *=(float f)
		{
			col[0] *= f;
			col[1] *= f;
			col[2] *= f;
			return *this;
		}
		/**
		 *  Divide the CMatrix33 by a float
		 */
		CMatrix33 &operator /=(float f)
		{
			col[0] /= f;
			col[1] /= f;
			col[2] /= f;
			return *this;
		}
		/**
		 *  Are these two CMatrix33's equal?
		 */
		friend bool operator ==(const CMatrix33 &a, const CMatrix33 &b)
		{
			return ((a[0] == b[0]) && (a[1] == b[1]) && (a[2] == b[2]));
		}
		/**
		 *  Are these two CMatrix33's not equal?
		 */
		friend bool operator !=(const CMatrix33 &a, const CMatrix33 &b)
		{
			return ((a[0] != b[0]) || (a[1] != b[1]) || (a[2] != b[2]));
		}
		/**
		 *  Add two CMatrix33's
		 */
		friend CMatrix33 operator +(const CMatrix33 &a, const CMatrix33 &b)
		{
			CMatrix33 ret(a);
			ret += b;
			return ret;
		}
		/**
		 *  Subtract one CMatrix33 from another
		 */
		friend CMatrix33 operator -(const CMatrix33 &a, const CMatrix33 &b)
		{
			CMatrix33 ret(a);
			ret -= b;
			return ret;
		}
		/**
		 *  Multiply CMatrix33 by another CMatrix33
		 */
		friend CMatrix33 operator *(const CMatrix33 &a, const CMatrix33 &b)
		{
			CMatrix33 ret(a);
			ret *= b;
			return ret;
		}
		/**
		 *  Multiply a CVector3 by this CMatrix33
		 */
		friend CVector3 operator *(const CMatrix33 &m, const CVector3 &v)
		{
			CVector3 ret;
			ret.x = DotProduct(m[0], v);
			ret.y = DotProduct(m[1], v);
			ret.z = DotProduct(m[2], v);
			return ret;
		}
		/**
		 *  Multiply a CVector3 by this CMatrix33
		 */
		friend CVector3 operator *(const CVector3 &v, const CMatrix33 &m)
		{
			CVector3 ret;
			ret.x = v.x * m[0][0] + v.y * m[1][0] + v.z * m[2][0];
			ret.y = v.x * m[0][1] + v.y * m[1][1] + v.z * m[2][1];
			ret.z = v.x * m[0][2] + v.y * m[1][2] + v.z * m[2][2];
			return ret;
		}
		/**
		 *  Multiply CMatrix33 by a float
		 */
		friend CMatrix33 operator *(const CMatrix33 &m, float f)
		{
			CMatrix33 ret(m);
			ret *= f;
			return ret;
		}
		/**
		 *  Multiply CMatrix33 by a float
		 */
		friend CMatrix33 operator *(float f, const CMatrix33 &m)
		{
			CMatrix33 ret(m);
			ret *= f;
			return ret;
		}

 	public:
		// Methods
		/**
		 *  Set CMatrix33 to the identity matrix
		 */
		CMatrix33 &identity()
		{
			col[0].set(1.0, 0.0, 0.0);
			col[1].set(0.0, 1.0, 0.0);
			col[2].set(0.0, 0.0, 1.0);
			return *this;
		}
		/**
		 *  Transpose the CMatrix33
		 */
		CMatrix33 &transpose();
		/**
		 *  Invert the CMatrix33
		 */
		CMatrix33 &invert();

	};

	CMatrix33 IdentityMatrix33();
	CMatrix33 TransposeMatrix33(const CMatrix33 &m);
	CMatrix33 InvertMatrix33(const CMatrix33 &m);
	CMatrix33 RotateRadMatrix33(float rad);
	CMatrix33 TranslateMatrix33(float x, float y);
	CMatrix33 ScaleMatrix33(float x, float y, float z = 1.0);
	void printMatrix33(CMatrix33* m);

	/**
	 * @class CMatrix44
	 * @brief Class encapsules 4x4 matrix representations and operations
	 * @author Stefan May
	 **/
	class CMatrix44
	{
 	public:
		/**
		 *  Members
		 */
		CVector4 col[4];

 	public:
		/**
		 *  Constructors
		 */
		CMatrix44()
		{
		}
		;
		/**
		 *  Constructor with initializing value
		 */
		CMatrix44(float v)
		{
			col[0].set(v, v, v, v);
			col[1].set(v, v, v, v);
			col[2].set(v, v, v, v);
			col[3].set(v, v, v, v);
		}
		/**
		 *  Constructor with initializing CMatrix44
		 */
		CMatrix44(const CMatrix44 &m)
		{
			col[0] = m[0];
			col[1] = m[1];
			col[2] = m[2];
			col[3] = m[3];
		}
		/**
		 *  Constructor with initializing CVector4's
		 */
		CMatrix44(const CVector4 &v0, const CVector4 &v1, const CVector4 &v2, const CVector4 &v3)
		{
			col[0] = v0;
			col[1] = v1;
			col[2] = v2;
			col[3] = v3;
		}
		/**
		 *  Constructor with initializing CMatrix33
		 */
		explicit CMatrix44(const CMatrix33 &m)
		{
			col[0] = m[0];
			col[1] = m[1];
			col[2] = m[2];
			col[3].set(0.0, 0.0, 0.0, 1.0);
		}

 	public:
		// Operators
		/**
		 *  Array indexing
		 */
		CVector4 &operator [](unsigned int i)
		{
			assert(i < 4);
			return col[i];
		}

		/**
		 *  Array indexing
		 */
		const CVector4 &operator [](unsigned int i) const
		{
			assert(i < 4);
			return col[i];
		}

		/**
		 *  Assign
		 */
		CMatrix44 &operator =(const CMatrix44 &m)
		{
			col[0] = m[0];
			col[1] = m[1];
			col[2] = m[2];
			col[3] = m[3];
			return *this;
		}

		/**
		 *  Assign a CMatrix33 to the CMatrix44
		 */
		CMatrix44 &operator =(const CMatrix33 &m)
		{
			col[0] = m[0];
			col[1] = m[1];
			col[2] = m[2];
			col[3].set(0.0, 0.0, 0.0, 1.0);
			return *this;
		}

		/**
		 *  Add a CMatrix44 to this one
		 */
		CMatrix44 &operator +=(const CMatrix44 &m)
		{
			col[0] += m[0];
			col[1] += m[1];
			col[2] += m[2];
			col[3] += m[3];
			return *this;
		}

		/**
		 *  Subtract a CMatrix44 from this one
		 */
		CMatrix44 &operator -=(const CMatrix44 &m)
		{
			col[0] -= m[0];
			col[1] -= m[1];
			col[2] -= m[2];
			col[3] -= m[3];
			return *this;
		}

		/**
		 *  Multiply the CMatrix44 by another CMatrix44
		 */
		CMatrix44 &operator *=(const CMatrix44 &m);

		/**
		 *  Multiply the CMatrix44 by a float
		 */
		CMatrix44 &operator *=(float f)
		{
			col[0] *= f;
			col[1] *= f;
			col[2] *= f;
			col[3] *= f;
			return *this;
		}

		/**
		 * Convert matrix to euler angles
		 * @param pdTranslation translation vector ([x,y,z])
		 * @param pdTheta angle vector ([th1,th2,th3])
		 */
		void toEuler(double *pdTranslation, double *pdTheta)
		{
			if (col[1][0] > 0.998)
			{  // singularity at north pole
				pdTheta[1] = atan2(col[0][2], col[2][2]);
				pdTheta[2] = M_PI / 2;
				pdTheta[0] = 0;
				return;
			}
			if (col[1][0] < -0.998)
			{  // singularity at south pole
				pdTheta[1] = atan2(col[0][2], col[2][2]);
				pdTheta[2] = -M_PI / 2;
				pdTheta[0] = 0;
				return;
			}
			pdTheta[1] = atan2(-col[2][0], col[0][0]);
			pdTheta[0] = atan2(-col[1][2], col[1][1]);
			pdTheta[2] = asin(col[1][0]);

			pdTranslation[0] = col[0][3];
			pdTranslation[1] = col[1][3];
			pdTranslation[2] = col[2][3];
		}

		/**
		 *  Are these two CMatrix44's equal?
		 */
		friend bool operator ==(const CMatrix44 &a, const CMatrix44 &b)
		{
			return ((a[0] == b[0]) && (a[1] == b[1]) && (a[2] == b[2]) && (a[3] == b[3]));
		}
		/**
		 *  Are these two CMatrix44's not equal?
		 */
		friend bool operator !=(const CMatrix44 &a, const CMatrix44 &b)
		{
			return ((a[0] != b[0]) || (a[1] != b[1]) || (a[2] != b[2]) || (a[3] != b[3]));
		}
		/**
		 *  Add two CMatrix44's
		 */
		friend CMatrix44 operator +(const CMatrix44 &a, const CMatrix44 &b)
		{
			CMatrix44 ret(a);
			ret += b;
			return ret;
		}
		/**
		 *  Subtract one CMatrix44 from another
		 */
		friend CMatrix44 operator -(const CMatrix44 &a, const CMatrix44 &b)
		{
			CMatrix44 ret(a);
			ret -= b;
			return ret;
		}
		/**
		 *  Multiply CMatrix44 by another CMatrix44
		 */
		friend CMatrix44 operator *(const CMatrix44 &a, const CMatrix44 &b)
		{
			CMatrix44 ret(a);
			ret *= b;
			return ret;
		}
		/**
		 *  Multiply a CVector3 by this CMatrix44
		 */
		friend CVector3 operator *(const CMatrix44 &m, const CVector3 &v)
		{
			CVector4 ret(v);
			ret = m * ret;
			return CVector3(ret.x, ret.y, ret.z);
		}
		/**
		 *  Multiply a CVector3 by this CMatrix44
		 */
		friend CVector3 operator *(const CVector3 &v, const CMatrix44 &m)
		{
			CVector4 ret(v);
			ret = ret * m;
			return CVector3(ret.x, ret.y, ret.z);
		}
		/**
		 *  Multiply a CVector4 by this CMatrix44
		 */
		friend CVector4 operator *(const CMatrix44 &m, const CVector4 &v)
		{
			CVector4 ret;
			ret.x = DotProduct(m[0], v);
			ret.y = DotProduct(m[1], v);
			ret.z = DotProduct(m[2], v);
			ret.w = DotProduct(m[3], v);
			return ret;
		}
		/**
		 *  Multiply a CVector4 by this CMatrix44
		 */
		friend CVector4 operator *(const CVector4 &v, const CMatrix44 &m)
		{
			CVector4 ret;
			ret.x = v.x * m[0][0] + v.y * m[1][0] + v.z * m[2][0] + v.w * m[3][0];
			ret.y = v.x * m[0][1] + v.y * m[1][1] + v.z * m[2][1] + v.w * m[3][1];
			ret.z = v.x * m[0][2] + v.y * m[1][2] + v.z * m[2][2] + v.w * m[3][2];
			ret.w = v.x * m[0][3] + v.y * m[1][3] + v.z * m[2][3] + v.w * m[3][3];
			return ret;
		}
		/**
		 *  Multiply CMatrix44 with scalar value
		 */
		friend CMatrix44 operator *(const CMatrix44 &m, float f)
		{
			CMatrix44 ret(m);
			ret *= f;
			return ret;
		}
		/**
		 *  Multiply CMatrix44 with scalar value
		 */
		friend CMatrix44 operator *(float f, const CMatrix44 &m)
		{
			CMatrix44 ret(m);
			ret *= f;
			return ret;
		}

 	public:
		// Methods
		/**
		 *  Set CMatrix44 to the identity matrix
		 */
		CMatrix44 &identity()
		{
			col[0].set(1.0, 0.0, 0.0, 0.0);
			col[1].set(0.0, 1.0, 0.0, 0.0);
			col[2].set(0.0, 0.0, 1.0, 0.0);
			col[3].set(0.0, 0.0, 0.0, 1.0);
			return *this;
		}
		/**
		 *  Transpose the CMatrix44
		 */
		CMatrix44 &transpose();
		/**
		 *  Invert the CMatrix44
		 */
		CMatrix44 &invert();
		/**
		 *  Move translation parameters from last col to last row
		 */
		void flipTranslation();

	};

	CMatrix44 IdentityMatrix44();
	CMatrix44 TransposeMatrix44(const CMatrix44 &m);
	CMatrix44 InvertMatrix44(const CMatrix44 &m);
	CMatrix44 RotateRadMatrix44(char axis, float rad);
	CMatrix44 RotateRadMatrix44(const CVector3 &axis, float rad);
	CMatrix44 TranslateMatrix44(float x, float y, float z);
	CMatrix44 ScaleMatrix44(float x, float y, float z, float w = 1.0);
	CMatrix44 LookAtMatrix44(const CVector3 &camPos, const CVector3 &camUp, const CVector3 &target);
	CMatrix44 FrustumMatrix44(float l, float r, float b, float t, float n, float f);
	CMatrix44 PerspectiveMatrix44(float fovY, float aspect, float n, float f);
	CMatrix44 OrthoMatrix44(float l, float r, float b, float t, float n, float f);
	CMatrix44 OrthoNormalMatrix44(const CVector3 &xdir, const CVector3 &ydir, const CVector3 &zdir);
	void printMatrix44(CMatrix44* m);
}

#endif /*MATRIX_H_*/
