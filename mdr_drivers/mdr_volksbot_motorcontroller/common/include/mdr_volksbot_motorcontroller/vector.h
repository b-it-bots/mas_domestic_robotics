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

#ifndef VECTOR_H_
#define VECTOR_H_

#include <assert.h>
#include <math.h>

//#pragma warning (disable:1572)	
#ifdef WIN32
#pragma warning(disable:4786)
#endif

namespace fair
{

	class CVector2;
	class CVector3;
	class CVector4;

	/**
	 * @class CVector2
	 * @brief Class encapsules 2D vector representations and operations
	 * @author Stefan May
	 **/
	class CVector2
	{
 	public:
		/**
		 *  x coodinate
		 */
		float x;

		/**
		 *  y coordinate
		 */
		float y;

 	public:
		/**
		 *  Constructors
		 */
		CVector2()
		{
		}
		;
		/**
		 *  Constructor with initializing float values
		 */
		CVector2(float inX, float inY)
				: x(inX),
				  y(inY)
		{
		}
		;
		/**
		 *  Constructor with initializing CVector2
		 */
		CVector2(const CVector2 &v)
				: x(v.x),
				  y(v.y)
		{
		}
		;

 	public:
		/**
		 *  Array indexing
		 */
		float &operator [](unsigned int i)
		{
			assert(i < 2);
			return *(&x + i);
		}
		/**
		 *  Array indexing
		 */
		const float &operator [](unsigned int i) const
		{
			assert(i < 2);
			return *(&x + i);
		}
		/**
		 *  Add a CVector2 to this one
		 */
		CVector2 &operator +=(const CVector2 &v)
		{
			x += v.x;
			y += v.y;
			return *this;
		}
		/**
		 *  Subtract a CVector2 from this one
		 */
		CVector2 &operator -=(const CVector2 &v)
		{
			x -= v.x;
			y -= v.y;
			return *this;
		}
		/**
		 *  Multiply the CVector2 by a float
		 */
		CVector2 &operator *=(float f)
		{
			x *= f;
			y *= f;
			return *this;
		}
		/**
		 *  Divide the CVector2 by a float
		 */
		CVector2 &operator /=(float f)
		{
			x /= f;
			y /= f;
			return *this;
		}
		/**
		 *  Are these two CVector2's equal?
		 */
		friend bool operator ==(const CVector2 &a, const CVector2 &b)
		{
			return ((a.x == b.x) && (a.y == b.y));
		}
		/**
		 *  Are these two CVector2's not equal?
		 */
		friend bool operator !=(const CVector2 &a, const CVector2 &b)
		{
			return ((a.x != b.x) || (a.y != b.y));
		}
		/**
		 *  Negate this vector
		 */
		friend CVector2 operator -(const CVector2 &a)
		{
			return CVector2(-a.x, -a.y);
		}
		/**
		 *  Add two CVector2's
		 */
		friend CVector2 operator +(const CVector2 &a, const CVector2 &b)
		{
			CVector2 ret(a);
			ret += b;
			return ret;
		}
		/**
		 *  Subtract one CVector2 from another
		 */
		friend CVector2 operator -(const CVector2 &a, const CVector2 &b)
		{
			CVector2 ret(a);
			ret -= b;
			return ret;
		}
		/**
		 *  Multiply CVector2 by a float
		 */
		friend CVector2 operator *(const CVector2 &v, float f)
		{
			return CVector2(f * v.x, f * v.y);
		}
		/**
		 *  Multiply CVector2 by a float
		 */
		friend CVector2 operator *(float f, const CVector2 &v)
		{
			return CVector2(f * v.x, f * v.y);
		}
		/**
		 *  Divide CVector2 by a float
		 */
		friend CVector2 operator /(const CVector2 &v, float f)
		{
			return CVector2(v.x / f, v.y / f);
		}

 	public:
		// Methods
		/**
		 *  Set Values
		 */
		void set(float xIn, float yIn)
		{
			x = xIn;
			y = yIn;
		}
		/**
		 *  Get length of a CVector2
		 */
		float length() const
		{
			return (float) sqrt(x * x + y * y);
		}
		/**
		 *  Get squared length of a CVector2
		 */
		float lengthSqr() const
		{
			return (x * x + y * y);
		}
		/**
		 *  Does CVector2 equal (0, 0)?
		 */
		bool isZero() const
		{
			return ((x == 0.0F) && (y == 0.0F));
		}
		/**
		 *  Normalize a CVector2
		 */
		CVector2 &normalize()
		{
			float m = length();

			if (m > 0.0F)
				m = 1.0F / m;
			else
				m = 0.0F;
			x *= m;
			y *= m;

			return *this;
		}

	};

	/**
	 * @class CVector3
	 * @brief Class encapsules 3D vector representations and operations
	 * @author Stefan May
	 **/
	class CVector3
	{
 	public:
		/**
		 *  x coordinate
		 */
		float x;

		/**
		 *  y coordinate
		 */
		float y;

		/**
		 *  z coordinate
		 */
		float z;

 	public:
		/**
		 *  Constructors
		 */
		CVector3()
		{
		}
		;
		/**
		 *  Constructor with initializing float values
		 */
		CVector3(float inX, float inY, float inZ)
				: x(inX),
				  y(inY),
				  z(inZ)
		{
		}
		/**
		 *  Constructor with initializing CVector3
		 */
		CVector3(const CVector3 &v)
				: x(v.x),
				  y(v.y),
				  z(v.z)
		{
		}
		/**
		 *  Constructor with initializing CVector2
		 */
		explicit CVector3(const CVector2 &v)
				: x(v.x),
				  y(v.y),
				  z(0.0F)
		{
		}

		/**
		 * Constructor with initializing CVector4. The first three dimensions will be taken.
		 */
		// ToDo
		explicit CVector3(const CVector4 &v);

 	public:
		// Operators
		/**
		 *  Array indexing
		 */
		float &operator [](unsigned int i)
		{
			assert(i < 3);
			return *(&x + i);
		}
		/**
		 *  Array indexing
		 */
		const float &operator [](unsigned int i) const
		{
			assert(i < 3);
			return *(&x + i);
		}
		/**
		 *  Assign from a CVector2
		 */
		CVector3 &operator =(const CVector2 &v)
		{
			x = v.x;
			y = v.y;
			z = 0.0F;
			return *this;
		}
		/**
		 *  Add a CVector3 to this one
		 */
		CVector3 &operator +=(const CVector3 &v)
		{
			x += v.x;
			y += v.y;
			z += v.z;
			return *this;
		}
		/**
		 *  Subtract a CVector3 from this one
		 */
		CVector3 &operator -=(const CVector3 &v)
		{
			x -= v.x;
			y -= v.y;
			z -= v.z;
			return *this;
		}
		/**
		 *  Multiply the CVector3 by a float
		 */
		CVector3 &operator *=(float f)
		{
			x *= f;
			y *= f;
			z *= f;
			return *this;
		}
		/**
		 *  Divide the CVector3 by a float
		 */
		CVector3 &operator /=(float f)
		{
			x /= f;
			y /= f;
			z /= f;
			return *this;
		}
		/**
		 *  Are these two CVector3's equal?
		 */
		friend bool operator ==(const CVector3 &a, const CVector3 &b)
		{
			return ((a.x == b.x) && (a.y == b.y) && (a.z == b.z));
		}
		/**
		 *  Are these two CVector3's not equal?
		 */
		friend bool operator !=(const CVector3 &a, const CVector3 &b)
		{
			return ((a.x != b.x) || (a.y != b.y) || (a.z != b.z));
		}
		/**
		 *  Negate a CVector3
		 */
		friend CVector3 operator -(const CVector3 &a)
		{
			return CVector3(-a.x, -a.y, -a.z);
		}
		/**
		 *  Add two CVector3's
		 */
		friend CVector3 operator +(const CVector3 &a, const CVector3 &b)
		{
			CVector3 ret(a);
			ret += b;
			return ret;
		}
		/**
		 *  Subtract one CVector3 from another
		 */
		friend CVector3 operator -(const CVector3 &a, const CVector3 &b)
		{
			CVector3 ret(a);
			ret -= b;
			return ret;
		}
		/**
		 *  Multiply CVector3 by a float
		 */
		friend CVector3 operator *(const CVector3 &v, float f)
		{
			return CVector3(f * v.x, f * v.y, f * v.z);
		}
		/**
		 *  Multiply CVector3 by a float
		 */
		friend CVector3 operator *(float f, const CVector3 &v)
		{
			return CVector3(f * v.x, f * v.y, f * v.z);
		}
		/**
		 *  Divide CVector3 by a float
		 */
		friend CVector3 operator /(const CVector3 &v, float f)
		{
			return CVector3(v.x / f, v.y / f, v.z / f);
		}

 	public:
		// Methods
		/**
		 *  Set Values
		 */
		void set(float xIn, float yIn, float zIn)
		{
			x = xIn;
			y = yIn;
			z = zIn;
		}
		/**
		 *  Get length of a CVector3
		 */
		float length() const
		{
			return (float) sqrt(x * x + y * y + z * z);
		}
		/**
		 *  Get squared length of a CVector3
		 */
		float lengthSqr() const
		{
			return (x * x + y * y + z * z);
		}
		/**
		 *  Does CVector3 equal (0, 0, 0)?
		 */
		bool isZero() const
		{
			return ((x == 0.0F) && (y == 0.0F) && (z == 0.0F));
		}
		/**
		 *  Normalize a CVector3
		 */
		CVector3 &normalize()
		{
			float m = length();
			if (m > 0.0F)
				m = 1.0F / m;
			else
				m = 0.0F;
			x *= m;
			y *= m;
			z *= m;
			return *this;
		}

	};

	/**
	 * @class CVector4
	 * @brief Class encapsules 4D vector representations and operations
	 * @author Stefan May
	 **/
	class CVector4
	{
 	public:
		/**
		 * x coordinate
		 */
		float x;

		/**
		 *  y coordinate
		 */
		float y;

		/**
		 *  z coordinate
		 */
		float z;

		/**
		 *  w coordinate
		 */
		float w;

 	public:
		/**
		 *  Constructors
		 */
		// CVector4(): x(0), y(0), z(0), w(0) {};
		CVector4()
		{
		}
		;
		/**
		 *  Constructor with initializing float values
		 */
		CVector4(float inX, float inY, float inZ, float inW)
				: x(inX),
				  y(inY),
				  z(inZ),
				  w(inW)
		{
		}
		;
		/**
		 *  Constructor with initializing CVector4
		 */
		CVector4(const CVector4 &v)
				: x(v.x),
				  y(v.y),
				  z(v.z),
				  w(v.w)
		{
		}
		;
		/**
		 *  Constructor with initializing CVector3
		 */
		explicit CVector4(const CVector3 &v)
				: x(v.x),
				  y(v.y),
				  z(v.z),
				  w(0.0F)
		{
		}
		;
		/**
		 *  Constructor with initializing CVector2
		 */
		explicit CVector4(const CVector2 &v)
				: x(v.x),
				  y(v.y),
				  z(0.0F),
				  w(0.0F)
		{
		}
		;

 	public:
		// Operators
		/**
		 *  Array indexing
		 */
		float &operator [](unsigned int i)
		{
			assert(i < 4);
			//return *(&x+i);
			return (i == 0) ? x : (i == 1) ? y : (i == 2) ? z : w;
		}
		/**
		 *  Array indexing
		 */
		const float &operator [](unsigned int i) const
		{
			assert(i < 4);
			//return *(&x+i);
			return (i == 0) ? x : (i == 1) ? y : (i == 2) ? z : w;
		}
		/**
		 *  Assign from a CVector3
		 */
		CVector4 &operator =(const CVector3 &v)
		{
			x = v.x;
			y = v.y;
			z = v.z;
			w = 0.0F;
			return *this;
		}
		/**
		 *  Assign from a CVector2
		 */
		CVector4 &operator =(const CVector2 &v)
		{
			x = v.x;
			y = v.y;
			z = 0.0F;
			w = 0.0F;
			return *this;
		}
		/**
		 *  Add a CVector4 to this one
		 */
		CVector4 &operator +=(const CVector4 &v)
		{
			x += v.x;
			y += v.y;
			z += v.z;
			w += v.w;
			return *this;
		}
		/**
		 *  Subtract a CVector4 from this one
		 */
		CVector4 &operator -=(const CVector4 &v)
		{
			x -= v.x;
			y -= v.y;
			z -= v.z;
			w -= v.w;
			return *this;
		}
		/**
		 *  Multiply the CVector4 by a float
		 */
		CVector4 &operator *=(float f)
		{
			x *= f;
			y *= f;
			z *= f;
			w *= f;
			return *this;
		}
		/**
		 *  Divide the CVector4 by a float
		 */
		CVector4 &operator /=(float f)
		{
			x /= f;
			y /= f;
			z /= f;
			w /= f;
			return *this;
		}
		/**
		 *  Are these two CVector4's equal?
		 */
		friend bool operator ==(const CVector4 &a, const CVector4 &b)
		{
			return ((a.x == b.x) && (a.y == b.y) && (a.z == b.z) && (a.w == b.w));
		}
		/**
		 *  Are these two CVector4's not equal?
		 */
		friend bool operator !=(const CVector4 &a, const CVector4 &b)
		{
			return ((a.x != b.x) || (a.y != b.y) || (a.z != b.z) || (a.w != b.w));
		}
		/**
		 *  Negate a CVector4
		 */
		friend CVector4 operator -(const CVector4 &a)
		{
			return CVector4(-a.x, -a.y, -a.z, -a.w);
		}
		/**
		 *  Add two CVector4's
		 */
		friend CVector4 operator +(const CVector4 &a, const CVector4 &b)
		{
			CVector4 ret(a);
			ret += b;
			return ret;
		}
		/**
		 *  Subtract one CVector4 from another
		 */
		friend CVector4 operator -(const CVector4 &a, const CVector4 &b)
		{
			CVector4 ret(a);
			ret -= b;
			return ret;
		}
		/**
		 *  Multiply CVector4 by a float
		 */
		friend CVector4 operator *(const CVector4 &v, float f)
		{
			return CVector4(f * v.x, f * v.y, f * v.z, f * v.w);
		}
		/**
		 *  Multiply CVector4 by a float
		 */
		friend CVector4 operator *(float f, const CVector4 &v)
		{
			return CVector4(f * v.x, f * v.y, f * v.z, f * v.w);
		}
		/**
		 *  Divide CVector4 by a float
		 */
		friend CVector4 operator /(const CVector4 &v, float f)
		{
			return CVector4(v.x / f, v.y / f, v.z / f, v.w / f);
		}

 	public:
		// Methods
		/**
		 *  Set Values
		 */
		void set(float xIn, float yIn, float zIn, float wIn)
		{
			x = xIn;
			y = yIn;
			z = zIn;
			w = wIn;
		}
		/**
		 *  Get length of a CVector4
		 */
		float length() const
		{
			return (float) sqrt(x * x + y * y + z * z + w * w);
		}
		/**
		 *  Get squared length of a CVector4
		 */
		float lengthSqr() const
		{
			return (x * x + y * y + z * z + w * w);
		}
		/**
		 *  Does CVector4 equal (0, 0, 0, 0)?
		 */
		bool isZero() const
		{
			return ((x == 0.0F) && (y == 0.0F) && (z == 0.0F) && (w == 0.0F));
		}
		/**
		 *  Normalize a CVector4
		 */
		CVector4 &normalize()
		{
			float m = length();
			if (m > 0.0F)
				m = 1.0F / m;
			else
				m = 0.0F;
			x *= m;
			y *= m;
			z *= m;
			w *= m;
			return *this;
		}

	};

////////////////////////////////////////////////////////////
// Miscellaneous vector functions
//
	CVector2 Normalized(const CVector2 &a);
	CVector3 Normalized(const CVector3 &a);
	CVector4 Normalized(const CVector4 &a);
	float DotProduct(const CVector2 &a, const CVector2 &b);
	float DotProduct(const CVector3 &a, const CVector3 &b);
	float DotProduct(const CVector4 &a, const CVector4 &b);
	void SwapVec(CVector2 &a, CVector2 &b);
	void SwapVec(CVector3 &a, CVector3 &b);
	void SwapVec(CVector4 &a, CVector4 &b);
	CVector3 CrossProduct(const CVector3 &a, const CVector3 &b);
	bool NearlyEquals(const CVector2 &a, const CVector2 &b, float r);
	bool NearlyEquals(const CVector3 &a, const CVector3 &b, float r);
	bool NearlyEquals(const CVector4 &a, const CVector4 &b, float r);

}

//#pragma warning (default:1572)

#endif /*VECTOR_H_*/
