#ifndef CORE_FUNCTIONS_H_
#define CORE_FUNCTIONS_H_

#include <math.h>
#include <errno.h>

#include "core_data_types.h"
#include "core_defines.h"
#include "matrix.h"

namespace RoboCup
{
	inline bool isEqual(const double& dValue1, const double& dValue2, const double dEpsilon = 1.0e-12)
	{
		return (fabs(dValue1 - dValue2) < dEpsilon);
	}
}

inline double getManhattanDistance2D(double dX1, double dY1, double dX2, double dY2)
{
	double dDeltaX = fabs(dX1 - dX2);
	double dDeltaY = fabs(dY1 - dY2);
	return (dDeltaX + dDeltaY);
}
;

inline double getManhattanDistance3D(double dX1, double dY1, double dZ1, double dX2, double dY2, double dZ2)
{
	double dDeltaX = fabs(dX1 - dX2);
	double dDeltaY = fabs(dY1 - dY2);
	double dDeltaZ = fabs(dZ1 - dZ2);
	return (dDeltaX + dDeltaY + dDeltaZ);
}
;

inline double getEuclideanDistance2D(double dX1, double dY1, double dX2, double dY2)
{
	double dDeltaX = dX1 - dX2;
	double dDeltaY = dY1 - dY2;
	return (sqrt(dDeltaX * dDeltaX + dDeltaY * dDeltaY));
}
;

inline double getEuclideanDistance3D(double dX1, double dY1, double dZ1, double dX2, double dY2, double dZ2)
{
	double dDeltaX = dX1 - dX2;
	double dDeltaY = dY1 - dY2;
	double dDeltaZ = dZ1 - dZ2;
	return (sqrt(dDeltaX * dDeltaX + dDeltaY * dDeltaY + dDeltaZ * dDeltaZ));
}
;

inline double getSquaredEuclideanDistance2D(double dX1, double dY1, double dX2, double dY2)
{
	double dDeltaX = dX1 - dX2;
	double dDeltaY = dY1 - dY2;
	return (dDeltaX * dDeltaX + dDeltaY * dDeltaY);
}
;

inline double getSquaredEuclideanDistance3D(double dX1, double dY1, double dZ1, double dX2, double dY2, double dZ2)
{
	double dDeltaX = dX1 - dX2;
	double dDeltaY = dY1 - dY2;
	double dDeltaZ = dZ1 - dZ2;
	return (dDeltaX * dDeltaX + dDeltaY * dDeltaY + dDeltaZ * dDeltaZ);
}
;

/**
 * Inline function to compute the squared Euclidean distance between two points on the X-Y plane (2D)
 * @return Squared Euclidean distance in 2D-space
 */
inline double sqrDistance2D(StrPoint a, StrPoint b)
{
	double dX = a.dX - b.dX;
	double dY = a.dY - b.dY;
	return (dX * dX + dY * dY);
}

/**
 * Inline function to compute the Euclidean distance between two points on the X-Y plane (2D)
 * @return Euclidean distance in 2D-space
 */
inline double distance2D(StrPoint a, StrPoint b)
{
	double dX = a.dX - b.dX;
	double dY = a.dY - b.dY;
	return sqrt((dX * dX + dY * dY));
}

/**
 * Inline function to compute the squared Euclidean distance between two points in cartesian space (3D)
 * @return Squared Euclidean distance in 3D-space
 */
inline double sqrDistance3D(StrPoint a, StrPoint b)
{
	double dX = a.dX - b.dX;
	double dY = a.dY - b.dY;
	double dZ = a.dZ - b.dZ;
	return (dX * dX + dY * dY + dZ * dZ);
}

/**
 * Inline function to compute the Euclidean distance between two points in cartesian space (3D)
 * @return Euclidean distance in 3D-space
 */
inline double distance3D(StrPoint a, StrPoint b)
{
	double dX = a.dX - b.dX;
	double dY = a.dY - b.dY;
	double dZ = a.dZ - b.dZ;
	return sqrt((dX * dX + dY * dY + dZ * dZ));
}

inline double correctAngle(double dAngle)
{
	if (dAngle > M_PI)
		return (dAngle - 2.0 * M_PI);
	else if (dAngle < -M_PI)
		return (dAngle + 2.0 * M_PI);
	else
		return dAngle;
}

/**
 * Inline function to convert a laser scan point (x,y,z,roll,pitch,yaw) to an StrPoint
 * @return StrPose of pdLaserScanPoint
 * @author Frederik Hegger, june 2009
 */
inline StrPoint getTransformationFromLaserScanPointToStrPoint(double *pdLaserScanPoint)
{
	StrPoint strPoint;

	strPoint.dDistance = pdLaserScanPoint[BUFFER_COLUMN_DISTANCE];
	strPoint.dX = pdLaserScanPoint[BUFFER_COLUMN_X];
	strPoint.dY = pdLaserScanPoint[BUFFER_COLUMN_Y];
	strPoint.dZ = pdLaserScanPoint[BUFFER_COLUMN_Z];
	strPoint.dRoll = pdLaserScanPoint[BUFFER_COLUMN_ROLL];
	strPoint.dPitch = pdLaserScanPoint[BUFFER_COLUMN_PITCH];
	strPoint.dYaw = pdLaserScanPoint[BUFFER_COLUMN_YAW];

	return strPoint;
}

inline fair::CMatrix44 getTransformationFrom2DPose(StrRobotPose* pstrPose)
{
	fair::CMatrix44 m;
	m.identity();
	if (pstrPose->dYaw != 0)
	{
		double dCosTheta = cos(pstrPose->dYaw);
		double dSinTheta = sin(pstrPose->dYaw);

		m[0][0] = dCosTheta;
		m[0][1] = -dSinTheta;
		m[1][0] = dSinTheta;
		m[1][1] = dCosTheta;
	}
	m[0][3] = pstrPose->dX;
	m[1][3] = pstrPose->dY;

	return m;
}

#ifndef NRPC_JACOBI_ROTATE
#define NRPC_JACOBI_ROTATE(a,i,j,k,l)\
        g=a[i][j];\
        h=a[k][l];\
        a[i][j]=g-s*(h+g*tau);\
        a[k][l]=h+s*(g-h*tau);
#endif

inline void NRPC_jacobi(fair::CMatrix33& matrix, fair::CVector3& vectorEigenValues, fair::CMatrix33& matrixEigenVectors, int* pnNumberRotations)
{
	int j, iq, ip, i;
	float tresh, theta, tau, t, sm, s, h, g, c, *b, *z;

	fair::CVector3 vB, vZ(0, 0, 0);

	// initialize matrix of eigenvectors to be the identity matrix
	fair::CMatrix33 mEigVec;
	mEigVec.identity();

	// initialize vector of eigenvalues (diagonal matrix) to be the diagonal of the matrix
	fair::CVector3 vEigVal;
	vEigVal[0] = b[0] = matrix[0][0];
	vEigVal[1] = b[1] = matrix[1][1];
	vEigVal[2] = b[2] = matrix[2][2];

	(*pnNumberRotations) = 0;

	int n = 2;  // TEMP!

	for (i = 1; i <= 50; i++)
	{
		// sum over off-diagonal elements
		sm = fabs(matrix[0][1]) + fabs(matrix[0][2]) + fabs(matrix[1][2]);
		if (sm == 0.0)
			return;

		if (i < 4)
			tresh = 0.2 * sm / (9);
		else
			tresh = 0.0;

		for (ip = 1; ip <= n - 1; ++i)
		{
			for (iq = ip + 1; iq <= n; ++iq)
			{
				g = 100.0 * fabs(matrix[ip][iq]);
				if ((i > 4) && ((float) (fabs(vEigVal[ip]) + g) == (float) fabs(vEigVal[ip])) && ((float) (fabs(vEigVal[iq]) + g) == (float) fabs(vEigVal[iq])))
				{
					matrix[ip][iq] = 0.0;
				}
				else if (fabs(matrix[ip][iq]) > tresh)
				{
					h = vEigVal[iq] - vEigVal[ip];
					if ((float) (fabs(h) + g) == (float) fabs(h))
						t = (matrix[ip][iq]) / h;
				}
				else
				{
					theta = 0.5 * h / (matrix[ip][iq]);
					t = 1.0 / (fabs(theta) + sqrt(1.0 + theta * theta));
					if (theta < 0.0)
						t = -t;
				}

				c = 1.0 / sqrt(1 + t * t);
				s = t * c;
				tau = s / (1.0 + c);
				h = t * matrix[ip][iq];
				z[ip] -= h;
				z[iq] += h;
				vEigVal[ip] -= h;
				vEigVal[iq] += h;
				matrix[ip][iq] = 0.0;
				for (j = 1; j <= ip - 1; ++j)
					NRPC_JACOBI_ROTATE(matrix, j, ip, j, iq)
				for (j = ip + 1; j <= iq - 1; ++j)
					NRPC_JACOBI_ROTATE(matrix, ip, j, j, iq)
				for (j = iq + 1; j <= n; ++j)
					NRPC_JACOBI_ROTATE(matrix, ip, j, iq, j)
				for (j = 1; j <= n; ++j)
					NRPC_JACOBI_ROTATE(mEigVec, j, ip, j, iq)
				++(*pnNumberRotations);

			}
		}
		for (ip = 1; ip <= n; ip++)
		{
			b[ip] += z[ip];
			vEigVal[ip] = b[ip];
			z[ip] = 0.0;
		}
	}
	printf("Too many iterations in routine jacobi");
}

/**
 * Calculate background brightness according to HDTV standard
 * I = 0.212671 * R + 0.715160 * G + 0.072169 * B
 */
inline float getColorBrightness(float fR, float fG, float fB)
{
	return ((0.212671f * fR + 0.715160f * fG + 0.072169f * fB) / 255.0f);
}

#endif /*CORE_FUNCTIONS_H_*/
