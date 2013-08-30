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

#ifndef CARTESIANBASE_H
#define CARTESIANBASE_H

#include "matrix.h"
#include <stdio.h>
#include <string.h>

/**
 * @namespace fair
 */
namespace fair
{

	/**
	 * @struct StrPointInfo
	 * @brief additional informations for each point.
	 * This structure is kept extra to minimize overhead for cartesian clouds without any additional informations.
	 * @author Stefan May
	 */
	struct StrPointInfo
	{
		/** Default constructor **/
		StrPointInfo()
		{
			pdUserData = NULL;
			unUserDataLength = 0;
		}
		;

		/** Copy constructor **/
		StrPointInfo(StrPointInfo* info)
		{
			fIntensity = info->fIntensity;
			fAmplitude = info->fAmplitude;
			dAccuracy = info->dAccuracy;
			afRGB[0] = info->afRGB[0];
			afRGB[1] = info->afRGB[1];
			afRGB[2] = info->afRGB[2];
			bValid = info->bValid;
			if (info->unUserDataLength == unUserDataLength)
				(pdUserData, info->pdUserData, unUserDataLength * sizeof(double));
		}
		;

		/** intensity, i.e. a grey value caused by backlight illumination */
		float fIntensity;
		/** amplitude, i.e. a grey value for an active sensor like TOF cameras */
		float fAmplitude;
		/** the pixels accuracy */
		double dAccuracy;
		/** Red/Green/Blue value */
		float afRGB[3];
		/** flag indicating a valid measurement (this depends on the application's requirements) */
		bool bValid;
		/** user defined data **/
		double* pdUserData;
		/** length of user data **/
		unsigned int unUserDataLength;
	};

	/**
	 * @struct StrCartesianPoint2D
	 * @brief Represents a single point in 2D cartesian space
	 * @author Stefan May
	 **/
	struct StrCartesianPoint2D
	{
		/** Default constructor **/
		StrCartesianPoint2D()
		{
		}
		;

		/** Copy constructor **/
		StrCartesianPoint2D(StrCartesianPoint2D* point)
		{
			dX = point->dX;
			dY = point->dY;
		}
		;

		/** copy raw data to array **/
		void getRawData(double* dBuffer)
		{
			dBuffer[0] = dX;
			dBuffer[1] = dY;
			//dBuffer[2] = dZ;
		}
		;

		/** x coordinate */
		double dX;
		/** y coordinate */
		double dY;
	};

	/**
	 * @struct StrCartesianPoint3D
	 * @brief Represents a single point in 3D cartesian space
	 * @author Stefan May
	 **/
	struct StrCartesianPoint3D
	{
		/** Default constructor **/
		StrCartesianPoint3D()
		{
		}
		;

		/** Copy constructor **/
		StrCartesianPoint3D(StrCartesianPoint3D* point)
		{
			dX = point->dX;
			dY = point->dY;
			dZ = point->dZ;
		}
		;

		/** copy raw data to array **/
		void getRawData(double* dBuffer)
		{
			dBuffer[0] = dX;
			dBuffer[1] = dY;
			dBuffer[2] = dZ;
		}
		;

		/** x coordinate */
		double dX;
		/** y coordinate */
		double dY;
		/** z coordinate */
		double dZ;
	};

	/**
	 * @struct TdCartesianPoint
	 * @brief Represents a single point in cartesian space
	 * @author Stefan May and Dirk Holz
	 **/
	typedef double* TdCartesianPoint;

	/**
	 * @struct StrCartesianIndexPair
	 * @brief Representation of one pair of point indices
	 * @author Stefan May
	 */
	struct StrCartesianIndexPair
	{
		/** index of first point */
		unsigned int indexFirst;
		/** index of second point */
		unsigned int indexSecond;
	};

	/**
	 * @struct StrCartesianPair
	 * @brief Represents a point pair in cartesian space
	 * @author Stefan May and Dirk Holz
	 **/
	struct StrCartesianPair
	{
		/** first point's coordinates */
		TdCartesianPoint first;
		/** second point's coordinates */
		TdCartesianPoint second;
	};

}

#endif /* CARTESIANBASE_H */
