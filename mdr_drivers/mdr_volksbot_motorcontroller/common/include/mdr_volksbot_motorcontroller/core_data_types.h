#ifndef DATA_TYPES_
#define DATA_TYPES_

#include "cartesian_base.h"

/**
 * @struct StrVertex
 * @brief Representens a vertex in 3D space
 * @author dirk holz
 */
struct StrVertex
{
	StrVertex()
			: dX(0),
			  dY(0),
			  dZ(0)
	{
	}
	;
	double dX;
	double dY;
	double dZ;
};

struct StrCellIndexPair
{
	StrCellIndexPair()
			: nIndexX(0),
			  nIndexY(0)
	{
	}
	;
	StrCellIndexPair(int nX, int nY)
			: nIndexX(nX),
			  nIndexY(nY)
	{
	}
	;
	int nIndexX;
	int nIndexY;
};

/**
 * @struct StrPose
 * @brief Represents the pose of an objects or robot in cartesian space.
 * @author Dirk Holz, February 2008
 */
struct StrPose
{
	/** Constructor. Initializes all values to 0. */
	StrPose()
			: dX(0),
			  dY(0),
			  dZ(0),
			  dRoll(0),
			  dPitch(0),
			  dYaw(0)
	{
	}
	;
	/** Position of object -- Position on X-axis */
	double dX;
	/** Position of object -- Position on Y-axis */
	double dY;
	/** Position of object -- Position on Z-axis */
	double dZ;
	/** Orientation of point -- Rotation around X-axis */
	double dRoll;
	/** Orientation of point -- Rotation around Y-axis */
	double dPitch;
	/** Orientation of point -- Rotation around Z-axis */
	double dYaw;
};

typedef StrPose StrRobotPose;

/**
 * @struct StrPoint
 * @brief Represents a single point in polar and(!) cartesian space
 * @author Dirk Holz, February 2008
 */
struct StrPoint
{
	/** Constructor. Initializes all values to 0. */
	StrPoint()
			: dDistance(0),
			  dRoll(0),
			  dPitch(0),
			  dYaw(0),
			  dX(0),
			  dY(0),
			  dZ(0)
	{
	}
	;
	/** Distance to point */
	double dDistance;
	/** Orientation of point -- Rotation around X-axis */
	double dRoll;
	/** Orientation of point -- Rotation around Y-axis */
	double dPitch;
	/** Orientation of point -- Rotation around Z-axis */
	double dYaw;
	/** Position of point -- Position on X-axis */
	double dX;
	/** Position of point -- Position on Y-axis */
	double dY;
	/** Position of point -- Position on Z-axis */
	double dZ;
};

struct StrEdge
{
	fair::StrCartesianPoint3D strPointStart;
	fair::StrCartesianPoint3D strPointEnd;
};

/**
 * @struct StrDoorPose
 * @brief represents the left, right and center points of a door
 * @author Frederik Hegger/Christian Mueller, April 2009
 */
struct StrDoorPose
{
	/* left pose of the door */
	StrPose strLeftDoorPose;
	/* right pose of the door */
	StrPose strRightDoorPose;
	/* center pose of the door */
	StrPose strCenterDoorPose;
};

/**
 * @struct StrLaserScanSegment
 * @brief represents the start and end point a laser scan segment
 * @author Frederik Hegger June 2009
 */
struct StrLaserScanSegment
{
	StrLaserScanSegment()
			: unStartPosition(0),
			  unEndPosition(0),
			  unNumberOfFeatures(13),
			  unNumberOfPoints(0),
			  dStandardDeviation(0),
			  dMeanAverageDeviationFromMedian(0),
			  dJumpDistanceFromPrecedingSegment(0),
			  dJumpDistanceToSucceedingSegment(0),
			  dWidth(0),
			  dLinearity(0),
			  dCircularity(0),
			  dRadius(0),
			  dBoundaryLength(0),
			  dBoundaryRegularity(0),
			  dMeanCurvature(0),
			  dDistanceToMedian(0)
	{
	}
	;
	/* first point of the laser scan segment */
	StrPoint strStartPoint;
	/* last point of the laser scan segment  */
	StrPoint strEndPoint;
	/* center point of the laser scan segment  */
	StrPoint strCenterPoint;
	/* first position of segement in the laser scan array */
	unsigned int unStartPosition;
	/* last position of segment in the laser scan array */
	unsigned int unEndPosition;
	/* number of features */
	unsigned int unNumberOfFeatures;
	/* number of points in the laser scan segment, including start and end point */
	unsigned int unNumberOfPoints;
	/* standard deviation */
	double dStandardDeviation;
	/* mean average deviation from median*/
	double dMeanAverageDeviationFromMedian;
	/* jump distance from preceding segment */
	double dJumpDistanceFromPrecedingSegment;
	/* jump distance from succeeding segment */
	double dJumpDistanceToSucceedingSegment;
	/* euclidean distance between first point and last point of the segemt */
	double dWidth;
	/* straightness of the segment */
	double dLinearity;
	/* circularity of the segment */
	double dCircularity;
	/* radius of the circle fitted to the segment */
	double dRadius;
	/* length of the poly-line correspond to the segment */
	double dBoundaryLength;
	/* standard deviation of distances of adjacent points */
	double dBoundaryRegularity;
	/* average curvature */
	double dMeanCurvature;
	/* distance to median */
	double dDistanceToMedian;
};

template<typename T>
class CHistogram
{
 public:
	CHistogram(unsigned int unSize)
	{
		this->size = unSize;
		values = new T(this->size);
	}
	;
	~CHistogram()
	{
		delete values;
	}
	;
	T* values;
	unsigned int size;
	unsigned int index;
	void add(T newValue)
	{
		values[index] = newValue;
		if (++index >= size)
			index = 0;
	}
	;
	T* getValues()
	{
		return values;
	}
	;
	double getSum()
	{
		double dSum = 0;
		for (int i = 0; i < size; ++i)
			dSum += values[i];
		return dSum;
	}
	double getMean()
	{
		return (this->getSum / (double) size);
	}
};

#endif /*DATA_TYPES_*/
