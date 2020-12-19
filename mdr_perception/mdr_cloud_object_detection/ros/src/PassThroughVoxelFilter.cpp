#include "mdr_cloud_object_detection/PassThroughVoxelFilter.h"

using namespace mdr_cloud_object_detection;

void PassThroughVoxelFilter::setParams(const PassThroughVoxelFilterParams& pParams)
{
    /* pass-through params */
    mPassThroughFilterX.setFilterFieldName("x");
    mPassThroughFilterX.setFilterLimits(pParams.mPassThroughLimitMinX, pParams.mPassThroughLimitMaxX);
    mPassThroughFilterY.setFilterFieldName("y");
    mPassThroughFilterY.setFilterLimits(pParams.mPassThroughLimitMinY, pParams.mPassThroughLimitMaxY);
    mPassThroughFilterZ.setFilterFieldName("z");
    mPassThroughFilterZ.setFilterLimits(pParams.mPassThroughLimitMinZ, pParams.mPassThroughLimitMaxZ);

    /* filter z-axis using voxel filter instead of making another member */
    mVoxelGridFilter.setFilterFieldName("z");
    mVoxelGridFilter.setFilterLimits(pParams.mVoxelLimitMinZ, pParams.mVoxelLimitMaxZ);

    /* voxel-grid params */
    mVoxelGridFilter.setLeafSize(pParams.mVoxelLeafSize, pParams.mVoxelLeafSize, pParams.mVoxelLeafSize);
}

PointCloud::Ptr PassThroughVoxelFilter::filterCloud(const PointCloud::ConstPtr &pCloudPtr)
{
    PointCloud::Ptr filteredCloudPtr = boost::make_shared<PointCloud>();

    mPassThroughFilterX.setInputCloud(pCloudPtr);
    mPassThroughFilterX.filter(*filteredCloudPtr);

    mPassThroughFilterY.setInputCloud(filteredCloudPtr);
    mPassThroughFilterY.filter(*filteredCloudPtr);

    mPassThroughFilterZ.setInputCloud(filteredCloudPtr);
    mPassThroughFilterZ.filter(*filteredCloudPtr);

    mVoxelGridFilter.setInputCloud(filteredCloudPtr);
    mVoxelGridFilter.filter(*filteredCloudPtr);

    return filteredCloudPtr;
}
