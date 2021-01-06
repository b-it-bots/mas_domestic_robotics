/*!
 * @copyright 2020 Bonn-Rhein-Sieg University
 *
 * @author Sushant Vijay Chavan
 *
 */
#include <mdr_cloud_object_detection/CropBoxVoxelFilter.h>

using namespace mdr_cloud_object_detection;

void CropBoxVoxelFilter::setParams(const CropBoxVoxelFilterParams& pParams)
{
    /* crop-box params */
    mCropboxFilter.setMin(Eigen::Vector4f(pParams.mCropBoxLimitMinX,
                                          pParams.mCropBoxLimitMinY,
                                          pParams.mCropBoxLimitMinZ,
                                          1.0));
    mCropboxFilter.setMax(Eigen::Vector4f(pParams.mCropBoxLimitMaxX,
                                          pParams.mCropBoxLimitMaxY,
                                          pParams.mCropBoxLimitMaxZ,
                                          1.0));

    /* filter z-axis using voxel filter instead of making another member */
    mVoxelGridFilter.setFilterFieldName("z");
    mVoxelGridFilter.setFilterLimits(pParams.mVoxelLimitMinZ, pParams.mVoxelLimitMaxZ);

    /* voxel-grid params */
    mVoxelGridFilter.setLeafSize(pParams.mVoxelLeafSize, pParams.mVoxelLeafSize, pParams.mVoxelLeafSize);
}

PointCloud::Ptr CropBoxVoxelFilter::filterCloud(const PointCloud::ConstPtr &pCloudPtr)
{
    PointCloud::Ptr filteredCloudPtr = boost::make_shared<PointCloud>(); 

    mCropboxFilter.setInputCloud(pCloudPtr);
    mCropboxFilter.filter(*filteredCloudPtr);

    mVoxelGridFilter.setInputCloud(filteredCloudPtr);
    mVoxelGridFilter.filter(*filteredCloudPtr);

    return filteredCloudPtr;
}
