/*!
 * @copyright 2020 Bonn-Rhein-Sieg University
 *
 * @author Sushant Vijay Chavan
 *
 */
#ifndef MDR_CLOUD_OBJECT_DETECTION_PASS_THROUGH_VOXEL_FILTER_PARAMS
#define MDR_CLOUD_OBJECT_DETECTION_PASS_THROUGH_VOXEL_FILTER_PARAMS

namespace mdr_cloud_object_detection
{
    /*!
    * @brief struct containing parameters necessary for filtering point clouds
    */
    struct PassThroughVoxelFilterParams
    {
        /* PassThrough filter parameters
        * limit the cloud to be filtered points outside of these x, y, z ranges will be discarded */
        float mPassThroughLimitMinX = 0.0f;
        float mPassThroughLimitMaxX = 0.0f;
        float mPassThroughLimitMinY = 0.0f;
        float mPassThroughLimitMaxY = 0.0f;
        float mPassThroughLimitMinZ = 0.0f;
        float mPassThroughLimitMaxZ = 0.0f;
        /* VoxelGrid filter parameters for down-sampling the cloud, also limit the cloud along the z axis */
        float mVoxelLimitMinZ = 0.0f;
        float mVoxelLimitMaxZ = 0.0f;
        float mVoxelLeafSize = 0.0f;
    };
}

#endif // MDR_CLOUD_OBJECT_DETECTION_PASS_THROUGH_VOXEL_FILTER_PARAMS
