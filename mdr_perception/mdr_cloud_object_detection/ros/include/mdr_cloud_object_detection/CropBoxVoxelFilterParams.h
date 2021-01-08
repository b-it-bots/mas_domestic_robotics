/*!
 * @copyright 2020 Bonn-Rhein-Sieg University
 *
 * @author Sushant Vijay Chavan
 *
 */
#ifndef MDR_CLOUD_OBJECT_DETECTION_CROP_BOX_VOXEL_FILTER_PARAMS
#define MDR_CLOUD_OBJECT_DETECTION_CROP_BOX_VOXEL_FILTER_PARAMS

namespace mdr_cloud_object_detection
{
    /*!
    * @brief struct containing parameters necessary for filtering point clouds
    */
    struct CropBoxVoxelFilterParams
    {
        /* CropBox filter parameters
        * limit the cloud to be filtered points outside of these x, y, z ranges will be discarded */
        float mCropBoxLimitMinX = 0.0f;
        float mCropBoxLimitMaxX = 0.0f;
        float mCropBoxLimitMinY = 0.0f;
        float mCropBoxLimitMaxY = 0.0f;
        float mCropBoxLimitMinZ = 0.0f;
        float mCropBoxLimitMaxZ = 0.0f;
        /* VoxelGrid filter parameters for down-sampling the cloud, also limit the cloud along the z axis */
        float mVoxelLimitMinZ = 0.0f;
        float mVoxelLimitMaxZ = 0.0f;
        float mVoxelLeafSize = 0.0f;
    };
}

#endif // MDR_CLOUD_OBJECT_DETECTION_CROP_BOX_VOXEL_FILTER_PARAMS
