/*!
 * @copyright 2020 Bonn-Rhein-Sieg University
 *
 * @author Sushant Vijay Chavan
 *
 */
#ifndef MDR_CLOUD_OBJECT_DETECTION_CROP_BOX_VOXEL_FILTER
#define MDR_CLOUD_OBJECT_DETECTION_CROP_BOX_VOXEL_FILTER

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

#include <mas_perception_libs/aliases.h>
#include <mdr_cloud_object_detection/CropBoxVoxelFilterParams.h>

namespace mdr_cloud_object_detection
{
    /*!
    * @brief class containing definition for filtering point clouds
    */
    class CropBoxVoxelFilter
    {
    public:
        CropBoxVoxelFilter() = default;

        /*! @brief set parameters relevant to filtering cloud */
        virtual void setParams(const CropBoxVoxelFilterParams& pParams);

        /*!
        * @brief filter point cloud using crop-box and voxel filters
        */
        PointCloud::Ptr filterCloud(const PointCloud::ConstPtr &pCloudPtr);

    private:
        pcl::CropBox<PointT> mCropboxFilter;
        pcl::VoxelGrid<PointT> mVoxelGridFilter;
    };
}

#endif // MDR_CLOUD_OBJECT_DETECTION_CROP_BOX_VOXEL_FILTER
