/*!
 * @copyright 2020 Bonn-Rhein-Sieg University
 *
 * @author Sushant Vijay Chavan
 *
 */
#ifndef MDR_CLOUD_OBJECT_DETECTION_PASS_THROUGH_VOXEL_FILTER
#define MDR_CLOUD_OBJECT_DETECTION_PASS_THROUGH_VOXEL_FILTER

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <mdr_cloud_object_detection/Aliases.h>
#include <mdr_cloud_object_detection/PassThroughVoxelFilterParams.h>

namespace mdr_cloud_object_detection
{
    /*!
    * @brief class containing definition for filtering point clouds
    */
    class PassThroughVoxelFilter
    {
    public:
        PassThroughVoxelFilter() = default;

        /*! @brief set parameters relevant to filtering cloud */
        virtual void setParams(const PassThroughVoxelFilterParams& pParams);

        /*!
        * @brief filter point cloud using passthrough and voxel filters
        */
        PointCloud::Ptr filterCloud(const PointCloud::ConstPtr &pCloudPtr);

    private:
        pcl::PassThrough<PointT> mPassThroughFilterX;
        pcl::PassThrough<PointT> mPassThroughFilterY;
        pcl::PassThrough<PointT> mPassThroughFilterZ;
        pcl::VoxelGrid<PointT> mVoxelGridFilter;
    };
}

#endif // MDR_CLOUD_OBJECT_DETECTION_PASS_THROUGH_VOXEL_FILTER
