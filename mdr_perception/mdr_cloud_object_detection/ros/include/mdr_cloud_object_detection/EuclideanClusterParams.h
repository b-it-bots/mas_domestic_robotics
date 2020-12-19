/*!
 * @copyright 2020 Bonn-Rhein-Sieg University
 *
 * @author Sushant Vijay Chavan
 *
 */
#ifndef MDR_CLOUD_OBJECT_DETECTION_EUCLIDEAN_CLUSTER_PARAMS
#define MDR_CLOUD_OBJECT_DETECTION_EUCLIDEAN_CLUSTER_PARAMS

#include <limits>

namespace mdr_cloud_object_detection
{
    /*!
    * @brief struct containing parameters necessary for clustering point clouds
    */
    struct EuclideanClusterParams
    {
        float mClusterTolerance = 0.01f;
        unsigned int mMinClusterSize = 1;
        unsigned int mMaxClusterSize = std::numeric_limits<unsigned int>::max();
    };
}

#endif // MDR_CLOUD_OBJECT_DETECTION_EUCLIDEAN_CLUSTER_PARAMS
