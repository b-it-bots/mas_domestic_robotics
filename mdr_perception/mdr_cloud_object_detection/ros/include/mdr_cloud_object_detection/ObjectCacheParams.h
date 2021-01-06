/*!
 * @copyright 2020 Bonn-Rhein-Sieg University
 *
 * @author Sushant Vijay Chavan
 *
 */
#ifndef MDR_CLOUD_OBJECT_DETECTION_OBJECT_CACHE_PARAMS
#define MDR_CLOUD_OBJECT_DETECTION_OBJECT_CACHE_PARAMS

namespace mdr_cloud_object_detection
{
    /*!
    * @brief struct containing parameters necessary for caching the detected objects
    */
    struct ObjectCacheParams
    {
        float mObjectCacheTime = 5.0f;
        float mSimilarityThreshold = 2.0f;
        int mPositionHistoryCacheSize = 10;
        float mUniquenessThreshold = 0.005f;
    };
}

#endif // MDR_CLOUD_OBJECT_DETECTION_OBJECT_CACHE_PARAMS
