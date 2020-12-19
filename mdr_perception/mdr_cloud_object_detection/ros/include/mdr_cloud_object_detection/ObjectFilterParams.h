/*!
 * @copyright 2020 Bonn-Rhein-Sieg University
 *
 * @author Sushant Vijay Chavan
 *
 */
#ifndef MDR_CLOUD_OBJECT_DETECTION_OBJECT_FILTERING_PARAMS
#define MDR_CLOUD_OBJECT_DETECTION_OBJECT_FILTERING_PARAMS

namespace mdr_cloud_object_detection
{
    /*!
    * @brief struct containing parameters necessary for filtering the detected objects
    */
    struct ObjectFilterParams
    {
        float mMaxBboxEdgeLength = 0.15f;
        float mMinDistFromOccupiedCell = 0.1f;
    };
}

#endif // MDR_CLOUD_OBJECT_DETECTION_OBJECT_FILTERING_PARAMS
