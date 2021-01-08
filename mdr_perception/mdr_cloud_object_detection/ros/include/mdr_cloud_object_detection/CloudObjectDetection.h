/*!
 * @copyright 2020 Bonn-Rhein-Sieg University
 *
 * @author Sushant Vijay Chavan
 *
 * @brief Class to detect objects from a point cloud
 */
#ifndef MDR_CLOUD_OBJECT_DETECTION_H
#define MDR_CLOUD_OBJECT_DETECTION_H

#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/server.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>

#include <mas_navigation_tools/OccupancyCheckerAction.h>
#include <mas_perception_msgs/ObjectList.h>

#include <mdr_cloud_object_detection/Utils.h>
#include <mdr_cloud_object_detection/CropBoxVoxelFilter.h>
#include <mdr_cloud_object_detection/ObjectDetectionConfig.h>

namespace mdr_cloud_object_detection
{
    class CloudObjectDetection
    {
    protected:
        ros::NodeHandle mNodeHandle;
        actionlib::SimpleActionClient<mas_navigation_tools::OccupancyCheckerAction> mOccupancyCheckerClient;
        dynamic_reconfigure::Server<ObjectDetectionConfig> mObjectDetectionConfigServer;
        ros::Subscriber mCloudInSub;
        ros::Subscriber mResetObjectCacheSub;
        ros::Publisher mFilteredCloudPub;
        ros::Publisher mObjectCloudPub;
        ros::Publisher mObjectBoundsPub;
        ros::Publisher mObjectListPub;
        tf::TransformListener mTfListener;
        std::string mTransformTargetFrame;
        std::string mClusterTargetFrame;
        CropBoxVoxelFilter mCloudFilter;
        EuclideanClusterParams mClusterParams;
        ObjectFilterParams mObjectFilterParams;
        ObjectCacheParams mObjectCacheParams;
        bool mPublishOrientedBBox;

        unsigned int mUniqueObjectId;
        unsigned int mCurrTime;
        std::map<int, int> mLastSeenTimeCache;
        std::map<int, const PointCloud::Ptr> mObjectsCache;
        std::map<int, std::vector<Eigen::Vector4f>> mPrevPositionsCache;
        std::map<int, mas_perception_msgs::Object> mObjectMsgCache;

    public:
        CloudObjectDetection(const ros::NodeHandle &pNodeHandle,
                             const std::string &pCloudInTopic,
                             const std::string &pFilteredCloudTopic,
                             const std::string &pObjectCloudTopic,
                             const std::string &pTransformTargetFrame,
                             const std::string &pClusterTargetFrame,
                             const std::string &pOccupancyCheckerActionName,
                             const std::string &pObjectsBoundsTopic,
                             const std::string &pObjectListTopic,
                             bool pPublishOrientedBBox);

    protected:
        void objectDetectionConfigCallback(const ObjectDetectionConfig &pConfig,
                                           uint32_t pLevel);

        void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pCloudMsgPtr);

        void resetObjectCacheCallback(const std_msgs::Bool& reset);

        PointCloud::Ptr transformPointCloud(const PointCloud& cloudIn,
                                            const std::string& targetFrame);

        void getClusterClouds(std::vector<PointCloud::Ptr>& clusterClouds,
                              PointCloud::ConstPtr filteredCloud,
                              const std::vector<pcl::PointIndices>& cluster_indices);

        float nearestDistance(const pcl::search::KdTree<PointT>& tree,
                              const PointT& pt);
        
        float getCloudSimilarity(const PointCloud& cloudA,
                                 const PointCloud& cloudB,
                                 float threshold);

        bool isNewObject(const PointCloud& cloud,
                         int& knownObjectId);

        void processNewClusters(const std::vector<PointCloud::Ptr>& clusterClouds);

        void filterClusterCloudsBySize(std::vector<PointCloud::Ptr>& clusterClouds);

        void filterClusterCloudsNearOccupiedSpaces(std::vector<PointCloud::Ptr>& clusterClouds);

        void removeStaleObjects();

        mas_perception_msgs::Object createObjectMessage(PointCloud::ConstPtr cloudPtr,
                                                        int id);

        void updateObjectMsgCache();

        const mas_perception_msgs::Object* getObjectMessage(int id);

        geometry_msgs::Point getPointMsg(float x,
                                         float y,
                                         float z);

        visualization_msgs::Marker getObjectBoundsMarker(const Eigen::Vector4f& min,
                                                         const Eigen::Vector4f& max,
                                                         int id);

        visualization_msgs::Marker getObjectBoundsMarker(const mas_perception_msgs::Object& objMsg,
                                                         int id);

        visualization_msgs::Marker getObjectOrientationMarker(const mas_perception_msgs::Object& objMsg,
                                                              int id);

        visualization_msgs::Marker getObjectIdMarker(const Eigen::Vector4f& min,
                                                     const Eigen::Vector4f& max,
                                                     int id);

        void publishCloud(PointCloud::ConstPtr cloudPtr,
                          const ros::Publisher& publisher);

        void publishObjectCloud();

        void publishObjectBoundsMarkers();

        void publishObjectListMessages();

        void cleanup();
    };

}   // namespace mdr_cloud_object_detection


#endif  // MDR_CLOUD_OBJECT_DETECTION_H
