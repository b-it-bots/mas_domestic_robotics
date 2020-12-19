/*!
 * @copyright 2018 Bonn-Rhein-Sieg University
 *
 * @author Sushant Vijay Chavan
 *
 * @brief Class to detect objects from a point cloud
 */

#include <cmath>
#include <actionlib/client/simple_action_client.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <mas_navigation_tools/OccupancyCheckerAction.h>
#include <mas_perception_msgs/ObjectList.h>
#include "mdr_cloud_object_detection/Color.h"
#include "mdr_cloud_object_detection/Utils.h"
#include <mdr_cloud_object_detection/ObjectDetectionConfig.h>

namespace mdr_cloud_object_detection
{
    class CloudObjectDetection
    {
    private:
        ros::NodeHandle mNodeHandle;
        dynamic_reconfigure::Server<ObjectDetectionConfig> mObjectDetectionConfigServer;
        ros::Subscriber mCloudSub;
        ros::Subscriber mResetSub;
        ros::Publisher mFilteredCloudPub;
        ros::Publisher mObjectCloudPub;
        ros::Publisher mObjectBoundsPub;
        ros::Publisher mObjectObjectsPub;
        actionlib::SimpleActionClient<mas_navigation_tools::OccupancyCheckerAction> mOccupancyCheckerClient;
        tf::TransformListener mTfListener;
        std::string mTransformTargetFrame;
        std::string mClusterTargetFrame;
        PassThroughVoxelFilter mCloudFilter;
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
                             const std::string &pCloudTopic,
                             const std::string &pFilteredCloudTopic,
                             const std::string &pObjectCloudTopic,
                             const std::string &pTransformTargetFrame,
                             const std::string &pClusterTargetFrame,
                             const std::string &pOccupancyCheckerActionName,
                             const std::string &pObjectsBoundsTopic,
                             const std::string &pObjectObjectsTopic,
                             bool pPublishOrientedBBox);

    private:
        void objectDetectionConfigCallback(const ObjectDetectionConfig &pConfig,
                                           uint32_t pLevel);

        void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pCloudMsgPtr);

        void resetCallback(const std_msgs::Bool& reset);

        void filterClusterCloudsBySize(std::vector<PointCloud::Ptr>& clusterClouds);

        void filterClusterCloudsNearOccupiedSpaces(std::vector<PointCloud::Ptr>& clusterClouds);

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

        void removeStaleObjects();

        geometry_msgs::Point getGeomPoint(float x,
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

        mas_perception_msgs::Object createObjectMessage(PointCloud::ConstPtr cloudPtr,
                                                        int id);

        void updateObjectMsgCache();

        const mas_perception_msgs::Object* getObjectMessage(int id);

        void publishCloud(PointCloud::ConstPtr cloudPtr,
                          const ros::Publisher& publisher);

        void publishObjectCloud();

        void publishObjectBoundsMarkers();

        void publishObjectObjectMessages();

        void cleanup();
    };

}   // namespace mdr_cloud_object_detection
