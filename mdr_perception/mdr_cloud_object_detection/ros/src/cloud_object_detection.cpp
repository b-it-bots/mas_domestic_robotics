/*!
 * @copyright 2018 Bonn-Rhein-Sieg University
 *
 * @author Sushant Chavan
 *
 * @brief script to detect objects from a point cloud
 */

#include <cmath>
#include <actionlib/client/simple_action_client.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <mas_navigation_tools/OccupancyCheckerAction.h>
#include <mas_perception_msgs/ObjectList.h>
#include <mdr_cloud_object_detection/color.h>
#include <mdr_cloud_object_detection/ObjectDetectionConfig.h>

// using namespace mdr_cloud_object_detection;

namespace mdr_cloud_object_detection
{

/*!
 * @brief struct containing parameters necessary for filtering point clouds
 */
struct CloudPassThroughVoxelFilterParams
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

/*!
 * @brief struct containing parameters necessary for clustering point clouds
 */
struct EuclideanClusterParams
{
    float mClusterTolerance = 0.01f;
    unsigned int mMinClusterSize = 1;
    unsigned int mMaxClusterSize = std::numeric_limits<unsigned int>::max();
};

/*!
 * @brief struct containing parameters necessary for filtering the detected objects
 */
struct ObjectFilterParams
{
    float mMaxBboxEdgeLength = 0.15f;
    float mMinDistFromOccupiedCell = 0.1f;
};

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

/*!
 * @brief class containing definition for filtering point clouds
 */
class CloudPassThroughVoxelFilter
{
public:
    CloudPassThroughVoxelFilter() = default;

    /*! @brief set parameters relevant to filtering cloud */
    virtual void
    setParams(const CloudPassThroughVoxelFilterParams& pParams)
    {
        /* pass-through params */
        mPassThroughFilterX.setFilterFieldName("x");
        mPassThroughFilterX.setFilterLimits(pParams.mPassThroughLimitMinX, pParams.mPassThroughLimitMaxX);
        mPassThroughFilterY.setFilterFieldName("y");
        mPassThroughFilterY.setFilterLimits(pParams.mPassThroughLimitMinY, pParams.mPassThroughLimitMaxY);
        mPassThroughFilterZ.setFilterFieldName("z");
        mPassThroughFilterZ.setFilterLimits(pParams.mPassThroughLimitMinZ, pParams.mPassThroughLimitMaxZ);

        /* filter z-axis using voxel filter instead of making another member */
        mVoxelGridFilter.setFilterFieldName("z");
        mVoxelGridFilter.setFilterLimits(pParams.mVoxelLimitMinZ, pParams.mVoxelLimitMaxZ);

        /* voxel-grid params */
        mVoxelGridFilter.setLeafSize(pParams.mVoxelLeafSize, pParams.mVoxelLeafSize, pParams.mVoxelLeafSize);
    }

    /*!
    * @brief filter point cloud using passthrough and voxel filters
    */
    PointCloud::Ptr
    filterCloud(const PointCloud::ConstPtr &pCloudPtr)
    {
        PointCloud::Ptr filteredCloudPtr = boost::make_shared<PointCloud>();

        mPassThroughFilterX.setInputCloud(pCloudPtr);
        mPassThroughFilterX.filter(*filteredCloudPtr);

        mPassThroughFilterY.setInputCloud(filteredCloudPtr);
        mPassThroughFilterY.filter(*filteredCloudPtr);

        mPassThroughFilterZ.setInputCloud(filteredCloudPtr);
        mPassThroughFilterZ.filter(*filteredCloudPtr);

        mVoxelGridFilter.setInputCloud(filteredCloudPtr);
        mVoxelGridFilter.filter(*filteredCloudPtr);

        return filteredCloudPtr;
    }

private:
    pcl::PassThrough<PointT> mPassThroughFilterX;
    pcl::PassThrough<PointT> mPassThroughFilterY;
    pcl::PassThrough<PointT> mPassThroughFilterZ;
    pcl::VoxelGrid<PointT> mVoxelGridFilter;
};

class CloudObjectDetectionNode
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
    CloudPassThroughVoxelFilter mCloudFilter;
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
    CloudObjectDetectionNode(const ros::NodeHandle &pNodeHandle, const std::string &pCloudTopic,
            const std::string &pFilteredCloudTopic,
            const std::string &pObjectCloudTopic,
            const std::string &pTransformTargetFrame,
            const std::string &pClusterTargetFrame,
            const std::string &pOccupancyCheckerActionName,
            const std::string &pObjectsBoundsTopic,
            const std::string &pObjectObjectsTopic,
            bool pPublishOrientedBBox)
    : mNodeHandle(pNodeHandle), mObjectDetectionConfigServer(mNodeHandle),
      mTransformTargetFrame(pTransformTargetFrame), mClusterTargetFrame(pClusterTargetFrame),
      mPublishOrientedBBox(pPublishOrientedBBox), mUniqueObjectId(0),
      mOccupancyCheckerClient(pOccupancyCheckerActionName, true)
    {
        ROS_INFO("[ObjectDetectionNode] Waiting for OccupancyChecker server");
        mOccupancyCheckerClient.waitForServer();

        ROS_INFO("setting up dynamic reconfiguration server for object detection");
        auto odCallback = boost::bind(&CloudObjectDetectionNode::objectDetectionConfigCallback, this, _1, _2);
        mObjectDetectionConfigServer.setCallback(odCallback);

        ROS_INFO("subscribing to point cloud topic and advertising processed result");
        mCloudSub = mNodeHandle.subscribe(pCloudTopic, 1, &CloudObjectDetectionNode::cloudCallback, this);
        mResetSub = mNodeHandle.subscribe("reset_cache", 1, &CloudObjectDetectionNode::resetCallback, this);
        mFilteredCloudPub = mNodeHandle.advertise<sensor_msgs::PointCloud2>(pFilteredCloudTopic, 1);
        mObjectCloudPub = mNodeHandle.advertise<sensor_msgs::PointCloud2>(pObjectCloudTopic, 1);
        mObjectBoundsPub = mNodeHandle.advertise<visualization_msgs::MarkerArray>(pObjectsBoundsTopic, 1);
        mObjectObjectsPub = mNodeHandle.advertise<mas_perception_msgs::ObjectList>(pObjectObjectsTopic, 1);
    }

private:
    void
    objectDetectionConfigCallback(const ObjectDetectionConfig &pConfig, uint32_t pLevel)
    {
        // Cloud Filter params
        CloudPassThroughVoxelFilterParams cloudFilterParams;
        cloudFilterParams.mPassThroughLimitMinX = static_cast<float>(pConfig.passthrough_limit_min_x);
        cloudFilterParams.mPassThroughLimitMaxX = static_cast<float>(pConfig.passthrough_limit_max_x);
        cloudFilterParams.mPassThroughLimitMinY = static_cast<float>(pConfig.passthrough_limit_min_y);
        cloudFilterParams.mPassThroughLimitMaxY = static_cast<float>(pConfig.passthrough_limit_max_y);
        cloudFilterParams.mPassThroughLimitMinZ = static_cast<float>(pConfig.passthrough_limit_min_z);
        cloudFilterParams.mPassThroughLimitMaxZ = static_cast<float>(pConfig.passthrough_limit_max_z);
        cloudFilterParams.mVoxelLimitMinZ = static_cast<float>(pConfig.voxel_limit_min_z);
        cloudFilterParams.mVoxelLimitMaxZ = static_cast<float>(pConfig.voxel_limit_max_z);
        cloudFilterParams.mVoxelLeafSize = static_cast<float>(pConfig.voxel_leaf_size);
        mCloudFilter.setParams(cloudFilterParams);

        // Cloud Clustering params
        mClusterParams.mClusterTolerance = static_cast<float>(pConfig.cluster_tolerance);
        mClusterParams.mMinClusterSize = static_cast<unsigned int>(pConfig.min_cluster_size);
        mClusterParams.mMaxClusterSize = static_cast<unsigned int>(pConfig.max_cluster_size);

        // Object filter params
        mObjectFilterParams.mMaxBboxEdgeLength = static_cast<float>(pConfig.max_bbox_edge_length);
        mObjectFilterParams.mMinDistFromOccupiedCell = static_cast<float>(pConfig.min_dist_from_occupied_cell);

        // Object cache params
        mObjectCacheParams.mObjectCacheTime = static_cast<unsigned int>(pConfig.object_cache_time);
        mObjectCacheParams.mSimilarityThreshold = static_cast<float>(pConfig.similarity_threshold);
        mObjectCacheParams.mUniquenessThreshold = static_cast<float>(pConfig.uniqueness_threshold);
        mObjectCacheParams.mPositionHistoryCacheSize = static_cast<float>(pConfig.position_history_cache_size);
    }

    void
    cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pCloudMsgPtr)
    {
        // do not process cloud when there's no subscriber
        if (mFilteredCloudPub.getNumSubscribers() == 0 &&
            mObjectBoundsPub.getNumSubscribers() == 0 &&
            mObjectObjectsPub.getNumSubscribers() == 0 &&
            mObjectCloudPub.getNumSubscribers() == 0)
            return;

        if (!mTfListener.waitForTransform(mTransformTargetFrame, pCloudMsgPtr->header.frame_id.c_str(), pCloudMsgPtr->header.stamp, ros::Duration(1.0)) ||
            !mTfListener.waitForTransform(mClusterTargetFrame, mTransformTargetFrame, pCloudMsgPtr->header.stamp, ros::Duration(1.0)))
            return;

        // transform the cloud to a desired frame
        auto transformedCloudPtr = boost::make_shared<sensor_msgs::PointCloud2>();
        if (!pcl_ros::transformPointCloud(mTransformTargetFrame, *pCloudMsgPtr, *transformedCloudPtr, mTfListener))
        {
            ROS_WARN("failed to transform cloud to frame '%s' from frame '%s'",
                     mTransformTargetFrame.c_str(), pCloudMsgPtr->header.frame_id.c_str());
            return;
        }

        // filter the cloud
        PointCloud::Ptr pclCloudPtr = boost::make_shared<PointCloud>();
        pcl::fromROSMsg(*transformedCloudPtr, *pclCloudPtr);
        PointCloud::Ptr filteredCloudPtr = mCloudFilter.filterCloud(pclCloudPtr);

        if (filteredCloudPtr->size() < mClusterParams.mMinClusterSize)
        {
            // Stop processing the point cloud if the filtered cloud does
            // not have enough points to create even one cluster
            return;
        }

        mCurrTime = pCloudMsgPtr->header.stamp.sec;

        if (mFilteredCloudPub.getNumSubscribers() > 0)
        {
            // publish the filtered cloud for debugging
            publishCloud(filteredCloudPtr, mFilteredCloudPub);
        }

        // Euclidean clustering
        pcl::search::Search<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(filteredCloudPtr);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::extractEuclideanClusters(*filteredCloudPtr,
                                      tree,
                                      mClusterParams.mClusterTolerance,
                                      cluster_indices,
                                      mClusterParams.mMinClusterSize,
                                      mClusterParams.mMaxClusterSize);

        if (cluster_indices.size() <= 0)
            return;

        std::vector<PointCloud::Ptr> clusterClouds;
        getClusterClouds(clusterClouds, filteredCloudPtr, cluster_indices);
        filterClusterCloudsBySize(clusterClouds);
        filterClusterCloudsNearOccupiedSpaces(clusterClouds);

        processNewClusters(clusterClouds);
        removeStaleObjects();

        publishObjectCloud();
        publishObjectBoundsMarkers();
        publishObjectObjectMessages();

        cleanup();
    }

    void
    resetCallback(const std_msgs::Bool& reset)
    {
        if (reset.data)
        {
            ROS_WARN("Cache reset requested! Resetting the object cache!");
            mUniqueObjectId = 0;
            mLastSeenTimeCache.clear();
            mObjectsCache.clear();
            mPrevPositionsCache.clear();
            mObjectMsgCache.clear();
        }
    }

    void
    filterClusterCloudsBySize(std::vector<PointCloud::Ptr>& clusterClouds)
    {
        // Check if the size of the object is within the specified limits
        for (std::vector<PointCloud::Ptr>::iterator it = clusterClouds.begin(); it != clusterClouds.end(); )
        {
            Eigen::Vector4f min, max;
            pcl::getMinMax3D(*(*it), min, max);
            float sizeX = std::abs(max[0] - min[0]);
            float sizeY = std::abs(max[1] - min[1]);
            float sizeZ = std::abs(max[2] - min[2]);
            if (sizeX > mObjectFilterParams.mMaxBboxEdgeLength ||
                sizeY > mObjectFilterParams.mMaxBboxEdgeLength ||
                sizeZ > mObjectFilterParams.mMaxBboxEdgeLength)
            {
                it = clusterClouds.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    void
    filterClusterCloudsNearOccupiedSpaces(std::vector<PointCloud::Ptr>& clusterClouds)
    {
        mas_navigation_tools::OccupancyCheckerGoal goal;
        for (std::vector<PointCloud::Ptr>::iterator it = clusterClouds.begin(); it != clusterClouds.end(); )
        {
            Eigen::Vector4f centroid;
            if (pcl::compute3DCentroid(*(*it), centroid) <= 0)
            {
                ROS_WARN("Could not compute centroid of point cloud. The cluster will be filtered out!");
                it = clusterClouds.erase(it);
            }
            else
            {
                geometry_msgs::Point p;
                p.x = centroid.x();
                p.y = centroid.y();
                p.z = centroid.z();
                goal.points.push_back(p);
                ++it;
            }
        }
        goal.search_radius = mObjectFilterParams.mMinDistFromOccupiedCell;
        mOccupancyCheckerClient.sendGoal(goal);
        mOccupancyCheckerClient.waitForResult(ros::Duration(5.0));
        if (mOccupancyCheckerClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            mas_navigation_tools::OccupancyCheckerResultConstPtr result = mOccupancyCheckerClient.getResult();
            int pos = 0;
            for (std::vector<PointCloud::Ptr>::iterator it = clusterClouds.begin(); it != clusterClouds.end(); )
            {
                if (!result->far_from_obstacle[pos++])
                    it = clusterClouds.erase(it);
                else
                    ++it;
            }
        }
        else
        {
            ROS_ERROR("Could not complete checking if the objects lie close to the occupied regions in the map. Discarding all the clusters!");
            clusterClouds.clear();
        }
    }

    void
    getClusterClouds(std::vector<PointCloud::Ptr>& clusterClouds,
                     PointCloud::ConstPtr filteredCloud,
                     const std::vector<pcl::PointIndices>& cluster_indices)
    {
        for (const auto& idx: cluster_indices)
        {
            PointCloud::Ptr clusterCloud(new PointCloud);
            for (const auto &index : idx.indices)
                clusterCloud->push_back ((*filteredCloud)[index]);

            clusterCloud->header = filteredCloud->header;
            clusterCloud->width = clusterCloud->size();
            clusterCloud->height = 1;
            clusterCloud->is_dense = true;

            PointCloud::Ptr transformedCloudPtr = boost::make_shared<PointCloud>();
            if (!pcl_ros::transformPointCloud(mClusterTargetFrame, *clusterCloud, *transformedCloudPtr, mTfListener))
            {
                ROS_WARN("Failed to transform cluster cloud to frame '%s' from frame '%s'. This cluster will be skipped in the current frame!",
                        mClusterTargetFrame.c_str(), clusterCloud->header.frame_id.c_str());
                continue;
            }

            clusterClouds.push_back(transformedCloudPtr);
        }
    }

    // Cloud comparison sample from https://stackoverflow.com/a/55930847
    float
    nearestDistance(const pcl::search::KdTree<PointT>& tree, const PointT& pt)
    {
        const int k = 1;
        std::vector<int> indices (k);
        std::vector<float> sqr_distances (k);

        tree.nearestKSearch(pt, k, indices, sqr_distances);

        return sqr_distances[0];
    }
    // compare cloudB to cloudA
    // use threshold for identifying outliers and not considering those for the similarity
    // a good value for threshold is 5 * <cloud_resolution>, e.g. 10cm for a cloud with 2cm resolution
    float
    getCloudSimilarity(const PointCloud& cloudA, const PointCloud& cloudB, float threshold)
    {
        // compare B to A
        int num_outlier = 0;
        pcl::search::KdTree<PointT> tree;
        tree.setInputCloud(cloudA.makeShared());
        auto sum = std::accumulate(cloudB.begin(), cloudB.end(), 0.0f, [&](float current_sum, const PointT& pt) {
            const auto dist = nearestDistance(tree, pt);

            if(dist < threshold)
            {
                return current_sum + dist;
            }
            else
            {
                num_outlier++;
                return current_sum;
            }
        });

        return sum / (cloudB.size() - num_outlier);
    }

    bool
    isNewObject(const PointCloud& cloud, int& knownObjectId)
    {
        for (const auto& o : mObjectsCache)
        {
            if (getCloudSimilarity(cloud, *(o.second), mObjectCacheParams.mSimilarityThreshold) < mObjectCacheParams.mUniquenessThreshold)
            {
                knownObjectId = o.first;
                return false;
            }
        }
        knownObjectId = -1;
        return true;
    }

    void
    processNewClusters(const std::vector<PointCloud::Ptr>& clusterClouds)
    {
        for (const auto& cloud : clusterClouds)
        {
            Eigen::Vector4f centroid;
            if (pcl::compute3DCentroid(*cloud, centroid) <= 0)
            {
                ROS_WARN("Could not compute centroid of point cloud. Skipping processing of potential object cluster!");
                return;
            }

            int objectID = -1;
            if (isNewObject(*cloud, objectID))
            {
                objectID = mUniqueObjectId++;
                ROS_INFO("Adding new object: %d", objectID);
                mObjectsCache.insert(std::pair<int, const PointCloud::Ptr>(objectID, cloud));
                std::vector<Eigen::Vector4f> prevPositions;
                prevPositions.push_back(centroid);
                mPrevPositionsCache.insert(std::pair<int, std::vector<Eigen::Vector4f>>(objectID, prevPositions));
                mLastSeenTimeCache.insert(std::pair<int, int>(objectID, mCurrTime));
            }
            else
            {
                // update last known position
                std::vector<Eigen::Vector4f>& prevPositions = mPrevPositionsCache[objectID];
                if (prevPositions.size() > mObjectCacheParams.mPositionHistoryCacheSize)
                {
                    prevPositions.erase(prevPositions.begin());
                }
                prevPositions.push_back(centroid);

                mLastSeenTimeCache[objectID] = mCurrTime;

                // Update the cloud
                mObjectsCache.erase(objectID);
                mObjectsCache.insert(std::pair<int, const PointCloud::Ptr>(objectID, cloud));
            }
        }
    }

    void
    removeStaleObjects()
    {
        for (auto it = mLastSeenTimeCache.cbegin(); it != mLastSeenTimeCache.cend(); )
        {
            int id = it->first;
            if (mCurrTime - it->second > mObjectCacheParams.mObjectCacheTime)
            {
                ROS_INFO("Removing Stale object: %d", id);
                mLastSeenTimeCache.erase(it++);
                mPrevPositionsCache.erase(id);
                mObjectsCache.erase(id);
            }
            else
            {
                ++it;
            }
        }
    }

    geometry_msgs::Point
    getGeomPoint(float x, float y, float z)
    {
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }

    visualization_msgs::Marker
    getObjectBoundsMarker(const Eigen::Vector4f& min, const Eigen::Vector4f& max, int id)
    {
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(1.0);
        marker.header.frame_id = mClusterTargetFrame;
        marker.scale.x = 0.005;
        marker.scale.y = 0.005;
        marker.color.a = 2.0;
        marker.ns = "";
        marker.id = id;
        marker.color = std_msgs::ColorRGBA(Color(Color::SCARLET));
        marker.points.push_back(getGeomPoint(min[0], min[1], min[2]));
        marker.points.push_back(getGeomPoint(min[0], max[1], min[2]));
        marker.points.push_back(getGeomPoint(min[0], max[1], min[2]));
        marker.points.push_back(getGeomPoint(max[0], max[1], min[2]));
        marker.points.push_back(getGeomPoint(max[0], max[1], min[2]));
        marker.points.push_back(getGeomPoint(max[0], min[1], min[2]));
        marker.points.push_back(getGeomPoint(max[0], min[1], min[2]));
        marker.points.push_back(getGeomPoint(min[0], min[1], min[2]));
        return marker;
    }

    visualization_msgs::Marker
    getObjectBoundsMarker(const mas_perception_msgs::Object& objMsg, int id)
    {
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(1.0);
        marker.header.frame_id = mClusterTargetFrame;
        marker.scale.x = 0.005;
        marker.scale.y = 0.005;
        marker.ns = "";
        marker.id = id;
        marker.color = std_msgs::ColorRGBA(Color(Color::SCARLET));

        marker.points.push_back(objMsg.bounding_box.vertices[0]);
        marker.points.push_back(objMsg.bounding_box.vertices[1]);
        marker.points.push_back(objMsg.bounding_box.vertices[1]);
        marker.points.push_back(objMsg.bounding_box.vertices[2]);
        marker.points.push_back(objMsg.bounding_box.vertices[2]);
        marker.points.push_back(objMsg.bounding_box.vertices[3]);
        marker.points.push_back(objMsg.bounding_box.vertices[3]);
        marker.points.push_back(objMsg.bounding_box.vertices[0]);

        marker.points.push_back(objMsg.bounding_box.vertices[4]);
        marker.points.push_back(objMsg.bounding_box.vertices[5]);
        marker.points.push_back(objMsg.bounding_box.vertices[5]);
        marker.points.push_back(objMsg.bounding_box.vertices[6]);
        marker.points.push_back(objMsg.bounding_box.vertices[6]);
        marker.points.push_back(objMsg.bounding_box.vertices[7]);
        marker.points.push_back(objMsg.bounding_box.vertices[7]);
        marker.points.push_back(objMsg.bounding_box.vertices[4]);

        marker.points.push_back(objMsg.bounding_box.vertices[0]);
        marker.points.push_back(objMsg.bounding_box.vertices[4]);
        marker.points.push_back(objMsg.bounding_box.vertices[1]);
        marker.points.push_back(objMsg.bounding_box.vertices[5]);
        marker.points.push_back(objMsg.bounding_box.vertices[2]);
        marker.points.push_back(objMsg.bounding_box.vertices[6]);
        marker.points.push_back(objMsg.bounding_box.vertices[3]);
        marker.points.push_back(objMsg.bounding_box.vertices[7]);
        return marker;
    }

    visualization_msgs::Marker
    getObjectOrientationMarker(const mas_perception_msgs::Object& objMsg, int id)
    {
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(1.0);
        marker.header.frame_id = mClusterTargetFrame;
        marker.scale.x = .1;
        marker.scale.y = .005;
        marker.scale.z = .005;
        marker.ns = "";
        marker.id = id + 2000;
        marker.color = std_msgs::ColorRGBA(Color(Color::SCARLET));
        marker.pose = objMsg.pose.pose;
        return marker;
    }

    visualization_msgs::Marker
    getObjectIdMarker(const Eigen::Vector4f& min, const Eigen::Vector4f& max, int id)
    {
        Eigen::Vector4f center = min + (max - min)/2.0;
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.text = std::to_string(id);
        marker.lifetime = ros::Duration(1.0);
        marker.header.frame_id = mClusterTargetFrame;
        marker.pose.position.x = center[0];
        marker.pose.position.y = center[1];
        marker.pose.position.z = center[2];
        marker.scale.z = 0.05;
        marker.id = id + 1000;
        marker.color = std_msgs::ColorRGBA(Color(Color::SCARLET));
        return marker;
    }

    mas_perception_msgs::Object
    createObjectMessage(PointCloud::ConstPtr cloudPtr, int id)
    {
        // Solution inspired from: https://stackoverflow.com/a/49705361
        PointT minPoint;
        PointT maxPoint;
        PointT position;
        Eigen::Matrix3f rotMatrix;
        Eigen::Vector3f massCenter;
        pcl::MomentOfInertiaEstimation<PointT> estimator;
        estimator.setInputCloud(cloudPtr);
        estimator.compute();
        estimator.getOBB(minPoint, maxPoint, position, rotMatrix);
        estimator.getMassCenter(massCenter);

        Eigen::Vector3f offset(position.x, position.y, position.z);
        std::vector<Eigen::Vector3f> points;
        points.push_back(Eigen::Vector3f(minPoint.x, minPoint.y, minPoint.z));
        points.push_back(Eigen::Vector3f(minPoint.x, maxPoint.y, minPoint.z));
        points.push_back(Eigen::Vector3f(maxPoint.x, maxPoint.y, minPoint.z));
        points.push_back(Eigen::Vector3f(maxPoint.x, minPoint.y, minPoint.z));
        points.push_back(Eigen::Vector3f(minPoint.x, minPoint.y, maxPoint.z));
        points.push_back(Eigen::Vector3f(minPoint.x, maxPoint.y, maxPoint.z));
        points.push_back(Eigen::Vector3f(maxPoint.x, maxPoint.y, maxPoint.z));
        points.push_back(Eigen::Vector3f(maxPoint.x, minPoint.y, maxPoint.z));

        // Copy the cluster cloud and set the z-value to that of the mass center for all points
        PointCloud::Ptr copy(new PointCloud);
        pcl::copyPointCloud(*cloudPtr, *copy);
        for(unsigned int i = 0; i < copy->points.size(); i++)
        {
            copy->points[i].z = massCenter.z();
        }

        mas_perception_msgs::Object msg;
        for (unsigned int i = 0 ; i < points.size(); i++)
        {
            points[i] = rotMatrix * points[i] + offset;
            geometry_msgs::Point p;
            p.x = points[i].x();
            p.y = points[i].y();
            p.z = points[i].z();
            msg.bounding_box.vertices.push_back(p);
        }

        msg.bounding_box.dimensions.x = std::abs(maxPoint.x - minPoint.x);
        msg.bounding_box.dimensions.y = std::abs(maxPoint.y - minPoint.y);
        msg.bounding_box.dimensions.z = std::abs(maxPoint.z - minPoint.z);

        geometry_msgs::Point center;
        center.x = massCenter.x();
        center.y = massCenter.y();
        center.z = massCenter.z();
        msg.bounding_box.center = center;

        // Compute the pose for the object
        pcl::MomentOfInertiaEstimation<PointT> orientationEstimator;
        Eigen::Vector3f major_vector, middle_vector, minor_vector;
        orientationEstimator.setInputCloud(copy);
        orientationEstimator.compute();
        orientationEstimator.getEigenVectors (major_vector, middle_vector, minor_vector);

        // Add pose to the Object message
        double yaw = std::atan2(major_vector.y(), major_vector.x());
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, yaw);
        msg.pose.pose.position = center;
        msg.pose.pose.orientation = tf2::toMsg(orientation);
        msg.pose.header.frame_id = mClusterTargetFrame;

        // Add dimensions to the Object message
        Eigen::Vector4f min, max;
        pcl::getMinMax3D(*cloudPtr, min, max);
        geometry_msgs::Vector3 dimensions;
        dimensions.x = std::abs(max[0] - min[0]);
        dimensions.y = std::abs(max[1] - min[1]);
        dimensions.z = std::abs(max[2] - min[2]);
        msg.dimensions.header.frame_id = mClusterTargetFrame;
        msg.dimensions.vector = dimensions;

        msg.name = std::string("object_") + std::to_string(id);

        return msg;
    }

    void
    updateObjectMsgCache()
    {
        if (mObjectMsgCache.empty())
        {
            for (const auto& c : mObjectsCache)
            {
                mObjectMsgCache.insert(std::pair<int, mas_perception_msgs::Object>(c.first, createObjectMessage(c.second, c.first)));
            }
        }
    }

    const mas_perception_msgs::Object*
    getObjectMessage(int id)
    {
        updateObjectMsgCache();
        std::map<int, mas_perception_msgs::Object>::iterator itr = mObjectMsgCache.find(id);
        if (itr == mObjectMsgCache.end())
        {
            ROS_WARN("Unable to find Object message for object %d", id);
            return nullptr;
        }

        return &(itr->second);
    }

    void
    publishCloud(PointCloud::ConstPtr cloudPtr, const ros::Publisher& publisher)
    {
        // publish the cloud
        sensor_msgs::PointCloud2::Ptr cloudMsgPtr = boost::make_shared<sensor_msgs::PointCloud2>();
        pcl::toROSMsg(*cloudPtr, *cloudMsgPtr);
        publisher.publish(*cloudMsgPtr);
    }

    void
    publishObjectCloud()
    {
        if ((mObjectCloudPub.getNumSubscribers() <= 0) || (mObjectsCache.empty()))
            return;

        // Merge all the clusters into once cloud
        PointCloud::Ptr mergedCloud(new PointCloud);
        bool headerInitialized = false;
        for (const auto& c : mObjectsCache)
        {   if (!headerInitialized)
            {
                mergedCloud->header = c.second->header;
                headerInitialized = true;
            }
            *mergedCloud += *(c.second);
        }
        mergedCloud->width = mergedCloud->size();
        mergedCloud->height = 1;
        mergedCloud->is_dense = true;

        // publish the cloud
        publishCloud(mergedCloud, mObjectCloudPub);
    }

    void
    publishObjectBoundsMarkers()
    {
        if (mObjectBoundsPub.getNumSubscribers() <= 0)
            return;

        visualization_msgs::MarkerArray markerArray;
        for (const auto& c : mObjectsCache)
        {
            Eigen::Vector4f min, max;
            pcl::getMinMax3D(*(c.second), min, max);
            if (mPublishOrientedBBox)
            {
                const mas_perception_msgs::Object* objMsg = getObjectMessage(c.first);
                if (objMsg == nullptr)
                    continue;
                markerArray.markers.push_back(getObjectBoundsMarker(*objMsg, c.first));
                markerArray.markers.push_back(getObjectOrientationMarker(*objMsg, c.first));
            }
            else
            {
                markerArray.markers.push_back(getObjectBoundsMarker(min, max, c.first));
            }
            markerArray.markers.push_back(getObjectIdMarker(min, max, c.first));
        }
        // Publish the marker array
        mObjectBoundsPub.publish(markerArray);
    }

    void
    publishObjectObjectMessages()
    {
        if (mObjectObjectsPub.getNumSubscribers() <= 0)
            return;

        updateObjectMsgCache();
        mas_perception_msgs::ObjectList objectList;
        for (const auto& msg : mObjectMsgCache)
        {
            objectList.objects.push_back(msg.second);
        }
        mObjectObjectsPub.publish(objectList);
    }

    void cleanup()
    {
        mObjectMsgCache.clear();
    }
};

}   // namespace mdr_cloud_object_detection

int main(int pArgc, char** pArgv)
{
    ros::init(pArgc, pArgv, "cloud_object_detection");
    ros::NodeHandle nh("~");

    // load launch parameters
    std::string cloudTopic, filteredCloudTopic, objectCloudTopic,
                objectBoundsTopic, objectObjectsTopic, transformTargetFrame,
                clusterTargetFrame, occupancyCheckerActionName;
    bool publishOrientedBBox = false;
    if (!nh.getParam("cloud_topic", cloudTopic) || cloudTopic.empty())
    {
        ROS_ERROR("No 'cloud_topic' specified as parameter");
        return EXIT_FAILURE;
    }
    if (!nh.getParam("filtered_cloud_topic", filteredCloudTopic) || filteredCloudTopic.empty())
    {
        ROS_ERROR("No 'filtered_cloud_topic' specified as parameter");
        return EXIT_FAILURE;
    }
    if (!nh.getParam("object_cloud_topic", objectCloudTopic) || objectCloudTopic.empty())
    {
        ROS_ERROR("No 'object_cloud_topic' specified as parameter");
        return EXIT_FAILURE;
    }
    if (!nh.getParam("objects_bounds_topic", objectBoundsTopic) || objectBoundsTopic.empty())
    {
        ROS_ERROR("No 'objects_bounds_topic' specified as parameter");
        return EXIT_FAILURE;
    }
    if (!nh.getParam("transform_target_frame", transformTargetFrame) || transformTargetFrame.empty())
    {
        ROS_ERROR("No 'transform_target_frame' specified as parameter");
        return EXIT_FAILURE;
    }
    if (!nh.getParam("cluster_target_frame", clusterTargetFrame) || clusterTargetFrame.empty())
    {
        ROS_ERROR("No 'cluster_target_frame' specified as parameter");
        return EXIT_FAILURE;
    }
    if (!nh.getParam("occupancy_checker_action_name", occupancyCheckerActionName) || occupancyCheckerActionName.empty())
    {
        ROS_ERROR("No 'occupancy_checker_action_name' specified as parameter");
        return EXIT_FAILURE;
    }
    if (!nh.getParam("objects_object_topic", objectObjectsTopic) || objectObjectsTopic.empty())
    {
        ROS_ERROR("No 'objects_object_topic' specified as parameter");
        return EXIT_FAILURE;
    }
    nh.getParam("publish_oriented_bbox", publishOrientedBBox);

    // run cloud filtering and object detection
    mdr_cloud_object_detection::CloudObjectDetectionNode objectDetection(nh, cloudTopic, filteredCloudTopic,
                                                                      objectCloudTopic, transformTargetFrame,
                                                                      clusterTargetFrame, occupancyCheckerActionName,
                                                                      objectBoundsTopic, objectObjectsTopic,
                                                                      publishOrientedBBox);

    while (ros::ok())
        ros::spin();

    return 0;
}
