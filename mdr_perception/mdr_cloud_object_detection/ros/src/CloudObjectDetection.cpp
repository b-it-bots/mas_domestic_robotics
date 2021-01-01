/*!
 * @copyright 2020 Bonn-Rhein-Sieg University
 *
 * @author Sushant Vijay Chavan
 *
 * @brief Class to detect objects from a point cloud
 */
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mdr_cloud_object_detection/Color.h>
#include <mdr_cloud_object_detection/CloudObjectDetection.h>

using namespace mdr_cloud_object_detection;

CloudObjectDetection::CloudObjectDetection(const ros::NodeHandle &pNodeHandle,
                                           const std::string &pCloudInTopic,
                                           const std::string &pFilteredCloudTopic,
                                           const std::string &pObjectCloudTopic,
                                           const std::string &pTransformTargetFrame,
                                           const std::string &pClusterTargetFrame,
                                           const std::string &pOccupancyCheckerActionName,
                                           const std::string &pObjectsBoundsTopic,
                                           const std::string &pObjectListTopic,
                                           bool pPublishOrientedBBox)
: mNodeHandle(pNodeHandle)
, mObjectDetectionConfigServer(mNodeHandle)
, mTransformTargetFrame(pTransformTargetFrame)
, mClusterTargetFrame(pClusterTargetFrame)
, mPublishOrientedBBox(pPublishOrientedBBox)
, mUniqueObjectId(0)
, mOccupancyCheckerClient(pOccupancyCheckerActionName, true)
{
    ROS_INFO("[CloudObjectDetection] Waiting for OccupancyChecker server");
    mOccupancyCheckerClient.waitForServer();

    ROS_INFO("setting up dynamic reconfiguration server for object detection");
    auto odCallback = boost::bind(&CloudObjectDetection::objectDetectionConfigCallback, this, _1, _2);
    mObjectDetectionConfigServer.setCallback(odCallback);

    ROS_INFO("subscribing to point cloud topic and advertising processed result");
    mCloudInSub = mNodeHandle.subscribe(pCloudInTopic, 1, &CloudObjectDetection::cloudCallback, this);
    mResetObjectCacheSub = mNodeHandle.subscribe("reset_cache", 1, &CloudObjectDetection::resetObjectCacheCallback, this);
    mFilteredCloudPub = mNodeHandle.advertise<sensor_msgs::PointCloud2>(pFilteredCloudTopic, 1);
    mObjectCloudPub = mNodeHandle.advertise<sensor_msgs::PointCloud2>(pObjectCloudTopic, 1);
    mObjectBoundsPub = mNodeHandle.advertise<visualization_msgs::MarkerArray>(pObjectsBoundsTopic, 1);
    mObjectListPub = mNodeHandle.advertise<mas_perception_msgs::ObjectList>(pObjectListTopic, 1);
}

void CloudObjectDetection::objectDetectionConfigCallback(const ObjectDetectionConfig &pConfig,
                                                         uint32_t pLevel)
{
    // Cloud Filter params
    PassThroughVoxelFilterParams cloudFilterParams;
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

void CloudObjectDetection::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pCloudMsgPtr)
{
    // do not process cloud when there's no subscriber
    if (mFilteredCloudPub.getNumSubscribers() == 0 &&
        mObjectBoundsPub.getNumSubscribers() == 0 &&
        mObjectListPub.getNumSubscribers() == 0 &&
        mObjectCloudPub.getNumSubscribers() == 0)
        return;

    if (!mTfListener.waitForTransform(mTransformTargetFrame, pCloudMsgPtr->header.frame_id.c_str(), pCloudMsgPtr->header.stamp, ros::Duration(1.0)) ||
        !mTfListener.waitForTransform(mClusterTargetFrame, mTransformTargetFrame, pCloudMsgPtr->header.stamp, ros::Duration(1.0)))
        return;

    // transform the cloud to a desired frame
    PointCloud::Ptr cloudInPtr = boost::make_shared<PointCloud>();
    pcl::fromROSMsg(*pCloudMsgPtr, *cloudInPtr);
    PointCloud::Ptr transformedCloudPtr = transformPointCloud(*cloudInPtr, mTransformTargetFrame);
    if (transformedCloudPtr == nullptr)
        return;

    // filter the cloud
    PointCloud::Ptr filteredCloudPtr = mCloudFilter.filterCloud(transformedCloudPtr);
    if (filteredCloudPtr->size() < mClusterParams.mMinClusterSize)
    {
        // Stop processing the point cloud if the filtered cloud does
        // not have enough points to create even one cluster
        return;
    }

    if (mFilteredCloudPub.getNumSubscribers() > 0)
    {
        // publish the filtered cloud for debugging
        publishCloud(filteredCloudPtr, mFilteredCloudPub);
    }

    mCurrTime = pCloudMsgPtr->header.stamp.sec;

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

    if (cluster_indices.size() == 0)
        return;

    std::vector<PointCloud::Ptr> clusterClouds;
    getClusterClouds(clusterClouds, filteredCloudPtr, cluster_indices);
    filterClusterCloudsBySize(clusterClouds);
    filterClusterCloudsNearOccupiedSpaces(clusterClouds);

    processNewClusters(clusterClouds);
    removeStaleObjects();

    publishObjectCloud();
    publishObjectBoundsMarkers();
    publishObjectListMessages();

    cleanup();
}

void CloudObjectDetection::resetObjectCacheCallback(const std_msgs::Bool& reset)
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

PointCloud::Ptr CloudObjectDetection::transformPointCloud(const PointCloud& cloudIn,
                                                          const std::string& targetFrame)
{
    PointCloud::Ptr transformedCloudPtr = boost::make_shared<PointCloud>();
    if (!pcl_ros::transformPointCloud(targetFrame, cloudIn, *transformedCloudPtr, mTfListener))
    {
        ROS_WARN("Failed to transform cloud to frame '%s' from frame '%s'.",
                targetFrame.c_str(), cloudIn.header.frame_id.c_str());
        return nullptr;
    }
    return transformedCloudPtr;
}

void CloudObjectDetection::filterClusterCloudsBySize(std::vector<PointCloud::Ptr>& clusterClouds)
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

void CloudObjectDetection::filterClusterCloudsNearOccupiedSpaces(std::vector<PointCloud::Ptr>& clusterClouds)
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

void CloudObjectDetection::getClusterClouds(std::vector<PointCloud::Ptr>& clusterClouds,
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

        PointCloud::Ptr transformedCloudPtr = transformPointCloud(*clusterCloud, mClusterTargetFrame);
        if (transformedCloudPtr == nullptr)
        {
            ROS_WARN("Failed to transform cluster cloudThis cluster will be skipped in the current frame!");
            continue;
        }

        clusterClouds.push_back(transformedCloudPtr);
    }
}

// Cloud comparison sample from https://stackoverflow.com/a/55930847
float CloudObjectDetection::nearestDistance(const pcl::search::KdTree<PointT>& tree,
                                            const PointT& pt)
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
float CloudObjectDetection::getCloudSimilarity(const PointCloud& cloudA,
                                               const PointCloud& cloudB,
                                               float threshold)
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

bool CloudObjectDetection::isNewObject(const PointCloud& cloud,
                                       int& knownObjectId)
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

void CloudObjectDetection::processNewClusters(const std::vector<PointCloud::Ptr>& clusterClouds)
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

void CloudObjectDetection::removeStaleObjects()
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

geometry_msgs::Point CloudObjectDetection::getPointMsg(float x,
                                                       float y,
                                                       float z)
{
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

visualization_msgs::Marker CloudObjectDetection::getObjectBoundsMarker(const Eigen::Vector4f& min,
                                                                       const Eigen::Vector4f& max,
                                                                       int id)
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
    marker.points.push_back(getPointMsg(min[0], min[1], min[2]));
    marker.points.push_back(getPointMsg(min[0], max[1], min[2]));
    marker.points.push_back(getPointMsg(min[0], max[1], min[2]));
    marker.points.push_back(getPointMsg(max[0], max[1], min[2]));
    marker.points.push_back(getPointMsg(max[0], max[1], min[2]));
    marker.points.push_back(getPointMsg(max[0], min[1], min[2]));
    marker.points.push_back(getPointMsg(max[0], min[1], min[2]));
    marker.points.push_back(getPointMsg(min[0], min[1], min[2]));
    return marker;
}

visualization_msgs::Marker CloudObjectDetection::getObjectBoundsMarker(const mas_perception_msgs::Object& objMsg,
                                                                       int id)
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

visualization_msgs::Marker CloudObjectDetection::getObjectOrientationMarker(const mas_perception_msgs::Object& objMsg,
                                                                            int id)
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

visualization_msgs::Marker CloudObjectDetection::getObjectIdMarker(const Eigen::Vector4f& min,
                                                                   const Eigen::Vector4f& max,
                                                                   int id)
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

mas_perception_msgs::Object CloudObjectDetection::createObjectMessage(PointCloud::ConstPtr cloudPtr,
                                                                      int id)
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

    mas_perception_msgs::Object msg;
    msg.name = std::string("object_") + std::to_string(id);

    // Add the cluster cloud
    sensor_msgs::PointCloud2::Ptr cloudMsgPtr = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*cloudPtr, *cloudMsgPtr);
    mas_perception_msgs::ObjectView view;
    view.point_cloud = *cloudMsgPtr;
    msg.views.push_back(view);

    // Add bounding box values
    geometry_msgs::Point center;
    center.x = massCenter.x();
    center.y = massCenter.y();
    center.z = massCenter.z();
    msg.bounding_box.center = center;
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
    msg.dimensions.header.frame_id = mClusterTargetFrame;
    msg.dimensions.vector = msg.bounding_box.dimensions;

    // Copy the cluster cloud and set the z-value to that of the mass center for all points
    PointCloud::Ptr copy(new PointCloud);
    pcl::copyPointCloud(*cloudPtr, *copy);
    for(unsigned int i = 0; i < copy->points.size(); i++)
    {
        copy->points[i].z = massCenter.z();
    }

    // Add pose to the Object message
    msg.pose.header.frame_id = mClusterTargetFrame;
    msg.pose.pose.position = center;
    // Compute the orientation of the object in the XY plane of the mClusterTargetFrame
    pcl::MomentOfInertiaEstimation<PointT> orientationEstimator;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    orientationEstimator.setInputCloud(copy);
    orientationEstimator.compute();
    orientationEstimator.getEigenVectors (major_vector, middle_vector, minor_vector);
    double yaw = std::atan2(major_vector.y(), major_vector.x());
    // restrict yaw to the first two quadrants of the mClusterTargetFrame
    constexpr double pi = 3.14159265358979323846;
    if (yaw > pi/2.0)
        yaw -= pi;
    else if (yaw < -pi/2.0)
        yaw += pi;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw);
    msg.pose.pose.orientation = tf2::toMsg(orientation);

    return msg;
}

void CloudObjectDetection::updateObjectMsgCache()
{
    if (mObjectMsgCache.empty())
    {
        for (const auto& c : mObjectsCache)
        {
            mObjectMsgCache.insert(std::pair<int, mas_perception_msgs::Object>(c.first, createObjectMessage(c.second, c.first)));
        }
    }
}

const mas_perception_msgs::Object* CloudObjectDetection::getObjectMessage(int id)
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

void CloudObjectDetection::publishCloud(PointCloud::ConstPtr cloudPtr,
                                        const ros::Publisher& publisher)
{
    // publish the cloud
    sensor_msgs::PointCloud2::Ptr cloudMsgPtr = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*cloudPtr, *cloudMsgPtr);
    publisher.publish(*cloudMsgPtr);
}

void CloudObjectDetection::publishObjectCloud()
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

void CloudObjectDetection::publishObjectBoundsMarkers()
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

void CloudObjectDetection::publishObjectListMessages()
{
    if (mObjectListPub.getNumSubscribers() <= 0)
        return;

    updateObjectMsgCache();
    mas_perception_msgs::ObjectList objectList;
    for (const auto& msg : mObjectMsgCache)
    {
        objectList.objects.push_back(msg.second);
    }
    mObjectListPub.publish(objectList);
}

void CloudObjectDetection::cleanup()
    {
        mObjectMsgCache.clear();
    }
