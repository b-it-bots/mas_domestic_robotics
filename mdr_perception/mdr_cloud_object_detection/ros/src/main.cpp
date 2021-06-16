/*!
 * @copyright 2020 Bonn-Rhein-Sieg University
 *
 * @author Sushant Vijay Chavan
 *
 */
#include <mdr_cloud_object_detection/CloudObjectDetection.h>

int main(int pArgc, char** pArgv)
{
    ros::init(pArgc, pArgv, "cloud_object_detection");
    ros::NodeHandle nh("~");

    // load launch parameters
    std::string cloudInTopic;
    if (!nh.getParam("cloud_in_topic", cloudInTopic) || cloudInTopic.empty())
    {
        ROS_ERROR("No 'cloud_in_topic' specified as parameter");
        return EXIT_FAILURE;
    }

    std::string filteredCloudTopic;
    if (!nh.getParam("filtered_cloud_topic", filteredCloudTopic) || filteredCloudTopic.empty())
    {
        ROS_ERROR("No 'filtered_cloud_topic' specified as parameter");
        return EXIT_FAILURE;
    }

    std::string objectCloudTopic;
    if (!nh.getParam("object_cloud_topic", objectCloudTopic) || objectCloudTopic.empty())
    {
        ROS_ERROR("No 'object_cloud_topic' specified as parameter");
        return EXIT_FAILURE;
    }

    std::string objectBoundsTopic;
    if (!nh.getParam("objects_bounds_topic", objectBoundsTopic) || objectBoundsTopic.empty())
    {
        ROS_ERROR("No 'objects_bounds_topic' specified as parameter");
        return EXIT_FAILURE;
    }

    std::string transformTargetFrame;
    if (!nh.getParam("transform_target_frame", transformTargetFrame) || transformTargetFrame.empty())
    {
        ROS_ERROR("No 'transform_target_frame' specified as parameter");
        return EXIT_FAILURE;
    }

    std::string clusterTargetFrame;
    if (!nh.getParam("cluster_target_frame", clusterTargetFrame) || clusterTargetFrame.empty())
    {
        ROS_ERROR("No 'cluster_target_frame' specified as parameter");
        return EXIT_FAILURE;
    }

    std::string occupancyCheckerActionName;
    if (!nh.getParam("occupancy_checker_action_name", occupancyCheckerActionName) || occupancyCheckerActionName.empty())
    {
        ROS_ERROR("No 'occupancy_checker_action_name' specified as parameter");
        return EXIT_FAILURE;
    }

    std::string objectListTopic;
    if (!nh.getParam("object_list_topic", objectListTopic) || objectListTopic.empty())
    {
        ROS_ERROR("No 'object_list_topic' specified as parameter");
        return EXIT_FAILURE;
    }

    bool publishOrientedBBox = false;
    nh.getParam("publish_oriented_bbox", publishOrientedBBox);

    // run cloud filtering and object detection
    mdr_cloud_object_detection::CloudObjectDetection objectDetection(nh,
                                                                     cloudInTopic,
                                                                     filteredCloudTopic,
                                                                     objectCloudTopic,
                                                                     transformTargetFrame,
                                                                     clusterTargetFrame,
                                                                     occupancyCheckerActionName,
                                                                     objectBoundsTopic,
                                                                     objectListTopic,
                                                                     publishOrientedBBox);

    ros::Rate node_rate(2);
    while (ros::ok())
    {
        ros::spinOnce();
        node_rate.sleep();
    }

    return 0;
}
