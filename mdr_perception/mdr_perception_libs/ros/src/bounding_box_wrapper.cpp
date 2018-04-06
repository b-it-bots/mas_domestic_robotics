#include <utility>
#include <boost/python.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <mcr_scene_segmentation/bounding_box.h>
#include <mcr_perception_msgs/BoundingBox.h>
#include "mdr_perception_libs/impl/ros_message_serialization.hpp"
#include "mdr_perception_libs/bounding_box_wrapper.h"

namespace bp = boost::python;

namespace mdr_perception_libs
{
    BoundingBoxWrapper::BoundingBoxWrapper(std::string pSerialCloud, bp::list &pNormal)
    {
        sensor_msgs::PointCloud2 rosCloud = from_python<sensor_msgs::PointCloud2>(std::move(pSerialCloud));

        bp::ssize_t len = bp::len(pNormal);
        if (len != 3)
            throw std::invalid_argument("expect normal vector to contain 3 numbers of type float");

        std::vector<float> normalVect;
        for (int i = 0; i < len; ++i)
        {
            normalVect.push_back(bp::extract<float>(pNormal[i]));
        }
        const Eigen::Vector3f normal(normalVect[0], normalVect[1], normalVect[2]);

        PointCloud::Ptr pclCloudPtr(new PointCloud);
        pcl::fromROSMsg(rosCloud, *pclCloudPtr);

        mBox = BoundingBox::create(pclCloudPtr->points, normal);

        calculatePose(rosCloud.header);
    }

    BoundingBoxWrapper::~BoundingBoxWrapper() = default;

    std::string BoundingBoxWrapper::getPose()
    {
        std::string serialPose = to_python(mPose);
        return serialPose;
    }

    void BoundingBoxWrapper::calculatePose(std_msgs::Header header)
    {
        BoundingBox::Points vertices = mBox.getVertices();
        Eigen::Vector3f n1;
        Eigen::Vector3f n2;
        Eigen::Vector3f n3 = (vertices[4] - vertices[0]) / (vertices[4] - vertices[0]).norm();

        // set n1 based on longest edge
        if ((vertices[1] - vertices[0]).norm() > (vertices[3] - vertices[0]).norm())
        {
            n1 = (vertices[1] - vertices[0]) / (vertices[1] - vertices[0]).norm();
        }
        else
        {
            n1 = (vertices[3] - vertices[0]) / (vertices[3] - vertices[0]).norm();
        }
        n2 = n3.cross(n1);

        Eigen::Matrix3f m;
        m << n1 , n2 , n3;
        Eigen::Quaternion<float> q(m);
        q.normalize();

        Eigen::Vector3f centroid = mBox.getCenter();

        mPose.header = header;
        mPose.pose.position.x = centroid(0);
        mPose.pose.position.y = centroid(1);
        mPose.pose.position.z = centroid(2);
        mPose.pose.orientation.x = q.x();
        mPose.pose.orientation.y = q.y();
        mPose.pose.orientation.z = q.z();
        mPose.pose.orientation.w = q.w();
    }

    std::string BoundingBoxWrapper::getRosMsg()
    {
        mcr_perception_msgs::BoundingBox boxMsg;
        boxMsg.center = mPose.pose.position;

        Eigen::Vector3f dimensions = mBox.getDimensions();
        boxMsg.dimensions.x = dimensions(0);
        boxMsg.dimensions.y = dimensions(1);
        boxMsg.dimensions.z = dimensions(2);

        BoundingBox::Points vertices = mBox.getVertices();
        for (size_t i = 0; i < vertices.size(); ++i)
        {
            geometry_msgs::Point vertice;
            vertice.x = vertices[i](0);
            vertice.y = vertices[i](1);
            vertice.z = vertices[i](2);
            boxMsg.vertices.push_back(vertice);
        }
        return to_python(boxMsg);
    }
}

BOOST_PYTHON_MODULE(_cpp_wrapper)
{
    using namespace mdr_perception_libs;
    bp::class_<BoundingBoxWrapper>("BoundingBoxWrapper", bp::init<std::string, bp::list&>())
            .def("get_pose", &BoundingBoxWrapper::getPose)
            .def("get_ros_message", &BoundingBoxWrapper::getRosMsg)
    ;
}