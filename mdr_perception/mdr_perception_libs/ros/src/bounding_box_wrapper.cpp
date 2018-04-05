#include <boost/python.hpp>
#include <geometry_msgs/PoseArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mcr_scene_segmentation/bounding_box.h>
#include "mdr_perception_libs/impl/ros_message_serialization.hpp"
#include "mdr_perception_libs/bounding_box_wrapper.h"

namespace bp = boost::python;

namespace mdr_perception_libs
{
    BoundingBoxWrapper::BoundingBoxWrapper(std::string pSerialCloud, bp::list &pNormal)
    {
        sensor_msgs::PointCloud2 rosCloud = from_python<sensor_msgs::PointCloud2>(pSerialCloud);

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
    }

    BoundingBoxWrapper::~BoundingBoxWrapper() {};

    std::string BoundingBoxWrapper::getPose()
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

        geometry_msgs::Pose pose;
        pose.position.x = centroid(0);
        pose.position.y = centroid(1);
        pose.position.z = centroid(2);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        std::string serialPose = to_python(pose);
        return serialPose;
    }

}

BOOST_PYTHON_MODULE(_cpp_wrapper)
{
    using namespace mdr_perception_libs;
    bp::class_<BoundingBoxWrapper>("BoundingBoxWrapper", bp::init<std::string, bp::list&>())
            .def("get_pose", &BoundingBoxWrapper::getPose)
    ;
}