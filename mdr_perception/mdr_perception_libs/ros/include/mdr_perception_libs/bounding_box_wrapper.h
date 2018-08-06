/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Minh Nguyen
 *
 */
#ifndef MDR_PERCEPTION_LIBS_BOUNDING_BOX_WRAPPER_H
#define MDR_PERCEPTION_LIBS_BOUNDING_BOX_WRAPPER_H

#include <string>

namespace mdr_perception_libs
{

class BoundingBoxWrapper
{
public:
    BoundingBoxWrapper(std::string, boost::python::list&);
    ~BoundingBoxWrapper();

    std::string getPose();
    std::string getRosMsg();

private:
    BoundingBox mBox;
    geometry_msgs::PoseStamped mPose;

    void calculatePose(std_msgs::Header);
};

}  // namespace mdr_perception_libs
#endif  // MDR_PERCEPTION_LIBS_BOUNDING_BOX_WRAPPER_H
