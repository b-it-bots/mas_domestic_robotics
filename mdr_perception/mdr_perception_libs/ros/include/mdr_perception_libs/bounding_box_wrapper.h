//
// Created by minh on 02.04.18.
//

#ifndef MDR_PERCEPTION_LIBS_BOUNDING_BOX_PY_WRAPPER_H
#define MDR_PERCEPTION_LIBS_BOUNDING_BOX_PY_WRAPPER_H
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
        geometry_msgs::Pose mPose;

        void calculatePose();
    };
}
#endif //MDR_PERCEPTION_LIBS_BOUNDING_BOX_PY_WRAPPER_H
