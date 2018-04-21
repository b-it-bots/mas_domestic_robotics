#ifndef MDR_PERCEPTION_LIBS_ROS_MESSAGE_SERIALIZATION_H
#define MDR_PERCEPTION_LIBS_ROS_MESSAGE_SERIALIZATION_H

namespace mdr_perception_libs
{
    template<typename M>
    M from_python(const std::string pSerialMsg)
    {
        size_t serialSize = pSerialMsg.size();
        boost::shared_array<uint8_t> buffer(new uint8_t[serialSize]);
        for (size_t i = 0; i < serialSize; ++i)
        {
            buffer[i] = pSerialMsg[i];
        }
        ros::serialization::IStream stream(buffer.get(), serialSize);
        M msg;
        ros::serialization::Serializer<M>::read(stream, msg);
        return msg;
    };

    template<typename M>
    std::string to_python(const M& pRosMsg)
    {
        size_t serialSize = ros::serialization::serializationLength(pRosMsg);
        boost::shared_array<uint8_t> buffer(new uint8_t[serialSize]);
        ros::serialization::OStream stream(buffer.get(), serialSize);
        ros::serialization::serialize(stream, pRosMsg);
        std::string serialMsg;
        serialMsg.reserve(serialSize);
        for (size_t i = 0; i < serialSize; ++i)
        {
            serialMsg.push_back(buffer[i]);
        }
        return serialMsg;
    };
}

#endif //MDR_PERCEPTION_LIBS_ROS_MESSAGE_SERIALIZATION_H
