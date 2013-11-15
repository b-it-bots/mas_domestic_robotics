#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <cob_relayboard/EmergencyStopState.h>


bool g_is_em_active = true;


void emergency_stop_state_cb(const cob_relayboard::EmergencyStopState &state)
{
    if (state.emergency_state == cob_relayboard::EmergencyStopState::EMFREE) {
        g_is_em_active = false;
    } else {
        g_is_em_active = true;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "emergency_stop_mapper");
    ros::NodeHandle nh("~");

    ros::Rate rate(20); // 20 Hz

    nh.subscribe("/emergency_stop_state", 1, emergency_stop_state_cb);
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("emergency_stop_active", 1);

    while (ros::ok()) {
        std_msgs::Bool em_active_msg;
        em_active_msg.data = g_is_em_active;

        pub.publish(em_active_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
