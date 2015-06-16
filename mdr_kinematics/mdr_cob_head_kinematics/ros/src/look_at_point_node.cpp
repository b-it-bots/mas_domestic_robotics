/*
 * look_at_point_node.cpp
 *
 *  Created on: Mar 29, 2015
 *      Author: fred
 */

#include <mdr_cob_head_kinematics/look_at_point_node.h>

LookAtPointNode::LookAtPointNode(): current_state_(INIT), current_pan_angle_(0.0), current_tilt_angle_(0.0), current_pan_velocity_(0.0), current_tilt_velocity_(0.0), pan_angle_(0.0), tilt_angle_(0.0), event_msg_received_(false), point_msg_received_(false)
{
    ros::NodeHandle nh("~");

    sub_point_ = nh.subscribe("input/point", 1, &LookAtPointNode::pointCallback, this);
    sub_joint_states_ = nh.subscribe("input/joint_states", 1, &LookAtPointNode::jointStatesCallback, this);
    sub_event_ = nh.subscribe("event_in", 1, &LookAtPointNode::eventCallback, this);

    pub_torso_velocities_ = nh.advertise < brics_actuator::JointVelocities > ("output/joint_velocities", 1);

    tf_listener_ = new tf::TransformListener();

    // set joint names and link names
    pan_frame_ = "torso_upper_neck_pan_link";
    pan_joint_name_ = "torso_upper_neck_pan_joint";
    tilt_frame_ = "torso_upper_neck_tilt_link";
    tilt_joint_name_ = "torso_upper_neck_tilt_joint";

    // read pan parameter
    nh.param<double>("min_pan_angle", min_pan_angle_, -1.0);
    nh.param<double>("max_pan_angle", max_pan_angle_, 1.0);
    nh.param<double>("min_pan_velocity", min_pan_velocity_, 0.05);
    nh.param<double>("max_pan_velocity", max_pan_velocity_, 0.15);

    // read tilt parameter
    nh.param<double>("min_tilt_angle", min_tilt_angle_, -0.5);
    nh.param<double>("max_tilt_angle", max_tilt_angle_, 0.5);
    nh.param<double>("min_tilt_velocity", min_tilt_velocity_, 0.05);
    nh.param<double>("max_tilt_velocity", max_tilt_velocity_, 0.15);
}

LookAtPointNode::~LookAtPointNode()
{
    sub_event_.shutdown();
    sub_point_.shutdown();
    sub_joint_states_.shutdown();
    pub_torso_velocities_.shutdown();
}

void LookAtPointNode::eventCallback(const std_msgs::StringPtr &msg)
{
    event_msg_ = *msg;
    event_msg_received_ = true;
}

void LookAtPointNode::pointCallback(const geometry_msgs::PointStampedPtr &msg)
{
    point_msg_ = *msg;
    point_msg_received_ = true;
}

void LookAtPointNode::jointStatesCallback(const sensor_msgs::JointStatePtr &msg)
{
    // get position and velocity of the pan and tilt joint + remember when we received these infos
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == pan_joint_name_)
        {
            current_pan_angle_ = msg->position[i];
            current_pan_velocity_ = msg->velocity[i];
            last_joint_states_timestamp = ros::Time::now();
        } else if (msg->name[i] == tilt_joint_name_)
        {
            current_tilt_angle_ = msg->position[i];
            current_tilt_velocity_ = msg->velocity[i];
            last_joint_states_timestamp = ros::Time::now();
        }
    }
}

bool LookAtPointNode::calculatePanAndTiltAngle()
{
    try
    {
        tf_listener_->waitForTransform(pan_frame_, point_msg_.header.frame_id, point_msg_.header.stamp, ros::Duration(0.1));
        tf_listener_->transformPoint(pan_frame_, point_msg_, set_point_in_pan_frame_);

        tf_listener_->waitForTransform(tilt_frame_, point_msg_.header.frame_id, point_msg_.header.stamp, ros::Duration(0.1));
        tf_listener_->transformPoint(tilt_frame_, point_msg_, set_point_in_tilt_frame_);

        // get pan angle (in rad) using polar coordinate
        pan_angle_ = atan2(set_point_in_pan_frame_.point.y, set_point_in_pan_frame_.point.x);

        // get pan angle (in rad) using polar coordinate
        tilt_angle_ = atan2(set_point_in_tilt_frame_.point.y, sqrt((pow(set_point_in_tilt_frame_.point.x, 2) + pow(set_point_in_tilt_frame_.point.z, 2))));

        // calculate the new joint position for pan and tilt
        pan_angle_ += current_pan_angle_;
        tilt_angle_ += current_tilt_angle_;

        // only allow a max and min angle
        if (pan_angle_ > max_pan_angle_)
            pan_angle_ = max_pan_angle_;
        else if (pan_angle_ < min_pan_angle_)
            pan_angle_ = min_pan_angle_;

        if (tilt_angle_ > max_tilt_angle_)
            tilt_angle_ = max_tilt_angle_;
        else if (tilt_angle_ < min_tilt_angle_)
            tilt_angle_ = min_tilt_angle_;

    } catch (std::exception &e)
    {
        ROS_ERROR_STREAM("Could not transform point: " << e.what());

        publishTorsoJointvelocities(0.0, 0.0, 0.0, 0.0);

        return false;
    }

    return true;
}

void LookAtPointNode::publishZeroTorsoJointvelocities()
{
    current_pan_velocity_ = 1.0;
    current_tilt_velocity_ = 1.0;
    while (current_pan_velocity_ != 0.0 || current_tilt_velocity_ != 0.0)
    {
        publishTorsoJointvelocities(0.0, 0.0, 0.0, 0.0);
        ros::spinOnce();
    }
}

void LookAtPointNode::publishTorsoJointvelocities(const double &lower_pan, const double &lower_tilt, const double &upper_pan, const double &upper_tilt)
{
    brics_actuator::JointVelocities list;
    brics_actuator::JointValue pos;

    pos.timeStamp = ros::Time::now();
    pos.joint_uri = "torso_lower_neck_pan_joint";
    pos.unit = "rad";
    pos.value = lower_pan;
    list.velocities.push_back(pos);

    pos.timeStamp = ros::Time::now();
    pos.joint_uri = "torso_lower_neck_tilt_joint";
    pos.unit = "rad";
    pos.value = lower_tilt;
    list.velocities.push_back(pos);

    pos.timeStamp = ros::Time::now();
    pos.joint_uri = pan_joint_name_;
    pos.unit = "rad";
    pos.value = upper_pan;
    list.velocities.push_back(pos);

    pos.timeStamp = ros::Time::now();
    pos.joint_uri = tilt_joint_name_;
    pos.unit = "rad";
    pos.value = upper_tilt;
    list.velocities.push_back(pos);

    pub_torso_velocities_.publish(list);
}



LookAtPointNode::State LookAtPointNode::update()
{
    // check if a new event has been received
    if (event_msg_received_)
    {
        ROS_INFO_STREAM("Received event: " << event_msg_.data);

        if(event_msg_.data == "e_start")
            current_state_ = IDLE;
        else if(event_msg_.data == "e_stop")
        {
            publishZeroTorsoJointvelocities();
            current_state_ = INIT;
        }
        else
            ROS_ERROR_STREAM("Event not supported: " << event_msg_.data);

        event_msg_received_ = false;
    }

    switch (current_state_)
    {
        case INIT: break;
        case IDLE: idleState(); break;
        case RUN: runState(); break;
        default: break;
    }

    point_msg_received_ = false;

    return current_state_;
}

void LookAtPointNode::LookAtPointNode::idleState()
{
    if (point_msg_received_)
    {
        if (calculatePanAndTiltAngle())
            current_state_ = RUN;
        else
            current_state_ = IDLE;
    }
}


void LookAtPointNode::LookAtPointNode::runState()
{
    // only work on "recent" joint state msgs
    ros::Duration age_of_last_joint_state = ros::Time::now() - last_joint_states_timestamp;
    if (age_of_last_joint_state.toSec() > 1.0)
    {
        publishZeroTorsoJointvelocities();
        return;
    }

    // calculate error between target and current joint position. Use this as velocity for the joint (HACK), but
    // this component will be anyway replaced by the whole body control framework
    double pan_velocity = (pan_angle_ - current_pan_angle_);
    double tilt_velocity = (tilt_angle_ - current_tilt_angle_);

    // restrict maximum and minimum velocity
    if (fabs(pan_velocity) > max_pan_velocity_)
        pan_velocity = max_pan_velocity_ * (pan_velocity / fabs(pan_velocity));
    else if (fabs(pan_velocity) < min_pan_velocity_)
        pan_velocity = 0.0;

    if (fabs(tilt_velocity) > max_tilt_velocity_)
        tilt_velocity = max_tilt_velocity_ * (tilt_velocity / fabs(tilt_velocity));
    else if (fabs(tilt_velocity) < min_tilt_velocity_)
        tilt_velocity = 0.0;

    if ((pan_velocity == 0.0) && (tilt_velocity == 0.0))
    {
        publishZeroTorsoJointvelocities();
        current_state_ = IDLE;
    } else
    {
        publishTorsoJointvelocities(0.0, 0.0, pan_velocity, tilt_velocity);
        current_state_ = RUN;
    }

    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "look_at_point");

    LookAtPointNode look_at_point = LookAtPointNode();

    ros::Rate loop_rate_run(100);

    while (ros::ok())
    {
        ros::spinOnce();

        look_at_point.update();

        loop_rate_run.sleep();
    }

    return 0;
}
