/*
 * follow_person_node.cpp
 *
 *  Created on: Jan 25, 2011
 *      Author: Frederik Hegger
 *
 */

#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>

#include <mcr_algorithms/controller/pi_controller.h>
#include <mcr_perception_msgs/Person.h>
#include <mcr_perception_msgs/PersonList.h>

enum EnumBehaviorState
{
    eBehaviorStateInit = 0,
    eBehaviorStateRun = 1,
    eBehaviorStateIdle = 2
};

class PersonFollowBehavior
{
public:
    PersonFollowBehavior(ros::NodeHandle &nh);
    ~PersonFollowBehavior();

    void personTrackerCallback(const mcr_perception_msgs::PersonList::ConstPtr &inputPersonList);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr &odom_msgs);
    void nextStep();

private:
    bool start(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool stop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool pause(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool resume(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    ros::NodeHandle *_nh;
    EnumBehaviorState _eState;
    ros::ServiceServer _srvStart;
    ros::ServiceServer _srvStop;
    ros::ServiceServer _srvPause;
    ros::ServiceServer _srvResume;

    PIController* pi_controller_x_;
    PIController* pi_controller_theta_;
    tf::TransformListener* _pTransformListener;
    mcr_perception_msgs::PersonList _personList;
    nav_msgs::Odometry current_odom_;
    double _dMaxTranslationalVelocity;
    double _dMaxRotationalVelocity;
    double _dTranslationalVelocityToSet;
    double _dRotationalVelocityToSet;
    double _dTranslationalDeaccelerationFactor;
    double _dRotationalDeaccelerationFactor;
    double _dDistanceToHuman;
    double _dOrientationToHuman;
    double _dMinimumDistanceToPerson;
    double _dAllowedAngularVariance;
    bool _bPersonFound;
    bool _bIsTrackedPersonOccluded;
    unsigned int _unUserToTrack;
    ros::Subscriber _subPersonTracker;
    ros::Subscriber _subOdom;
    ros::Publisher _pubBaseCommands;
    geometry_msgs::Twist _baseCmd;
    double _dLastSpeed;
    bool _is_driving_backward_enabled;
    double _distance_when_driving_backward;
};

PersonFollowBehavior::PersonFollowBehavior(ros::NodeHandle &nh)
{
    this->_eState = eBehaviorStateInit;
    this->_nh = &nh;

    this->_srvStart = this->_nh->advertiseService("start", &PersonFollowBehavior::start, this);
    this->_srvStop = this->_nh->advertiseService("stop", &PersonFollowBehavior::stop, this);
    this->_srvPause = this->_nh->advertiseService("pause", &PersonFollowBehavior::pause, this);
    this->_srvResume = this->_nh->advertiseService("resume", &PersonFollowBehavior::resume, this);

    this->_eState = eBehaviorStateInit;

    // subscribe and advertise
    this->_subPersonTracker = this->_nh->subscribe < mcr_perception_msgs::PersonList
                              > ("people_positions", 1, &PersonFollowBehavior::personTrackerCallback, this);
    this->_subOdom = this->_nh->subscribe < nav_msgs::Odometry > ("odometry", 1, &PersonFollowBehavior::odometryCallback, this);
    this->_pubBaseCommands = this->_nh->advertise < geometry_msgs::Twist > ("cmd_vel", 1);

    // init Transformation Listener
    this->_pTransformListener = new tf::TransformListener();

    // get parameters from server
    if (!this->_nh->getParam("max_translational_velocity", this->_dMaxTranslationalVelocity))
    {
        this->_dMaxTranslationalVelocity = 0.2;
        ROS_WARN("Parameter max_translational_velocity not specified, use default value for COB3: %lf", this->_dMaxTranslationalVelocity);
    }

    if (!this->_nh->getParam("max_rotational_velocity", this->_dMaxRotationalVelocity))
    {
        this->_dMaxRotationalVelocity = 0.2;
        ROS_WARN("Parameter max_rotational_velocity not specified, use default value for COB3: %lf", this->_dMaxRotationalVelocity);
    }
    if (!this->_nh->getParam("minimum_distance_to_person", this->_dMinimumDistanceToPerson))
    {
        this->_dMinimumDistanceToPerson = 1.0;
        ROS_WARN("Parameter minimum_distance_to_person not specified, use default value: %lf", this->_dMinimumDistanceToPerson);
    }
    if (!this->_nh->getParam("translational_deacceleration_factor", this->_dTranslationalDeaccelerationFactor))
    {
        this->_dTranslationalDeaccelerationFactor = 0.5;
        ROS_WARN("Parameter translational_deacceleration_factor not specified, use default value for COB3: %lf", this->_dTranslationalDeaccelerationFactor);
    }
    if (!this->_nh->getParam("rotational_deacceleration_factor", this->_dRotationalDeaccelerationFactor))
    {
        this->_dRotationalDeaccelerationFactor = 1.0;
        ROS_WARN("Parameter rotational_deacceleration_factor not specified, use default value for COB3: %lf", this->_dRotationalDeaccelerationFactor);
    }

    if (!this->_nh->getParam("allowed_angular_variance", this->_dAllowedAngularVariance))
    {
        this->_dAllowedAngularVariance = 5.0;
        ROS_WARN("Parameter allowed_angular_variance not specified, use default value for COB3: %lf", this->_dAllowedAngularVariance);
    }

    if (!this->_nh->getParam("driving_backward_enabled", this->_is_driving_backward_enabled))
    {
        this->_is_driving_backward_enabled = false;
        ROS_WARN("Parameter driving_backward_enabled not specified, use default value for COB3: %d", this->_is_driving_backward_enabled);
    }

    if (!this->_nh->getParam("distance_when_driving_backward", this->_distance_when_driving_backward))
    {
        this->_distance_when_driving_backward = 0.9;
        ROS_WARN("Parameter distance_when_driving_backward not specified, use default value for COB3: %lf", this->_distance_when_driving_backward);
    }

    //convert from degree to radian
    this->_dAllowedAngularVariance = this->_dAllowedAngularVariance / 180 * M_PI;

    this->_bPersonFound = false;
    this->_bIsTrackedPersonOccluded = false;
    this->_unUserToTrack = -1;

    //init pd controller
    this->pi_controller_x_ = new PIController(0.01, 0.01, 20);
    this->pi_controller_theta_ = new PIController(1, 0.00, 20);
}

PersonFollowBehavior::~PersonFollowBehavior()
{
    this->_srvStart.shutdown();
    this->_srvStop.shutdown();
    this->_srvPause.shutdown();
    this->_srvResume.shutdown();

    if (this->_pTransformListener != NULL)
        delete this->_pTransformListener;
    if (this->pi_controller_x_ != NULL)
        delete this->pi_controller_x_;
    if (this->pi_controller_theta_ != NULL)
        delete this->pi_controller_theta_;
}

bool PersonFollowBehavior::start(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    //if(this->_eState == eBehaviorStateInit)
    //{
    ROS_INFO("behavior started");
    this->_eState = eBehaviorStateRun;
    return true;
    //}
    //else
    //{
    //  ROS_ERROR("illegal state transition");
    //  return false;
    //}
}
;

bool PersonFollowBehavior::stop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    //if(this->_eState == eBehaviorStateRun)
    //{
    ROS_INFO("behavior stopped");
    this->_eState = eBehaviorStateInit;
    return true;
    /*}
     else
     {
     ROS_ERROR("illegal state transition");
     return false;
     }*/
}
;

bool PersonFollowBehavior::pause(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    //if(this->_eState == eBehaviorStateRun)
    //{
    ROS_INFO("behavior paused");
    this->_eState = eBehaviorStateIdle;
    return true;
    /*}
     else
     {
     ROS_ERROR("illegal state transition");
     return false;
     }*/
}
;

bool PersonFollowBehavior::resume(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    //if(this->_eState == eBehaviorStateIdle)
    //{
    ROS_INFO("behavior resumed");
    this->_eState = eBehaviorStateRun;
    return true;
    /*}
     else
     {
     ROS_ERROR("illegal state transition");
     return false;
     }*/
}
;

void PersonFollowBehavior::nextStep()
{
    if (this->_eState == eBehaviorStateInit)
    {
        this->_bPersonFound = false;

        this->_dTranslationalVelocityToSet = 0.0;
        this->_dRotationalVelocityToSet = 0.0;
    }

    // human not found -> search for it
    if (this->_eState == eBehaviorStateRun)
    {
        // if no person is assign for following, search for a candidate in a specific distance range (AND ANGULAR RANGE !!!????)
        if (!this->_bPersonFound)
        {
            for (unsigned int i = 0; i < this->_personList.persons.size(); ++i)
            {
                //if ( this->_dDistanceToHuman < this->_dDistanceRangeForSearching )
                if (this->_personList.persons[i].is_tracked)
                {
                    this->_unUserToTrack = this->_personList.persons[i].id;
                    this->_bPersonFound = true;
                    this->_bIsTrackedPersonOccluded = this->_personList.persons[i].is_occluded;
                    ROS_INFO("Person found in Range. Start tracking person with ID = %d", this->_unUserToTrack);
                    break;
                }
            }

            this->_dTranslationalVelocityToSet = 0.0;
            this->_dRotationalVelocityToSet = 0.0;
        }

        else
        {
            this->_dDistanceToHuman = 0.0;
            this->_dOrientationToHuman = 0.0;
            this->_dTranslationalVelocityToSet = 0.0;
            this->_dRotationalVelocityToSet = 0.0;

            for (unsigned int i = 0; i < this->_personList.persons.size(); ++i)
            {
                if (this->_personList.persons[i].id == this->_unUserToTrack)
                {
                    if (this->_personList.persons[i].pose.pose.position.x == 0 && this->_personList.persons[i].pose.pose.position.y == 0)
                    {
                        this->_bPersonFound = false;
                        continue;
                    }

                    // calculate euclidean distance to tracked person
                    this->_dDistanceToHuman = sqrt(
                                                  (this->_personList.persons[i].pose.pose.position.x * this->_personList.persons[i].pose.pose.position.x)
                                                  + (this->_personList.persons[i].pose.pose.position.y * this->_personList.persons[i].pose.pose.position.y));
                    // actual orientation to tracked person
                    this->_dOrientationToHuman = atan(this->_personList.persons[i].pose.pose.position.y / this->_personList.persons[i].pose.pose.position.x);

                    this->_bIsTrackedPersonOccluded = this->_personList.persons[i].is_occluded;

                    break;
                }
            }

            // rotational velocities
            if (this->_dOrientationToHuman <= this->_dAllowedAngularVariance && this->_dOrientationToHuman >= -this->_dAllowedAngularVariance)
                this->_dRotationalVelocityToSet = 0.0;
            if (this->_bIsTrackedPersonOccluded)
                this->_dRotationalVelocityToSet = 0.0;
            else
            {
                double dFactor = this->_dRotationalDeaccelerationFactor * this->_dOrientationToHuman;

                if (dFactor > this->_dMaxRotationalVelocity)
                    dFactor = this->_dMaxRotationalVelocity;

                if (dFactor < -this->_dMaxRotationalVelocity)
                    dFactor = -this->_dMaxRotationalVelocity;

                this->_dRotationalVelocityToSet = dFactor;
            }

            // translational velocities
            if (this->_bIsTrackedPersonOccluded)
            {
                // person starts to be occluded -> stop
                std_msgs::Empty msg;
                this->_dTranslationalVelocityToSet = 0.0;
            }

            else if (this->_dDistanceToHuman < this->_distance_when_driving_backward && this->_is_driving_backward_enabled)  // drive backward if person is to close
            {
                double dFactor = this->_dTranslationalDeaccelerationFactor * (this->_dDistanceToHuman - this->_dMinimumDistanceToPerson);
                this->_dTranslationalVelocityToSet = dFactor < this->_dMaxTranslationalVelocity ? dFactor : -this->_dMaxTranslationalVelocity;
            }

            else if (this->_dDistanceToHuman < this->_dMinimumDistanceToPerson)  // person in desired range -> stop moving
                this->_dTranslationalVelocityToSet = 0.0;

            else    // person to far away -> drive forward
            {
                double dFactor = this->_dTranslationalDeaccelerationFactor * (this->_dDistanceToHuman - this->_dMinimumDistanceToPerson);
                this->_dTranslationalVelocityToSet = dFactor < this->_dMaxTranslationalVelocity ? dFactor : this->_dMaxTranslationalVelocity;
            }

            // PI Controller
            this->_dTranslationalVelocityToSet = this->pi_controller_x_->control(this->current_odom_.twist.twist.linear.x, this->_dTranslationalVelocityToSet);
            this->_dRotationalVelocityToSet = this->pi_controller_theta_->control(this->current_odom_.twist.twist.angular.z, this->_dRotationalVelocityToSet);
        }
    }

    // pause following
    else if (this->_eState == eBehaviorStateIdle)
    {
        // set speeds to zero
        this->_dTranslationalVelocityToSet = 0.0;
        this->_dRotationalVelocityToSet = 0.0;
    }

    if (fabs(this->_dTranslationalVelocityToSet) < 0.01)
        this->_dTranslationalVelocityToSet = 0.0;

    if (fabs(this->_dRotationalVelocityToSet) < 0.01)
        this->_dRotationalVelocityToSet = 0.0;

    // setup data structure and publish the velocities
    this->_baseCmd.linear.x = this->_dTranslationalVelocityToSet;
    this->_baseCmd.linear.y = 0.0;
    this->_baseCmd.angular.z = this->_dRotationalVelocityToSet;

    if (_dLastSpeed != 0 || (fabs(this->_dTranslationalVelocityToSet) + fabs(this->_dRotationalVelocityToSet) > 0))
    {
        this->_pubBaseCommands.publish(this->_baseCmd);
    }

    _dLastSpeed = fabs(this->_dTranslationalVelocityToSet) + fabs(this->_dRotationalVelocityToSet);
}

void PersonFollowBehavior::personTrackerCallback(const mcr_perception_msgs::PersonList::ConstPtr& inputPersonList)
{
    this->_personList = *inputPersonList;
}

void PersonFollowBehavior::odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    this->current_odom_ = *odom_msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "follow_person");
    ros::NodeHandle nh("~");

    PersonFollowBehavior* pFollowBehavior = new PersonFollowBehavior(nh);

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        pFollowBehavior->nextStep();

        ros::spinOnce();
        loop_rate.sleep();
    }

    if (pFollowBehavior != NULL)
        delete pFollowBehavior;

    return 0;
}

