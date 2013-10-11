#include <head_kinematics.h>


void calculate_camera_pan_tilt(double &pan, double &tilt,
		double x, double y, double z)
{
	pan = 0.0;
	tilt = 0.0;
}


void set_pan_tilt_limits(double pan_limit, double tilt_limit)
{
}


bool is_camera_pan_tilt_limit_violated(double pan, double tilt)
{
	return true;
}


void set_head_tilt_limits(double lower_limit, double upper_limit)
{
}


void calculate_torso_head_tilt(double &torso_tilt, double &head_tilt,
		double camera_tilt)
{
	torso_tilt = 0.0;
	head_tilt = 0.0;
}


void calculate_joint_values(std::vector<double> &joint_values,
		double pan, double torso_tilt, double head_tilt)
{
	joint_values.clear();
}
