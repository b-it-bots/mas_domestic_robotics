#ifndef HEAD_POINTING_KINEMATICS_H
#define HEAD_POINTING_KINEMATICS_H

#include <vector>

void calculate_camera_pan_tilt(double &pan, double &tilt,
		double x, double y, double z);

void set_pan_tilt_limits(double pan_limit, double tilt_limit);

bool is_camera_pan_tilt_limit_violated(double pan, double tilt);

void set_head_tilt_limits(double lower_limit, double upper_limit);

void calculate_torso_head_tilt(double &torso_tilt, double &head_tilt,
		double camera_tilt);

void calculate_joint_values(std::vector<double> &joint_values,
		double pan, double torso_tilt, double head_tilt);

#endif
