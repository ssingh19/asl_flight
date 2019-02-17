#include <trajectory/trajectory.h>
#include <math.h>
#include <ros/ros.h>

Fig8Trajectory::Fig8Trajectory(double _radius_x, double _radius_y, double _om, double _start_delay)
{
		start_delay = _start_delay;
		radius_x = _radius_x;
    radius_y = _radius_y;
		om = _om;
		start_pos << 0, 0, 0;
}

void Fig8Trajectory::eval(double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc, Eigen::Vector3d &jer,
													double &yaw, double &yaw_dot)
{
	// Horizontal Fig 8

	if (t <= start_delay) {
		pos = start_pos;
		vel.setZero();
		acc.setZero();
		jer.setZero();
		yaw = M_PI/2.0;
		yaw_dot = 0.0;
	} else {
		t = t-start_delay;
		pos <<  radius_x - radius_x*cos(om*t)+start_pos(0), radius_y*sin(2*om*t)+start_pos(1), start_pos(2);
		vel <<  om*radius_x*sin(om*t), radius_y*2*om*cos(2*om*t), 0.0;
		acc <<  om*om*radius_x*cos(om*t), -radius_y*4*om*om*sin(2*om*t), 0.0;
		jer << -om*om*om*radius_x*sin(om*t), -radius_y*8*om*om*om*cos(2*om*t), 0.0;

		yaw = (M_PI/2.0) - (3.0*M_PI/4.0)*sin(om*t);
		yaw_dot = -(3.0*M_PI/4.0)*om*cos(om*t);
	}

}

void Fig8Trajectory::set_start_pos(const Eigen::Vector3d &_start_pos)
{
	start_pos = _start_pos;
}
