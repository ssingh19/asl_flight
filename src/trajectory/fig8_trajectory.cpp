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

		ROS_INFO("radius_x: %.3f, radius_y:%.3f", radius_x, radius_y);
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
		yaw = 0.0;
		yaw_dot = 0.0;
	} else {
		t = t-start_delay;

		double om_ = om;
		double t_ = 0.0;

		if (t < 2.0*2.0*M_PI/om) {
			om_ = (1.0/2.0)*om;
		} else if (t >= 2.0*2.0*M_PI/om && t < (2.0+1.5)*2.0*M_PI/om) {
			om_ = (1.0/1.5)*om;
			t_ = 2.0*2.0*M_PI/om;
		} else {
			t_ = 3.5*2.0*M_PI/om;
		}

		t = t - t_;

		pos <<  radius_x - radius_x*cos(om_*t)+start_pos(0), radius_y*sin(2*om_*t)+start_pos(1), start_pos(2);
		vel <<  om_*radius_x*sin(om_*t), radius_y*2*om_*cos(2*om_*t), 0.0;
		acc <<  om_*om_*radius_x*cos(om_*t), -radius_y*4*om_*om_*sin(2*om_*t), 0.0;
		jer << -om_*om_*om_*radius_x*sin(om_*t), -radius_y*8*om_*om_*om_*cos(2*om_*t), 0.0;

		yaw = -(3.0*M_PI/4.0)*sin(om_*t);
		yaw_dot = -(3.0*M_PI/4.0)*om_*cos(om_*t);
		// yaw = 0.0;
		// yaw_dot = 0.0;
	}

}

void Fig8Trajectory::set_start_pos(const Eigen::Vector3d &_start_pos)
{
	start_pos = _start_pos;
}
