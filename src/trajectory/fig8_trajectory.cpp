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

void Fig8Trajectory::eval(double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc, Eigen::Vector3d &jer)
{


	// 3D Circle
	// Vector3d pos(radius*sin(om*t), radius*cos(om*t), -sin(om*t));
	// Vector3d vel(om*radius*cos(om*t), -om*radius*sin(om*t), -om*cos(om*t));
	// Vector3d acc(-om*om*radius*sin(om*t), -om*om*radius*cos(om*t), om*om*sin(om*t));
	// Vector3d jer(-om*om*om*radius*cos(om*t), om*om*om*radius*sin(om*t), om*om*om*cos(om*t));

	// Horizontal Circle

	if (t <= start_delay) {
		pos = start_pos;
		vel.setZero();
		acc.setZero();
		jer.setZero();
	} else {
		t = t-start_delay;
		pos << radius_x*sin(om*t)+start_pos(0), radius_x-radius_x*cos(om*t)+start_pos(1), start_pos(2);
		vel << om*radius_x*cos(om*t), om*radius_x*sin(om*t), 0.0;
		acc << -om*om*radius_x*sin(om*t), om*om*radius_x*cos(om*t), 0.0;
		jer << -om*om*om*radius_x*cos(om*t), -om*om*om*radius_x*sin(om*t), 0.0;
	}

}

void Fig8Trajectory::set_start_pos(const Eigen::Vector3d &_start_pos)
{
	start_pos = _start_pos;
}
