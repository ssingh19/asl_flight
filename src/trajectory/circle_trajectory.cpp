#include <trajectory/trajectory.h>
#include <math.h>
#include <ros/ros.h>

CircleTrajectory::CircleTrajectory(double _radius, double _om, double _start_delay)
{
		start_delay = _start_delay;
		radius = _radius;
		om = _om;
		start_pos << 0, 0, 0;
}

void CircleTrajectory::eval(double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc, Eigen::Vector3d &jer,
														double &yaw, double &yaw_dot)
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
		yaw = 0.0;
		yaw_dot = 0.0;
	} else {
		t = t-start_delay;

		double t_hat = t-5;

		double g = 1.0/(1+std::exp(-t_hat));
		double gd = g*(1-g);
		double gdd = gd*(1-g) - g*gd;
		double gddd = gdd*(1-g) - 2*(gd*gd) - g*gdd;

		double r = radius*g;
		double rd = radius*gd;
		double rdd = radius*gdd;
		double rddd = radius*gddd;

		pos << start_pos(0), r-r*cos(om*t)+start_pos(1), r*sin(om*t)+start_pos(2);
		vel << 0.0, -rd*cos(om*t)+r*sin(om*t)*om, rd*sin(om*t)+r*cos(om*t)*om;

		acc << 0.0,
				  -rdd*cos(om*t)+rd*sin(om*t)*om+rd*sin(om*t)*om+r*cos(om*t)*(om*om),
					 rdd*sin(om*t)+rd*cos(om*t)*om+rd*cos(om*t)*om-r*sin(om*t)*(om*om);

		jer << 0.0,
				  -rddd*cos(om*t)+rdd*sin(om*t)*om+rdd*sin(om*t)*om+rd*cos(om*t)*(om*om)+rdd*sin(om*t)*om+rd*cos(om*t)*(om*om)+rd*cos(om*t)*(om*om)-r*sin(om*t)*pow(om,3.0),
					 rddd*cos(om*t)+rdd*cos(om*t)*om+rdd*cos(om*t)*om-rd*sin(om*t)*(om*om)+rdd*cos(om*t)*om-rd*sin(om*t)*(om*om)-rd*sin(om*t)*(om*om)-r*cos(om*t)*pow(om,3.0);

		yaw = 0.0;
		yaw_dot = 0.0;
	}

}

void CircleTrajectory::set_start_pos(const Eigen::Vector3d &_start_pos)
{
	start_pos = _start_pos;
}
