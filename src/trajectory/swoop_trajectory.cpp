#include <trajectory/trajectory.h>
#include <math.h>
#include <ros/ros.h>

SwoopTrajectory::SwoopTrajectory(double _ybar, double _delta1, double _delta2, double _zbar, double _start_delay):
y_bar(_ybar),delta_1(_delta1),delta_2(_delta2),z_bar(_zbar){

    start_delay = _start_delay;
		start_pos << 0, 0, 0;
}

void SwoopTrajectory::eval(double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc, Eigen::Vector3d &jer,
														double &yaw, double &yaw_dot)
{


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

		double y = y_bar * g;
    double yd = y_bar * gd;
    double ydd = y_bar * gdd;
    double yddd = y_bar * gddd;

    double z, zd, zdd, zddd;

    if (y<=delta_1){

      z = -pow(y-delta_1,2.0) + z_bar;
      zd = -2*(y-delta_1)*yd;
      zdd = -2*yd*yd - 2*(y-delta_1)*ydd;
      zddd = -4*yd*ydd - 2*yd*ydd - 2*(y-delta_1)*yddd;

      pos << start_pos(0), start_pos(1) + y, z;
      vel << 0.0, yd, zd;
      acc << 0.0, ydd, zdd;
      jer << 0.0, yddd, zddd;

    } else if (y>delta_1 && y<= delta_2) {

      pos << start_pos(0), start_pos(1) + y, z_bar;
      vel << 0.0, yd , 0.0;
      acc << 0.0, ydd, 0.0;
      jer << 0.0, yddd, 0.0;

    } else {

      z = -pow(y-delta_2,2.0) + z_bar;
      zd = -2*(y-delta_2)*yd;
      zdd = -2*yd*yd - 2*(y-delta_2)*ydd;
      zddd = -4*yd*ydd - 2*yd*ydd - 2*(y-delta_2)*yddd;

      pos << start_pos(0), start_pos(1) + y, z;
      vel << 0.0, yd, zd;
      acc << 0.0, ydd, zdd;
      jer << 0.0, yddd, zddd;

    }

		yaw = 0.0;
		yaw_dot = 0.0;
	}

}

void SwoopTrajectory::set_start_pos(const Eigen::Vector3d &_start_pos)
{
	start_pos = _start_pos;
}
