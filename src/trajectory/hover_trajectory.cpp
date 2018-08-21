#include <trajectory/trajectory.h>
#include <math.h>
#include <ros/ros.h>

HoverTrajectory::HoverTrajectory()
{
		start_pos << 0, 0, 0;
}

void HoverTrajectory::eval(double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc, Eigen::Vector3d &jer)
{
	pos = start_pos;
	vel.setZero();
	acc.setZero();
	jer.setZero();

}

void HoverTrajectory::set_start_pos(const Eigen::Vector3d &_start_pos)
{
	start_pos = _start_pos;
}
