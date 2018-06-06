#include <iostream>
#include <se3controller.h>
#include <mavros_msgs/State.h>
#include <string>

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "asl_traj_ctrl");
  ros::NodeHandle nh;

  // Main state subscription
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, state_cb);

  SE3Controller se3ctrl;

  ros::Rate rate(200.0);

  Eigen::Vector3d r_euler(0, 0, 0);
  Eigen::Vector3d r_wb(0, 0, 0);
  Eigen::Vector3d r_pos(1.0, 0, -1.0);
  Eigen::Vector3d r_vel(0, 0, 0);
  Eigen::Vector3d r_acc(0, 0, 0);

  while(ros::ok()) {
    if (current_state.mode == "OFFBOARD") {
      se3ctrl.calcSE3(r_euler, r_wb, r_pos, r_vel, r_acc);
    }

    //se3ctrl.joySE3();
    ros::spinOnce();
	  rate.sleep();
  }

  return 0;
}
