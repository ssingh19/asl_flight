#include <iostream>
#include <control/se3controller.h>
#include <trajectory/trajectory.h>
#include "mavros_msgs/ActuatorControl.h"
#include <mavros_msgs/State.h>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>

#define TAKEOFF_TIME 5.0

// Measured states
std::string current_mode;
Eigen::Matrix3d mea_R;
Eigen::Vector3d mea_wb;
Eigen::Vector3d mea_pos;
Eigen::Vector3d mea_vel;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_mode = msg->mode;
}

void poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // By default, MAVROS gives local_position/pose in ENU
  mea_pos(0) = msg->pose.position.x;
  mea_pos(1) = -msg->pose.position.y;
  mea_pos(2) = -msg->pose.position.z;

  double q1 = msg->pose.orientation.w;
  double q2 = msg->pose.orientation.x;
  double q3 = -msg->pose.orientation.y;
  double q4 = -msg->pose.orientation.z;
  double q1s = q1*q1;
  double q2s = q2*q2;
  double q3s = q3*q3;
  double q4s = q4*q4;

  mea_R <<
    q1s+q2s-q3s-q4s, 2.0*(q2*q3-q1*q4) ,2.0*(q2*q4+q1*q3),
    2.0*(q2*q3+q1*q4), q1s-q2s+q3s-q4s, 2.0*(q3*q4-q1*q2),
    2.0*(q2*q4-q1*q3), 2.0*(q3*q4+q1*q2), q1s-q2s-q3s+q4s;

}

void velSubCB(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  // By default, MAVROS gives local_position/velocity in ENU
  mea_vel(0) = msg->twist.linear.x;
  mea_vel(1) = -msg->twist.linear.y;
  mea_vel(2) = -msg->twist.linear.z;
  mea_wb(0) = msg->twist.angular.x;
  mea_wb(1) = -msg->twist.angular.y;
  mea_wb(2) = -msg->twist.angular.z;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "asl_traj_ctrl");
  ros::NodeHandle nh;

  // Subscriptions
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, state_cb);
  ros::Subscriber poseSub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, poseSubCB);
  ros::Subscriber velSub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 1, velSubCB);

  // Actuator publisher
  ros::Publisher actuatorPub = nh.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 1);
  mavros_msgs::ActuatorControl cmd;

  // Define controller classes
  SE3Controller se3ctrl;

  // Define trajectory class
  Trajectory* traj = new CircleTrajectory(1.0, 2.0*3.14*(1.0/10.0));
  /*
    std::string TRAJ;
    nh.getParam("traj_type", traj_type);
    if (traj_type == "POLY")
    {
        traj = new PolyTrajectory(CALIB_END, POLY_SCALE);
    }
    else if (traj_type == "HOVER")
    {
        traj = new ConstTrajectory(12,0.0);
    }
    else if (traj_type == "CIRCLE")
    {
        VectorXd start_pos(3);
        traj = new CircleTrajectory(CALIB_END,1.0, 2.0*3.1416*(1/10.0), start_pos);
    }
  */

  // Controller frequency
  ros::Rate rate(250.0);

  // Tracking status variables
  bool traj_started = false;
  double time_prev = ros::Time::now().toSec();
  double time_traj = 0.0;
  double dt = 0.0;

  double yaw_des = 0.0;
  Eigen::Vector3d r_pos(0, 0, 0);
  Eigen::Vector3d r_vel(0, 0, 0);
  Eigen::Vector3d r_acc(0, 0, 0);
  Eigen::Vector3d r_jer(0, 0, 0);

  while(ros::ok()) {

    // Time loop calculations
    dt = ros::Time::now().toSec() - time_prev;
    time_prev = ros::Time::now().toSec();

    //ROS_INFO_STREAM("mode: " << current_mode);

    if (current_mode == "OFFBOARD") {
      if (!traj_started){
          traj_started = true;
          time_traj = 0.0;

          ROS_INFO("offboard started");

          // set takeoff hover point
          r_pos << mea_pos(0)+1.0, mea_pos(1), mea_pos(2)-1.0;

          // set start point for trajectory
          traj->set_start_pos(r_pos);
      } else {
          time_traj += dt;
      }
    } else {
      //reset
      traj_started = false;
      time_traj = 0.0;
    }

    // Once past initial wait time, overwrite reference
    if (time_traj > TAKEOFF_TIME) {
      ROS_INFO("error: %.2f", (r_pos-mea_pos).norm());
      traj->eval(time_traj-TAKEOFF_TIME, r_pos, r_vel, r_acc, r_jer);
    }
    // Update controller internal state
    se3ctrl.updatePose(mea_pos, mea_R);
    se3ctrl.updateVel(mea_vel, mea_wb);

    // compute control effort
    se3ctrl.calcSE3(yaw_des, r_pos, r_vel, r_acc, r_jer);

    // publish commands
    cmd.header.stamp = ros::Time::now();
    cmd.group_mix = 0;
    for(int i=0; i<4; i++) {
      cmd.controls[i] = se3ctrl.motorCmd[i];
    }
    cmd.controls[7] = 0.1234; // secret key to enabling direct motor control in px4
    actuatorPub.publish(cmd);

    //se3ctrl.joySE3();
    ros::spinOnce();
	  rate.sleep();
  }

  return 0;
}
