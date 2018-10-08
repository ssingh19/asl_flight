#include <iostream>
#include <control/se3controller.h>
#include <control/ccmcontroller.h>
#include <trajectory/trajectory.h>
#include <std_msgs/Float64.h>
#include "mavros_msgs/ActuatorControl.h"
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <GeoProb/GeoProb.h>
#include <GeoProb/Geodesic.h>
#include <GeoProb/Metric.h>

#define TAKEOFF_TIME 5.0

#define TAKEOFF_HGT 1.0

// Measured states
std::string current_mode;
Eigen::Matrix3d mea_R;
Eigen::Vector3d mea_wb;
Eigen::Vector3d mea_pos;
Eigen::Vector3d mea_vel;
Eigen::Vector3d vel_prev;
double vel_prev_t;
double fz_est;
double fz_cmd;

// Update variables
int pose_up;
int vel_up;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_mode = msg->mode;
}

void poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // By default, MAVROS gives local_position/pose in ENU (x-dir preserved)
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

  pose_up = 1;

}

void velSubCB(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  // By default, MAVROS gives local_position/velocity in ENU

  vel_prev = mea_vel;

  double vel_dt = (msg->header.stamp.sec + msg->header.stamp.nsec*(1.0e-9))-
                   vel_prev_t;
  vel_prev_t = msg->header.stamp.sec + msg->header.stamp.nsec*(1.0e-9);

  mea_vel(0) = msg->twist.linear.x;
  mea_vel(1) = -msg->twist.linear.y;
  mea_vel(2) = -msg->twist.linear.z;
  mea_wb(0) = msg->twist.angular.x;
  mea_wb(1) = -msg->twist.angular.y;
  mea_wb(2) = -msg->twist.angular.z;

  //update accel estimate
  Eigen::Vector3d acc = (mea_vel - vel_prev)/vel_dt;
  acc(2) += -9.8066;
  double fz_est_up = -acc.dot(mea_R.col(2));
  double fz_avg = 0.5*(fz_est+fz_cmd);
  fz_est =  std::abs((fz_est_up-fz_avg)/fz_avg)>=0.2 ? fz_cmd : fz_est_up;

  vel_up = 1;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "asl_traj_ctrl");
  ros::NodeHandle nh;

  // Subscriptions
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, state_cb);
  ros::Subscriber poseSub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, poseSubCB);
  ros::Subscriber velSub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 1, velSubCB);

  pose_up = 0; vel_up = 0;

  // Actuator publisher
  ros::Publisher actuatorPub = nh.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 1);
  mavros_msgs::ActuatorControl cmd;

  // DEBUG PUBLICATIONS
  ros::Publisher debug_pub1 = nh.advertise<std_msgs::Float64>("/debug1", 1);
  ros::Publisher debug_pub2 = nh.advertise<std_msgs::Float64>("/debug2", 1);
  ros::Publisher debug_pub3 = nh.advertise<std_msgs::Float64>("/debug3", 1);
  ros::Publisher debug_pub4 = nh.advertise<std_msgs::Float64>("/debug4", 1);
  ros::Publisher debug_pub5 = nh.advertise<std_msgs::Float64>("/debug5", 1);
  ros::Publisher debug_pub6 = nh.advertise<std_msgs::Float64>("/debug6", 1);
  ros::Publisher debug_pub7 = nh.advertise<std_msgs::Float64>("/debug7", 1);
  ros::Publisher debug_pub8 = nh.advertise<std_msgs::Float64>("/debug8", 1);
  ros::Publisher debug_pub9 = nh.advertise<std_msgs::Float64>("/debug9", 1);
  ros::Publisher debug_pub10 = nh.advertise<std_msgs::Float64>("/debug10", 1);
  ros::Publisher debug_pub11 = nh.advertise<std_msgs::Float64>("/debug11", 1);
  ros::Publisher debug_pub12 = nh.advertise<std_msgs::Float64>("/debug12", 1);
  std_msgs::Float64 debug_msg;

  // Define controller classes
  // SE3Controller ctrl;
  CCMController ctrl;


  // Define trajectory class
  std::string traj_type;
  ros::param::get("~TRAJ", traj_type);
  Trajectory* traj;
  // if (traj_type == "POLY")
  // {
  //     traj = new PolyTrajectory(CALIB_END, POLY_SCALE);
  // }
  if (traj_type == "HOVER")
  {
      traj = new HoverTrajectory();
  }
  else
  {
      traj = new CircleTrajectory(1.0, 2.0*3.14*(1.0/10.0));
  }


  // Controller frequency
  ros::Rate rate(250.0);

  // Tracking status variables
  bool traj_started = false;
  double time_prev = ros::Time::now().toSec();
  double time_traj = 0.0;
  double dt = 0.0;

  // Reference values
  double yaw_des = 0.0;
  Eigen::Vector3d r_pos(0, 0, 0);
  Eigen::Vector3d r_vel(0, 0, 0);
  Eigen::Vector3d r_acc(0, 0, 0);
  Eigen::Vector3d r_jer(0, 0, 0);
  Eigen::Vector3d takeoff_loc(0, 0, 0);

  // CCM values
  fz_est = 9.8066;
  fz_cmd = 9.8066;
  Eigen::Vector3d euler;
  Eigen::Vector3d euler_dot;
  vel_prev_t = 0.0;

  while(ros::ok()) {
    // Check for state update
    ros::spinOnce();

    debug_msg.data = r_pos(0);
    debug_pub1.publish(debug_msg);
    debug_msg.data = mea_pos(0);
    debug_pub2.publish(debug_msg);

    debug_msg.data = r_pos(1);
    debug_pub3.publish(debug_msg);
    debug_msg.data = mea_pos(1);
    debug_pub4.publish(debug_msg);

    debug_msg.data = r_pos(2);
    debug_pub5.publish(debug_msg);
    debug_msg.data = mea_pos(2);
    debug_pub6.publish(debug_msg);

    // Time loop calculations
    dt = ros::Time::now().toSec() - time_prev;
    time_prev = ros::Time::now().toSec();

    // Compare measured and commanded thrust
    fz_cmd = ctrl.getfz();

    if (current_mode == "OFFBOARD") {

      // update trajectory status variables
      if (!traj_started){

          traj_started = true;
          ctrl.setMode(traj_started);

          time_traj = 0.0;

          ROS_INFO("offboard started");

          // set takeoff location
          takeoff_loc << mea_pos(0), mea_pos(1), mea_pos(2);

          // initialize ref pos and ref vel
          r_pos = takeoff_loc;
          r_vel << 0.0, 0.0, -(TAKEOFF_HGT/TAKEOFF_TIME);

          // set start point for trajectory (hover point above takeoff)
          traj->set_start_pos(takeoff_loc + Eigen::Vector3d(0.0,0.0,-TAKEOFF_HGT));
      } else {
          time_traj += dt;
      }
    } else {
      //reset
      traj_started = false;
      time_traj = 0.0;
      ctrl.setMode(traj_started);
      r_vel.setZero(); r_acc.setZero(); r_jer.setZero();
    }

    // Compute nominal
    if (time_traj >= TAKEOFF_TIME) {
      // Once past takeoff time, start trajectory
      ROS_INFO("error: %.3f", (r_pos-mea_pos).norm());
      traj->eval(time_traj-TAKEOFF_TIME+dt, r_pos, r_vel, r_acc, r_jer);
    } else {
      // smooth takeoff
      r_pos(2) = takeoff_loc(2)-(time_traj/TAKEOFF_TIME)*TAKEOFF_HGT;
      //r_vel(2) = (TAKEOFF_HGT/std::pow(TAKEOFF_TIME,2.0))*(time_traj-TAKEOFF_TIME);
      //r_acc << 0.0, 0.0,(TAKEOFF_HGT/std::pow(TAKEOFF_TIME,2.0));
    }
    // Update controller internal state
    ctrl.updateState(mea_pos, mea_R, mea_vel, mea_wb, fz_est, dt, pose_up, vel_up);
    ctrl.calcCCM(yaw_des, r_pos, r_vel, r_acc, r_jer);

    // ctrl.updateVel(mea_vel, mea_wb, dt);
    // ctrl.updatePose(mea_pos, mea_R);
    // ctrl.calcSE3(yaw_des, r_pos, r_vel, r_acc, r_jer);


    // publish commands
    cmd.header.stamp = ros::Time::now();
    cmd.group_mix = 0;
    for(int i=0; i<4; i++) {
      cmd.controls[i] = ctrl.motorCmd[i];
    }
    cmd.controls[7] = 0.1234; // secret key to enabling direct motor control in px4
    actuatorPub.publish(cmd);

    // Publish

    euler_dot = ctrl.getEulerdot();
    euler = ctrl.getEuler();
    debug_msg.data = ctrl.getE();
    debug_pub7.publish(debug_msg);

    debug_msg.data = euler_dot(0);
    debug_pub8.publish(debug_msg);
    debug_msg.data = euler_dot(1);
    debug_pub9.publish(debug_msg);
    debug_msg.data = euler_dot(2);
    debug_pub10.publish(debug_msg);

    debug_msg.data = fz_cmd;
    debug_pub11.publish(debug_msg);
    debug_msg.data = fz_est;
    debug_pub12.publish(debug_msg);

    // Reset update
    pose_up = 0; vel_up = 0;
    // Check for state update
    ros::spinOnce();

	  rate.sleep();
  }

  return 0;
}
