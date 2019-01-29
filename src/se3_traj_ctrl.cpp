#include <iostream>
#include <control/se3controller.h>
#include <trajectory/trajectory.h>
#include <std_msgs/Float64.h>
#include "mavros_msgs/Thrust.h"
#include "mavros_msgs/ActuatorControl.h"
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <utils/utils.h>

#define TAKEOFF_TIME 5.0
#define THROTTLE_SCALE 0.51
#define TAKEOFF_HGT 1.0

// Measured states
std::string current_mode;
Eigen::Vector4d mea_q;
Eigen::Matrix3d mea_R;
Eigen::Vector3d mea_wb;
Eigen::Vector3d mea_pos;
Eigen::Vector3d mea_vel;
Eigen::Vector3d vel_prev;
double vel_prev_t;
double fz_est;
double fz_cmd;
static Eigen::Matrix<double,3,3> Rz_T =
(Eigen::Matrix<double,3,3>() << 0.0, 1.0, 0.0,
                               -1.0, 0.0, 0.0,
                                0.0, 0.0, 1.0).finished();

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

  // ROS quaternion
  mea_q(0) = msg->pose.orientation.w;
  mea_q(1) = msg->pose.orientation.x;
  mea_q(2) = msg->pose.orientation.y;
  mea_q(3) = msg->pose.orientation.z;

  // Convert to Matrix
  utils::quat2rotM(mea_q, mea_R);

  // Adjust by R_z
  mea_R = mea_R * Rz_T;

  // Extract ENU encoding of PX4 quat
  utils::rotM2quat(mea_q, mea_R);

  // Convert to NED
  double q_w = mea_q(0);
  double q_x = mea_q(2);
  double q_y = mea_q(1);
  double q_z = -mea_q(3);

  mea_q << q_w, q_x, q_y, q_z;


  // ROS_INFO("pos: (%.4f, %.4f, %.4f), quat: (%.4f, %.4f: %.4f, %.4f)", mea_pos(0), mea_pos(1), mea_pos(2),
  //                                                                     q_w, q_x,q_y, q_z);

  // Final conversion to rot matrix
  utils::quat2rotM(mea_q, mea_R);

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
  ros::init(argc, argv, "se3_traj_ctrl");
  ros::NodeHandle nh;

  // Subscriptions
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
  ros::Subscriber poseSub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, poseSubCB);
  ros::Subscriber velSub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 1, velSubCB);

  pose_up = 0; vel_up = 0;

  // Actuator publisher
  ros::Publisher actuatorPub = nh.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 1);
  //ros::Publisher omegaPub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_attitude/cmd_vel", 2);
  //ros::Publisher throttlePub = nh.advertise<mavros_msgs::Thrust>("mavros/setpoint_attitude/thrust", 2);

  mavros_msgs::ActuatorControl cmd;
  // geometry_msgs::TwistStamped omega_sp;
  // mavros_msgs::Thrust throttle_sp;

  // DEBUG PUBLICATIONS
  ros::Publisher debug_pub1 = nh.advertise<std_msgs::Float64>("/debug1", 1);
  ros::Publisher debug_pub2 = nh.advertise<std_msgs::Float64>("/debug2", 1);
  ros::Publisher debug_pub3 = nh.advertise<std_msgs::Float64>("/debug3", 1);
  ros::Publisher debug_pub4 = nh.advertise<std_msgs::Float64>("/debug4", 1);
  ros::Publisher debug_pub5 = nh.advertise<std_msgs::Float64>("/debug5", 1);
  ros::Publisher debug_pub6 = nh.advertise<std_msgs::Float64>("/debug6", 1);
  ros::Publisher debug_pub7 = nh.advertise<std_msgs::Float64>("/debug7", 1);
  ros::Publisher debug_pub8 = nh.advertise<std_msgs::Float64>("/debug8", 1);

  std_msgs::Float64 debug_msg;

  // Define controller classes
  SE3Controller ctrl;

  // Define trajectory class
  std::string traj_type;
  ros::param::get("~TRAJ", traj_type);
  double circle_T;
  ros::param::get("~CIRCLE_T", circle_T);
  double poly_scale;
  ros::param::get("~POLY_SCALE", poly_scale);
  double start_delay;
  ros::param::get("~START_DELAY", start_delay);

  Trajectory* traj;

  if (traj_type == "HOVER")  {

    traj = new HoverTrajectory();

  } else if (traj_type == "POLY") {

    traj = new PolyTrajectory(start_delay,poly_scale);

  }  else  {

    traj = new CircleTrajectory(1.0, 2.0*3.14*(1.0/circle_T),start_delay);

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
  vel_prev_t = 0.0;

  while(ros::ok()) {
    // Check for state update
    ros::spinOnce();

    // Publish tracking results
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

    // Get commanded normalized thrust
    fz_cmd = ctrl.getfz();

    debug_msg.data = fz_cmd;
    debug_pub7.publish(debug_msg);
    debug_msg.data = fz_est;
    debug_pub8.publish(debug_msg);

    // Time loop calculations
    dt = ros::Time::now().toSec() - time_prev;
    time_prev = ros::Time::now().toSec();

    if (current_mode == "OFFBOARD") {

      // update trajectory status variables
      if (!traj_started){

          traj_started = true;

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
    }
    // Update controller internal state
    ctrl.updateState(mea_pos, mea_R, mea_vel, mea_wb, fz_est, dt, pose_up, vel_up);
    ctrl.calcSE3(yaw_des, r_pos, r_vel, r_acc, r_jer);

    // publish commands
    cmd.header.stamp = ros::Time::now();
    cmd.group_mix = 0;
    for(int i=0; i<4; i++) {
      cmd.controls[i] = ctrl.motorCmd[i];
    }
    cmd.controls[7] = 0.1234; // secret key to enabling direct motor control in px4
    actuatorPub.publish(cmd);
    /*
    fz_cmd = ctrl.getfz();
    euler_dot = ctrl.getEulerdot();

    throttle_sp.header.stamp = ros::Time::now();
    throttle_sp.thrust = std::min(1.0, std::max(0.0, THROTTLE_SCALE * (fz_cmd) / 9.8066));
    throttlePub.publish(throttle_sp);

    omega_sp.header.stamp = ros::Time::now();
    omega_sp.twist.angular.x = euler_dot(1);
    omega_sp.twist.angular.y = euler_dot(0);
    omega_sp.twist.angular.z = -euler_dot(2);
    omegaPub.publish(omega_sp);
    */
    // Reset update
    pose_up = 0; vel_up = 0;
    // Check for state update
    ros::spinOnce();

	  rate.sleep();
  }

  return 0;
}
