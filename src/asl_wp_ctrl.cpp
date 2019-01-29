// This node does state estimation (use for calibration) and sends waypoint commands

#include <iostream>
#include <std_msgs/Float64.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <utils/utils.h>

#define FZ_EST_N 10.0

// Measured states
std::string current_mode;
Eigen::Vector4d mea_q;
Eigen::Matrix3d mea_R;
Eigen::Vector3d mea_wb;
Eigen::Vector3d mea_pos;
Eigen::Vector3d mea_vel;
Eigen::Vector3d vel_prev;
double vel_prev_t;
static Eigen::Matrix<double,3,3> Rz_T =
(Eigen::Matrix<double,3,3>() << 0.0, 1.0, 0.0,
                               -1.0, 0.0, 0.0,
                                0.0, 0.0, 1.0).finished();

// Thrust estimation MA
double fz_est_raw;
double fz_est;
double fz_est_sum;

// Update variables
int pose_up;
int vel_up;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_mode = msg->mode;
}

void poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // By default, MAVROS gives local_position in ENU
  mea_pos(0) = msg->pose.position.y;
  mea_pos(1) = msg->pose.position.x;
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

  // Final conversion to rot matrix
  utils::quat2rotM(mea_q, mea_R);

  pose_up = 1;

}

void velSubCB(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  // By default, MAVROS gives local_position/velocity in NWU

  vel_prev = mea_vel;

  double vel_dt = (msg->header.stamp.sec + msg->header.stamp.nsec*(1.0e-9))-
                   vel_prev_t;
  vel_prev_t = msg->header.stamp.sec + msg->header.stamp.nsec*(1.0e-9);

  mea_vel(0) = msg->twist.linear.y;
  mea_vel(1) = msg->twist.linear.x;
  mea_vel(2) = -msg->twist.linear.z;
  mea_wb(0) = msg->twist.angular.y;
  mea_wb(1) = msg->twist.angular.x;
  mea_wb(2) = -msg->twist.angular.z;

  //update accel estimate
  Eigen::Vector3d acc = (mea_vel - vel_prev)/vel_dt;
  acc(2) += -9.8066;
  double fz_est_up = -acc.dot(mea_R.col(2));

  fz_est_raw = fz_est_up;

  // Moving average update
  fz_est_sum = fz_est_sum + fz_est_up - (fz_est_sum/FZ_EST_N);
  fz_est = fz_est_sum/FZ_EST_N;

  vel_up = 1;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "asl_traj_ctrl");
  ros::NodeHandle nh;

  // Subscriptions
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 2, state_cb);
  ros::Subscriber poseSub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 2, poseSubCB);
  ros::Subscriber velSub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 2, velSubCB);

  pose_up = 0; vel_up = 0;

  // Waypoint publisher
  ros::Publisher wpPub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
  geometry_msgs::PoseStamped pos_sp;

  // Waypoint command type
  // if true then does position hold at offboard initialization location
  // else will go to desired setpoint coded in launch file
  bool cmd_hold;
  ros::param::get("~CMD_HOLD", cmd_hold);

  // Assume hard coded setpoint in launch file is NED
  double ref_x = 0.0;
  double ref_y = 0.0;
  double ref_z = 0.0;
  ros::param::get("~REF_X", ref_x);
  ros::param::get("~REF_Y", ref_y);
  ros::param::get("~REF_Z", ref_z);

  // DEBUG PUBLICATIONS
  // ros::Publisher debug_pub1 = nh.advertise<std_msgs::Float64>("/debug1", 1);
  // ros::Publisher debug_pub2 = nh.advertise<std_msgs::Float64>("/debug2", 1);
  // ros::Publisher debug_pub3 = nh.advertise<std_msgs::Float64>("/debug3", 1);
  // ros::Publisher debug_pub4 = nh.advertise<std_msgs::Float64>("/debug4", 1);
  // ros::Publisher debug_pub5 = nh.advertise<std_msgs::Float64>("/debug5", 1);
  // ros::Publisher debug_pub6 = nh.advertise<std_msgs::Float64>("/debug6", 1);
  // ros::Publisher debug_pub7 = nh.advertise<std_msgs::Float64>("/debug7", 1);
  // ros::Publisher debug_pub8 = nh.advertise<std_msgs::Float64>("/debug8", 1);
  // ros::Publisher debug_pub9 = nh.advertise<std_msgs::Float64>("/debug9", 1);
  // ros::Publisher debug_pub10 = nh.advertise<std_msgs::Float64>("/debug10", 1);
  // ros::Publisher debug_pub11 = nh.advertise<std_msgs::Float64>("/debug11", 1);
  // ros::Publisher debug_pub12 = nh.advertise<std_msgs::Float64>("/debug12", 1);
  // ros::Publisher debug_pub13 = nh.advertise<std_msgs::Float64>("/debug13", 1);
  // std_msgs::Float64 debug_msg;

  // Controller frequency
  ros::Rate rate(250.0);

  // Tracking status variables
  bool traj_started = false;

  // Reference yaw in ENU
  double yaw_des = M_PI/4.0;

  // Estimator values
  fz_est = 9.8066;
  fz_est_sum = fz_est * FZ_EST_N;
  vel_prev_t = 0.0;
  mea_q.setZero();
  mea_R = Eigen::Matrix3d::Identity();
  mea_wb.setZero();
  mea_pos.setZero();
  mea_vel.setZero();
  vel_prev.setZero();

  // Command values
  Eigen::Vector3d r_pos(0, 0, 0);

  while(ros::ok()) {
    // Check for state update
    ros::spinOnce();

    if (current_mode == "OFFBOARD") {

      // update trajectory status variables
      if (!traj_started){

          traj_started = true;

          ROS_INFO("offboard started");

          // initialize ref pos to be current location
          r_pos << mea_pos(0), mea_pos(1), mea_pos(2);

      } else {
          // offboard already initialized
      }
    } else {
      //reset
      traj_started = false;
    }

    // set reference to command if not position hold at initialization
    if (!cmd_hold){
      r_pos << ref_x, ref_y, ref_z;
    }

    // Publish waypoint in ENU frame
    pos_sp.header.stamp = ros::Time::now();
    pos_sp.pose.position.x = r_pos(1);
    pos_sp.pose.position.y = r_pos(0);
    pos_sp.pose.position.z = -r_pos(2);
    pos_sp.pose.orientation.x = 0;
    pos_sp.pose.orientation.y = 0;
    pos_sp.pose.orientation.z = sin(yaw_des/2.0);
    pos_sp.pose.orientation.w = cos(yaw_des/2.0);
    wpPub.publish(pos_sp);

    // Reset update
    pose_up = 0; vel_up = 0;
    // Check for state update
    ros::spinOnce();

	  rate.sleep();
  }

  return 0;
}
