#include <iostream>
#include <control/ccmcontroller.h>
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
#include <GeoProb/Geodesic.h>
#include <GeoProb/Metric.h>

#define TAKEOFF_TIME 5.0
#define TAKEOFF_HGT 1.0
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


  // ROS_INFO("pos: (%.4f, %.4f, %.4f), quat: (%.4f, %.4f: %.4f, %.4f)", mea_pos(0), mea_pos(1), mea_pos(2),
  //                                                                     q_w, q_x,q_y, q_z);

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

  // ROS_INFO("vel: (%.4f, %.4f, %.4f), omb: (%.4f, %.4f: %.4f)", mea_vel(0), mea_vel(1), mea_vel(2),
  //                                                               mea_wb(0), mea_wb(1), mea_wb(2));

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

  // Actuator publisher
  bool cmd_mot;
  ros::param::get("~CMD_MOT", cmd_mot);

  ros::Publisher actuatorPub;
  ros::Publisher omegaPub;
  ros::Publisher throttlePub;

  if (cmd_mot) {

    actuatorPub = nh.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 1);

  } else {

    omegaPub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_attitude/cmd_vel", 2);
    throttlePub = nh.advertise<mavros_msgs::Thrust>("mavros/setpoint_attitude/thrust", 2);

  }

  mavros_msgs::ActuatorControl mot_sp;
  geometry_msgs::TwistStamped omega_sp;
  mavros_msgs::Thrust throttle_sp;


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
  ros::Publisher debug_pub13 = nh.advertise<std_msgs::Float64>("/debug13", 1);
  std_msgs::Float64 debug_msg;

  // Define controller classes
  CCMController ctrl;


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
  double yaw_des = M_PI/2.0;
  Eigen::Vector3d r_pos(0, 0, 0);
  Eigen::Vector3d r_vel(0, 0, 0);
  Eigen::Vector3d r_acc(0, 0, 0);
  Eigen::Vector3d r_jer(0, 0, 0);
  Eigen::Vector3d takeoff_loc(0, 0, 0);

  // Estimator values
  fz_est = 9.8066;
  fz_est_sum = fz_est * FZ_EST_N;
  Eigen::Vector3d euler;
  vel_prev_t = 0.0;
  mea_q.setZero();
  mea_R = Eigen::Matrix3d::Identity();
  mea_wb.setZero();
  mea_pos.setZero();
  mea_vel.setZero();
  vel_prev.setZero();

  // Command values
  Eigen::Vector3d ref_er;
  Eigen::Vector3d ref_om;
  double fz_cmd = 9.8066;


  double THROTTLE_SCALE;
  ros::param::get("~TSCALE", THROTTLE_SCALE);


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
    debug_pub11.publish(debug_msg);
    debug_msg.data = fz_est;
    debug_pub12.publish(debug_msg);
    debug_msg.data = fz_est_raw;
    debug_pub13.publish(debug_msg);

    // Time loop calculations
    dt = ros::Time::now().toSec() - time_prev;
    time_prev = ros::Time::now().toSec();

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
    }
    // Update controller internal state
    ctrl.updateState(mea_pos, mea_R, mea_vel, mea_wb, fz_est, dt, pose_up, vel_up);

    // Compute feedback
    ctrl.calcCCM(yaw_des, r_pos, r_vel, r_acc, r_jer);

    // publish commands
    fz_cmd = ctrl.getfz();
    ref_er = ctrl.getEr();
    ref_om = ctrl.getOm();

    euler = ctrl.getEuler();

    // ROS_INFO("(%.4f, %.4f, %.4f: %.4f, %.4f, %.4f)", euler(0), euler(1), euler(2), ref_om(0), ref_om(1), ref_om(2));

    if (cmd_mot) {

      mot_sp.header.stamp = ros::Time::now();
      mot_sp.group_mix = 0;
      for(int i=0; i<4; i++) {
        mot_sp.controls[i] = ctrl.motorCmd[i];
      }
      mot_sp.controls[7] = 0.1234; // secret key to enabling direct motor control in px4
      actuatorPub.publish(mot_sp);

    } else {

      throttle_sp.header.stamp = ros::Time::now();
      throttle_sp.thrust = std::min(1.0, std::max(0.0, THROTTLE_SCALE * (fz_cmd) / 9.8066));
      throttlePub.publish(throttle_sp);

      // Publish omega_sp in ENU
      omega_sp.header.stamp = ros::Time::now();
      omega_sp.twist.angular.x = ref_er(1);
      omega_sp.twist.angular.y = ref_er(0);
      omega_sp.twist.angular.z = -ref_er(2);
      omegaPub.publish(omega_sp);

    }

    // Publish
    debug_msg.data = ctrl.getE();
    debug_pub7.publish(debug_msg);

    debug_msg.data = ref_om(0);
    debug_pub8.publish(debug_msg);
    debug_msg.data = ref_om(1);
    debug_pub9.publish(debug_msg);
    debug_msg.data = ref_om(2);
    debug_pub10.publish(debug_msg);

    // Reset update
    pose_up = 0; vel_up = 0;
    // Check for state update
    ros::spinOnce();

	  rate.sleep();
  }

  return 0;
}
