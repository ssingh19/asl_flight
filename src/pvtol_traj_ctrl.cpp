#include <iostream>
#include <std_msgs/Float64.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "mavros_msgs/AttitudeTarget.h"
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>

#include <utils/utils.h>
#include <trajectory/trajectory.h>
#include <trajectory/nlp_coeffs.h>

#include <control/ccmcontroller.h>
#include <GeoProb/Geodesic.h>
#include <GeoProb/Metric.h>

#define GRAV 9.8066

// Measured states
std::string current_mode;
Eigen::Vector4d mea_q;
Eigen::Matrix3d mea_R;
Eigen::Vector3d mea_eul;
Eigen::Vector3d mea_wb;
Eigen::Vector3d mea_er;
Eigen::Vector3d mea_pos;
Eigen::Vector3d mea_vel;
Eigen::Vector3d vel_prev;
double vel_prev_t;
static Eigen::Matrix<double,3,3> Rz_T =
(Eigen::Matrix<double,3,3>() << 0.0, 1.0, 0.0,
                               -1.0, 0.0, 0.0,
                                0.0, 0.0, 1.0).finished();


const Eigen::Matrix<double,2,6> K_LQR_ramp =
(Eigen::Matrix<double,2,6>() << -0.082402,1.423293,-0.510851,-0.077668,2.225517,-0.151686,
                                -1.402603,-0.078241,-5.638937,-2.096978,-0.140249,-2.689655).finished();

const Eigen::Matrix<double,2,6> K_LQR_flat =
(Eigen::Matrix<double,2,6>() << 0.004742,1.137935,-0.412228,0.216654,2.489380,0.307755,
                              -1.435149,-0.040185,-5.885191,-1.801157,-0.082870,-2.379801).finished();

const Eigen::Matrix<double,2,6> K_LQR_slow =
(Eigen::Matrix<double,2,6>() << 0.003505,1.141416,-0.435992,0.195134,2.470314,0.293488,
                              -1.435852,-0.033932,-5.857074,-1.792798,-0.100479,-2.403797).finished();

// Thrust estimation MA
double FZ_EST_N;
double fz_est_raw;
double fz_est;
double fz_est_sum;

// om estimation MA
Eigen::Vector3d om_est_sum;
double OM_EST_N;

// Update variables
int pose_up;
int vel_up;

// Controller gains
double KX;
double KV;
double KP;
double KY;

void compute_cntrl_planar(const double fz_cmd, const double x_des, const double yaw_des, Eigen::Vector3d &ref_er){

  // control to keep quad planar
  // compute after lqr control

  double x_ddot_des = -KX*(mea_pos(0)-x_des) - KV*mea_vel(0);

  // set equal to -fz * sin(p)
  double pitch_cmd = std::asin(-x_ddot_des/fz_cmd);

  utils::om2er(mea_er,mea_wb,mea_eul);

  // pitch command rate
  double pitch_rate_nom = KX*mea_vel(0)/(fz_cmd*std::cos(pitch_cmd));

  ref_er(1) = pitch_rate_nom + KP * (pitch_cmd - mea_eul(1));

  // yaw command rate
  ref_er(2) = KY * (yaw_des - mea_eul(2));

}

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

  // Conversion to Euler(123)
  utils::rotM2Euler(mea_eul, mea_R);

  pose_up = 1;

}

void velSubCB(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  // By default, MAVROS gives local_position/velocity in ENU

  vel_prev = mea_vel;

  double vel_dt = (msg->header.stamp.sec + msg->header.stamp.nsec*(1.0e-9))-
                   vel_prev_t;
  vel_prev_t = msg->header.stamp.sec + msg->header.stamp.nsec*(1.0e-9);

  mea_vel(0) = msg->twist.linear.y;
  mea_vel(1) = msg->twist.linear.x;
  mea_vel(2) = -msg->twist.linear.z;
  mea_wb(0) = msg->twist.angular.x;
  mea_wb(1) = -msg->twist.angular.y;
  mea_wb(2) = -msg->twist.angular.z;

  om_est_sum += mea_wb - (om_est_sum/OM_EST_N);
  mea_wb = om_est_sum/OM_EST_N;

  //update accel estimate
  Eigen::Vector3d acc = (mea_vel - vel_prev)/vel_dt;
  acc(2) += -9.8066;
  fz_est_raw = -acc.dot(mea_R.col(2));

  // Moving average update
  fz_est_sum = fz_est_sum + fz_est_raw - (fz_est_sum/FZ_EST_N);
  fz_est = fz_est_sum/FZ_EST_N;

  vel_up = 1;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pvtol_traj_ctrl");
  ros::NodeHandle nh;

  // Subscriptions
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
  ros::Subscriber poseSub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 2, poseSubCB);
  ros::Subscriber velSub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 2, velSubCB);

  pose_up = 0; vel_up = 0;

  // Actuator publisher
  ros::Publisher cmdPub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude",2);
  mavros_msgs::AttitudeTarget att_sp;


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
  ros::Publisher debug_pub18 = nh.advertise<std_msgs::Float64>("/debug18", 1);
  ros::Publisher debug_pub19 = nh.advertise<std_msgs::Float64>("/debug19", 1);
  std_msgs::Float64 debug_msg;

  // Controller frequency
  double CONTROL_DT = 1.0/250.0;
  ros::Rate rate(1.0/CONTROL_DT);

  double FZ_CTRL_N = 1.0;
  ros::param::get("~FZ_CTRL_N", FZ_CTRL_N);
  CCMController ctrl(FZ_CTRL_N);

  // Controller gains
  ros::param::get("~KX", KX);
  ros::param::get("~KV", KV);
  ros::param::get("~KP", KP);
  ros::param::get("~KY", KY);

  double K_LQR_SCALE = 1.0;
  ros::param::get("~K_LQR_SCALE", K_LQR_SCALE);

  // Initialize LQR gain
  // Eigen::VectorXd K_lqr_row_1(6);
  // Eigen::VectorXd K_lqr_row_2(6);
  // K_lqr_row_1.setZero();
  // K_lqr_row_2.setZero();

  // LQR* lqr_t = new LQR(NLP_T);
  Eigen::MatrixXd K_LQR(2,6);


  // Define trajectory class
  std::string traj_type;
  ros::param::get("~TRAJ", traj_type);

  double start_delay;
  ros::param::get("~START_DELAY", start_delay);

  Trajectory* traj;

  // NOTE: NO AUTO TAKEOFF

  if (traj_type == "CIRCLE") {

    double circle_T;
    double circle_R;
    ros::param::get("~CIRCLE_T", circle_T);
    ros::param::get("~RADIUS_C", circle_R);
    traj = new CircleTrajectory(circle_R, 2.0*M_PI*(1.0/circle_T),start_delay);

  } else if (traj_type == "POLY") {

    double poly_scale;
    ros::param::get("~POLY_SCALE", poly_scale);
    traj = new PolyTrajectory(poly_scale, start_delay);

  } else if (traj_type == "FIG8") {

    double circle_T;
    double radius_x, radius_y = 1.0;
    ros::param::get("~CIRCLE_T", circle_T);
    ros::param::get("~RADIUS_X", radius_x);
    ros::param::get("~RADIUS_Y", radius_y);
    traj = new Fig8Trajectory(radius_x, radius_y, 2.0*M_PI*(1.0/circle_T), start_delay);

  } else if (traj_type == "NLP") {

    traj = new NLPTrajectory(6,2,CONTROL_DT);

  } else  {

    traj = new HoverTrajectory();

  }

  // Define takeoff params
  double Z_INIT_DES;
  ros::param::get("~Z_INIT", Z_INIT_DES);
  double TAKEOFF_HGT;

  double TAKEOFF_TIME;
  ros::param::get("~TAKEOFF_TIME",TAKEOFF_TIME);

  double START_DELAY;
  ros::param::get("~START_DELAY",START_DELAY);

  double YAW_INIT;

  Eigen::Vector3d takeoff_loc;

  // Tracking status variables
  bool traj_started = false;
  double time_prev = ros::Time::now().toSec();
  double time_traj = 0.0;
  double dt = 0.0;

  // Reference for CCM controller
  Eigen::Vector3d r_pos(0, 0, 0);
  Eigen::Vector3d r_vel(0, 0, 0);
  Eigen::Vector3d r_acc(0, 0, 0);
  Eigen::Vector3d r_jer(0, 0, 0);

  // Reference for PVTOL controller
  double x_des = 0.0; //x_dot_des = 0
  double yaw_des = 0.0; //yaw_dot_des = 0

  Eigen::VectorXd _state_nom(6); //y,z,phi,y_dot,z_dot,phi_dot
  _state_nom.setZero();
  Eigen::Vector2d _ctrl_nom;
  _ctrl_nom.setZero();
  int ind_seg = 1;

  Eigen::VectorXd _state(6);

  // Estimator values
  ros::param::get("~FZ_EST_N", FZ_EST_N);
  ros::param::get("~OM_EST_N", OM_EST_N);


  fz_est = 9.8066;
  fz_est_sum = fz_est * FZ_EST_N;
  Eigen::Vector3d euler;
  vel_prev_t = 0.0;
  mea_q.setZero();
  mea_R = Eigen::Matrix3d::Identity();
  mea_eul.setZero();
  mea_wb.setZero();
  om_est_sum.setZero();
  mea_er.setZero();
  mea_pos.setZero();
  mea_vel.setZero();
  Eigen::Vector3d vb(0.0,0.0,0.0);
  vel_prev.setZero();

  // Command values
  Eigen::Vector3d ref_er(0,0,0);
  double fz_cmd = 9.8066;

  while(ros::ok()) {
    // Check for state update
    ros::spinOnce();

    // Publish tracking results
    debug_msg.data = x_des;
    debug_pub1.publish(debug_msg);
    debug_msg.data = mea_pos(0);
    debug_pub2.publish(debug_msg);

    debug_msg.data = _state_nom(0);
    debug_pub3.publish(debug_msg);
    debug_msg.data = mea_pos(1);
    debug_pub4.publish(debug_msg);

    debug_msg.data = _state_nom(1);
    debug_pub5.publish(debug_msg);
    debug_msg.data = mea_pos(2);
    debug_pub6.publish(debug_msg);

    debug_msg.data = _state_nom(2);
    debug_pub7.publish(debug_msg);

    debug_msg.data = mea_eul(0);
    debug_pub8.publish(debug_msg);
    debug_msg.data = mea_eul(1);
    debug_pub9.publish(debug_msg);
    debug_msg.data = mea_eul(2);
    debug_pub10.publish(debug_msg);

    debug_msg.data = fz_cmd;
    debug_pub18.publish(debug_msg);
    debug_msg.data = fz_est;
    debug_pub19.publish(debug_msg);

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
        ROS_INFO("takeoff loc: (%.3f,%.3f,%.3f)",mea_pos(0),mea_pos(1),mea_pos(2));

        // initialize ref pos
        x_des = takeoff_loc(0);
        _state_nom(0) = mea_pos(1);
        _state_nom(1) = mea_pos(2);
        TAKEOFF_HGT = mea_pos(2)-Z_INIT_DES;

        r_pos = takeoff_loc;

        YAW_INIT = mea_eul(2);
        yaw_des = YAW_INIT;

        // initialize takeoff velocity
        _state_nom(4) = -(TAKEOFF_HGT/TAKEOFF_TIME);
        r_vel << 0.0, 0.0, -(TAKEOFF_HGT/TAKEOFF_TIME);

        // if doing takeoff, start point is hover point above takeoff
        traj->set_start_pos(takeoff_loc + Eigen::Vector3d(0.0,0.0,-TAKEOFF_HGT));

      } else {
          time_traj += dt;
      }
    } else {

      //reset
      traj_started = false;
      ctrl.setMode(traj_started);

      time_traj = 0.0;

      x_des = mea_pos(0);
      _state_nom << mea_pos(1), mea_pos(2), 0.0, 0.0, 0.0, 0.0;
      _ctrl_nom << GRAV, 0.0;

      r_pos = mea_pos;
      r_vel.setZero();

    }

    if (time_traj > 0 && time_traj <= TAKEOFF_TIME) {
      // in takeoff

      r_pos(2) = takeoff_loc(2)-(time_traj/TAKEOFF_TIME)*TAKEOFF_HGT;

      _state_nom(1) = takeoff_loc(2)-(time_traj/TAKEOFF_TIME)*TAKEOFF_HGT;
      _ctrl_nom << GRAV, 0.0;

      if (time_traj > 2.0){
        yaw_des = YAW_INIT - YAW_INIT*(time_traj/TAKEOFF_TIME);
      } else {
        yaw_des = YAW_INIT;
      }

    } else if (time_traj > TAKEOFF_TIME && time_traj <= TAKEOFF_TIME+START_DELAY) {
      // adjust yaw to zero

      r_pos(2) = Z_INIT_DES;
      r_vel.setZero();

      _state_nom(1) = Z_INIT_DES;
      _state_nom(4) = 0.0;

      yaw_des = 0.0;

    } else if (time_traj > TAKEOFF_TIME+START_DELAY){
      // in main trajectory
      yaw_des = 0.0;
      traj->eval_explicit(time_traj+dt-(TAKEOFF_TIME+START_DELAY), _state_nom, _ctrl_nom, ind_seg);

    } else {

      // use generic setpoint
      x_des = mea_pos(0);
      _state_nom << mea_pos(1), mea_pos(2), 0.0, 0.0, 0.0, 0.0;
      _ctrl_nom << GRAV , 0.0;

      r_pos = mea_pos;
      r_vel.setZero();

    }

    if (time_traj > TAKEOFF_TIME+START_DELAY) {

      // Use Learning controller

      vb = mea_R.transpose() * mea_vel;
      _state << mea_pos(1), mea_pos(2), mea_eul(0), vb(1), vb(2), mea_wb(0);

      // Get time-varying LQR gains
      // lqr_t->eval(time_traj+dt-(TAKEOFF_TIME+START_DELAY),
                  // ind_seg, K_lqr_row_1, K_lqr_row_2);

      // fz_cmd = _ctrl_nom(0) + K_LQR_SCALE * K_lqr_row_1.dot(_state - _state_nom);
      // fz_cmd = std::max(fz_cmd, 2.0);
      // ref_er(0) = _ctrl_nom(1) + K_LQR_SCALE * K_lqr_row_2.dot(_state - _state_nom);


      if (ind_seg ==1) {
        K_LQR = K_LQR_ramp;
      } else if (ind_seg==2){
        K_LQR = K_LQR_flat;
      } else {
        K_LQR = K_LQR_slow;
      }

      K_LQR = K_LQR_SCALE*K_LQR;

      fz_cmd = _ctrl_nom(0) + K_LQR.row(0).dot(_state - _state_nom);
      fz_cmd = std::max(fz_cmd, 2.0);
      ref_er(0) = _ctrl_nom(1) + K_LQR.row(1).dot(_state - _state_nom);

      // Compute planar control
      compute_cntrl_planar(fz_cmd, x_des, yaw_des, ref_er);

    } else {

      // Use CCM controller
      ctrl.updateState(mea_pos, mea_R, mea_vel, mea_wb, fz_est, dt, pose_up, vel_up);
      ctrl.calcCCM(yaw_des, 0.0, r_pos, r_vel, r_acc, r_jer);
      fz_cmd = ctrl.getfz();
      ref_er = ctrl.getEr();
    }

    // Publish commands
    att_sp.header.stamp = ros::Time::now();
    att_sp.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    att_sp.body_rate.x = ref_er(0);
    att_sp.body_rate.y = ref_er(1);
    att_sp.body_rate.z = ref_er(2);
    att_sp.thrust = std::min(1.0, std::max(0.0, 0.56 * (fz_cmd) / 9.8066));
    cmdPub.publish(att_sp);

    // Reset update
    pose_up = 0; vel_up = 0;
    // Check for state update
    ros::spinOnce();

	  rate.sleep();
  }

  return 0;
}
