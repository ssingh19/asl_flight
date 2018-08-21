#ifndef SE3CONTROLLER_H
#define SE3CONTROLLER_H

#include "ros/ros.h"
#include "mavros_msgs/ActuatorControl.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <Eigen/Dense>


class SE3Controller {
private:
  ros::NodeHandle nh;

  // measured quantities
  Eigen::Matrix3d mea_R;
  Eigen::Vector3d mea_wb;
  Eigen::Vector3d mea_pos;
  Eigen::Vector3d mea_vel;

  // use for computing accel
  Eigen::Vector3d vel_prev;
  double lpf;
  double dt;


  ros::Subscriber joySub;

  // config matrix
  Eigen::Matrix4d W;

  // inertia matrix
  Eigen::Matrix3d J;

  double fzCmd; // result of SE3 controller computation
  Eigen::Vector3d tauCmd; // result of SE3 controller computation

  double fzCmd_prev;

  double joyCmd[4]; // desired thrust + euler angles from joystick

  // gains
  double KP;
  double KV;
  double KR;
  double KW;

  // constants
  double M;
  double g;
  double TCOEFF;

  //model
  std::string MODEL;

  void joyCB(const sensor_msgs::JoyConstPtr &joy);

public:
  double motorCmd[4];
  SE3Controller(void);

  // copy in updated state into controller class
  void updatePose(const Eigen::Vector3d &r, const Eigen::Matrix3d &R);
  void updateVel(const Eigen::Vector3d &v, const Eigen::Vector3d &w, const double &_dt);
  void calcSE3(
          const double &yaw_des,
          const Eigen::Vector3d &r_pos,
          const Eigen::Vector3d &r_vel,
          const Eigen::Vector3d &r_acc,
          const Eigen::Vector3d &r_jer);
  void joySE3(void); // give reference input to SE3 controller from joystick. This enables manual joystick control
};




#endif
