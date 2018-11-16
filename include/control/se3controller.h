#ifndef SE3CONTROLLER_H
#define SE3CONTROLLER_H

#include "ros/ros.h"
#include "mavros_msgs/ActuatorControl.h"
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
  double fz;
  double dt;

  // config matrix
  Eigen::Matrix4d A;

  // inertia matrix
  Eigen::Matrix3d J;

  double FzCmd; // result of SE3 controller computation
  Eigen::Vector3d tauCmd; // result of SE3 controller computation

  // gains
  double KP;
  double KV;
  double KR;
  double KW;

  // constants
  double mass;
  double g;
  double TCOEFF;
  std::string MODEL;

  // Motor mixing
  void motor_mix(Eigen::Vector4d &wrench, Eigen::Vector4d &ffff);

public:
  double motorCmd[4];
  SE3Controller(void);

  // copy in updated state into controller class
  void updateState(const Eigen::Vector3d &r, const Eigen::Matrix3d &R,
                   const Eigen::Vector3d &v, const Eigen::Vector3d &w,
                   const double _fz, const double _dt, const int pose_up, const int vel_up);
  void calcSE3(
          const double &yaw_des,
          const Eigen::Vector3d &r_pos,
          const Eigen::Vector3d &r_vel,
          const Eigen::Vector3d &r_acc,
          const Eigen::Vector3d &r_jer);

  double getfz();
};




#endif
