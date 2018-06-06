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
  ros::Subscriber poseSub;
  ros::Subscriber velSub;
  ros::Subscriber joySub;
  ros::Publisher actuatorPub;

  Eigen::Matrix3d mea_R;
  Eigen::Vector3d mea_wb;
  Eigen::Vector3d mea_pos;
  Eigen::Vector3d mea_vel;

  Eigen::Matrix4d W;

  double fzCmd; // result of SE3 controller computation
  Eigen::Vector3d tauCmd; // result of SE3 controller computation
  double motorCmd[4];
  double joyCmd[4]; // desired thrust + euler angles from joystick

  double KP;
  double KV;
  double KR;
  double KW;
  double M;
  double g;
  double TCOEFF;
  std::string MODEL;

  void poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void velSubCB(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void joyCB(const sensor_msgs::JoyConstPtr &joy);

public:
  SE3Controller(void);
  void calcSE3(
          const Eigen::Vector3d &r_euler,
          const Eigen::Vector3d &r_wb,
          const Eigen::Vector3d &r_pos,
          const Eigen::Vector3d &r_vel,
          const Eigen::Vector3d &r_acc);
  void joySE3(void); // give reference input to SE3 controller from joystick. This enables manual joystick control
};




#endif
