#ifndef CCMCONTROLLER_H
#define CCMCONTROLLER_H

#include "ros/ros.h"
#include "mavros_msgs/ActuatorControl.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <Eigen/Dense>


class CCMController {
private:
  ros::NodeHandle nh;

  // Mode
  bool active;

  // measured states
  Eigen::Matrix3d mea_R;
  Eigen::Vector3d euler;
  Eigen::Vector3d mea_wb;
  Eigen::Vector3d mea_pos;
  Eigen::Vector3d mea_vel;


  // CCM specific variables
  Eigen::VectorXd _xc_nom;
  Eigen::Vector4d _uc_nom;
  Eigen::VectorXd _xic_nom;
  Eigen::VectorXd _xc;
  Eigen::VectorXd _xic;
  Eigen::VectorXd _xc_nom_dot;
  Eigen::VectorXd _xc_dot;
  Eigen::MatrixXd _M_nom;
  Eigen::MatrixXd _M;
  Eigen::MatrixXd _M_xi;
  double E;
  double fz;

  // use for integration
  double dt;

  // config matrix
  Eigen::Matrix4d W;

  // inertia
  Eigen::Matrix3d J;

  // CCM outputs
  Eigen::Vector4d uc_fb;
  double fz_dot; //CCM computed thrust_dot
  Eigen::Vector3d euler_dot;
  Eigen::Vector3d r_wb;
  double fzCmd; // result of CCM controller computation
  double fzCmd_prev;
  Eigen::Vector3d tauCmd; // result of CCM controller computation

  // gains
  double KW;

  // constants
  double M;
  double g;
  double TCOEFF;
  double lambda;
  Eigen::MatrixXd _B_ctrl;

  // model
  std::string MODEL;

  // CCM controller funcs
  void calc_CCM_aux
  void Phi(const Eigen::VectorXd &xc, Eigen::VectorXd &xic); // xc -> xic conversion fnc
  void M_fnc(const Eigen::VectorXd &xc, Eigen::MatrixXd &M); // computes M
  void dynamics(const Eigen::VectorXd &xc, const Eigen::Vector4d &uc, Eigen::VectorXd &dyn); // compute f(xc)
  void calc_xc_uc_nom(const Eigen::Vector3d &r_pos,
                      const Eigen::Vector3d &r_vel,
                      const Eigen::Vector3d &r_acc,
                      const Eigen::Vector3d &r_jer,
                      const double yaw_des); // compute xc, uc nom

  // Coordinate conversions
  void R2euler_123(void);
  void R2euler_321(void);
  void R_om(const Eigen::Vector3d &q, const Eigen::Vector3d &q_dot);
  Eigen::Matrix3d R_rot(const double a, const Eigen::Vector3d n);

public:
  double motorCmd[4];
  CCMController(void);

  // copy in updated state into controller class
  void updateState(const Eigen::Vector3d &r, const Eigen::Matrix3d &R,
                   const Eigen::Vector3d &v, const Eigen::Vector3d &w,
                   const double _dt, const int pos_type, const int vel_type);

  // compute
  void calcCCM(
          const double &yaw_des,
          const Eigen::Vector3d &r_pos,
          const Eigen::Vector3d &r_vel,
          const Eigen::Vector3d &r_acc,
          const Eigen::Vector3d &r_jer);

  // Return functions
  double getE();
  double getfz();
  Eigen::Vector3d getEulerdot();
  Eigen::Vector3d getvel();

  void setMode(const bool _m);
  void setfz(double _fz);
};




#endif
