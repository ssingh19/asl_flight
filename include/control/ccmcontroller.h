#ifndef CCMCONTROLLER_H
#define CCMCONTROLLER_H

#include "ros/ros.h"
#include "mavros_msgs/ActuatorControl.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <GeoProb/GeoProb.h>
#include <GeoProb/Geodesic.h>
#include <GeoProb/Metric.h>


class CCMController {
private:
  using Jacobian = ifopt::Component::Jacobian;

  ros::NodeHandle nh;

  // Mode
  bool active;

  // measured states
  Eigen::Matrix3d mea_R;
  Eigen::Vector3d euler;
  Eigen::Vector3d mea_wb;
  Eigen::Vector3d mea_pos;
  Eigen::Vector3d mea_vel;
  double fz;


  // CCM specific variables
  Eigen::VectorXd _xc_nom;
  Eigen::Vector4d _uc_nom;
  Eigen::VectorXd _xc;
  Eigen::VectorXd _xc_nom_dot;
  Eigen::VectorXd _xc_dot;
  Eigen::MatrixXd _W_nom;
  Eigen::MatrixXd _W;
  Eigen::MatrixXd _M_nom;
  Eigen::MatrixXd _M;
  // Eigen::MatrixXd X_dot_Geod;
  Eigen::MatrixXd X_dot_EndP;
  // std::vector<Jacobian> Phi_dot;
  double _M_yaw;
  Eigen::Matrix3d _R_des;

  // Geodesic Problem
  // ifopt::Problem* geo_Prob;
  // ifopt::IpoptSolver* ipopt;
  // std::shared_ptr<ifopt::GeoVariables> geovar_ptr;
  // std::shared_ptr<ifopt::GeoConstraint> geocon_ptr;
  // Eigen::VectorXd c_opt;

  // use for integration
  double dt;

  // config matrix
  Eigen::Matrix4d A;

  // inertia
  Eigen::Matrix3d J;

  // CCM outputs
  double E;
  Eigen::Vector4d uc_fb;
  double fz_dot; //CCM computed thrust_dot
  Eigen::Vector3d euler_dot;
  Eigen::Vector3d r_w_nom;
  Eigen::Vector3d r_wb;
  double fzCmd; // Thrust command
  Eigen::Vector3d tauCmd; // Torque command

  // constants
  double KR,KW;
  double mass;
  double g;
  double TCOEFF;
  double lambda;
  Eigen::MatrixXd _B_ctrl;
  std::string MODEL;

  // CCM controller funcs
  void calc_CCM_dyn(const Eigen::VectorXd &xc,const Eigen::Vector4d &uc,
                    Eigen::VectorXd &dyn);
  void calc_xc_uc_nom(const Eigen::Vector3d &r_pos,
                      const Eigen::Vector3d &r_vel,
                      const Eigen::Vector3d &r_acc,
                      const Eigen::Vector3d &r_jer,
                      const double yaw_des); // compute xc, uc nom

  // Coordinate conversions
  void R2euler_123(void);
  void Euler2R_123(const double r, const double p, const double y);
  void R_om(const Eigen::Vector3d &q, const Eigen::Vector3d &q_dot);

public:
  double motorCmd[4];
  CCMController(void);

  // copy in updated state into controller class
  void updateState(const Eigen::Vector3d &r, const Eigen::Matrix3d &R,
                   const Eigen::Vector3d &v, const Eigen::Vector3d &w,
                   const double _fz, const double _dt, const int pose_up, const int vel_up);

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
  Eigen::Vector3d getEuler();

  void setMode(const bool _m);
};




#endif
