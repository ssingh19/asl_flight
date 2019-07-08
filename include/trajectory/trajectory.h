/* trajectory classes */

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Eigen/Dense>
#include <vector>
#include <fstream>

class Trajectory
{
public:
  Trajectory() {};
  virtual void eval(double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc, Eigen::Vector3d &jer,
                    double &yaw, double &yaw_dot) {};

//  virtual void set_output(int index, double value) {};
  virtual void set_start_pos(const Eigen::Vector3d &_start_pos) {};

  virtual void eval_explicit(double t, Eigen::VectorXd &_state_nom, Eigen::Vector2d &_cntrl_nom, int &ind_seg) {};
};

class HoverTrajectory: public Trajectory {
private:
  Eigen::Vector3d start_pos;

public:
  HoverTrajectory();
  void eval(double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc, Eigen::Vector3d &jer,
            double &yaw, double &yaw_dot);
  void set_start_pos(const Eigen::Vector3d &_start_pos);

};

class CircleTrajectory: public Trajectory {
private:
  Eigen::Vector3d start_pos;
  double om;
  double radius;
  double start_delay;

public:
  CircleTrajectory(double _radius, double _om, double _start_delay);
  void eval(double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc, Eigen::Vector3d &jer,
            double &yaw, double &yaw_dot);

  void set_start_pos(const Eigen::Vector3d &_start_pos);

};

class Fig8Trajectory: public Trajectory {
private:
  Eigen::Vector3d start_pos;
  double om;
  double radius_x;
  double radius_y;
  double start_delay;

public:
  Fig8Trajectory(double _radius_x, double _radius_y, double _om, double _start_delay);
  void eval(double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc, Eigen::Vector3d &jer,
            double &yaw, double &yaw_dot);

  void set_start_pos(const Eigen::Vector3d &_start_pos);

};


class PolyTrajectory: public Trajectory {
  // Requires pre-stored matrix of poly coeffs for x,y,z,yaw & time breakpoints (direct load)
private:
  Eigen::Vector3d start_pos;
  Eigen::VectorXd switch_time;
  std::vector<Eigen::MatrixXd> cp;
  std::vector<Eigen::MatrixXd> cp_yaw;

  Eigen::VectorXd output;
  Eigen::Vector2d output_yaw;

  double start_delay;
  double scale;

  Eigen::MatrixXd expand_coeffs(Eigen::MatrixXd& _coeffs);
  Eigen::MatrixXd expand_coeffs_yaw(Eigen::MatrixXd& _coeffs);

public:
  PolyTrajectory(double _scale, double _start_delay);
  void eval(double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc, Eigen::Vector3d &jer,
            double &yaw, double &yaw_dot);

  void set_start_pos(const Eigen::Vector3d &_start_pos);

};

class SwoopTrajectory: public Trajectory {

private:
  Eigen::Vector3d start_pos;

  double start_delay;
  double y_bar;
  double delta_1;
  double delta_2;
  double z_bar;

public:
  SwoopTrajectory(double _ybar, double _delta1, double _delta2, double _zbar, double _start_delay);
  void eval(double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc, Eigen::Vector3d &jer,
            double &yaw, double &yaw_dot);

  void set_start_pos(const Eigen::Vector3d &_start_pos);

};

class NLPTrajectory: public Trajectory {
  // Requires pre-stored matrix of Lagrange coefficients & total time (direct load)
private:
  Eigen::Vector3d start_pos;

  int n;
  int m;
  int N;
  double dt;
  double t_end;

  Eigen::VectorXd tau_grid; // normalized collocation nodes [-1,1]

  // Lagrange Interpolation matrix
  Eigen::MatrixXd Lagrange_interp;

  // Lagrange computation
  void compute_lagrange(const double t);

public:
  NLPTrajectory(int _n, int _m, double _dt);

  void set_start_pos(const Eigen::Vector3d &_start_pos);

  void eval_explicit(double t, Eigen::VectorXd &_state_nom, Eigen::Vector2d &_cntrl_nom, int &ind_seg);

};

#endif /* TRAJECTORY_H */
