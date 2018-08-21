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
  virtual void eval(double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc, Eigen::Vector3d &jer) {};

//  virtual void set_output(int index, double value) {};
  virtual void set_start_pos(const Eigen::Vector3d &_start_pos) {};
};

/*
class ConstTrajectory: public Trajectory
{
public:
  ConstTrajectory(int m, double c);
  VectorXd eval(double t);

  void set_output(int index, double value);

private:
  VectorXd output;
};

class PolyTrajectory: public Trajectory
{
public:
  PolyTrajectory(double _start_delay, double _scale);
  VectorXd eval(double t);

  void set_start_pos(VectorXd& _start_pos);

private:
  VectorXd start_pos;
  VectorXd time;
  std::vector<MatrixXd> cp;
  double start_delay;
  double scale;

  MatrixXd expand_coeffs(MatrixXd& _coeffs);

};
*/

class HoverTrajectory: public Trajectory {
private:
  Eigen::Vector3d start_pos;

public:
  HoverTrajectory();
  void eval(double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc, Eigen::Vector3d &jer);
  void set_start_pos(const Eigen::Vector3d &_start_pos);

};

class CircleTrajectory: public Trajectory {
private:
  Eigen::Vector3d start_pos;
  double om;
  double radius;

public:
  CircleTrajectory(double _radius, double _om);
  void eval(double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc, Eigen::Vector3d &jer);

  void set_start_pos(const Eigen::Vector3d &_start_pos);

};

#endif /* TRAJECTORY_H */
