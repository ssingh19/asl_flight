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
private:
  Eigen::Vector3d start_pos;
  Eigen::VectorXd switch_time;
  std::vector<Eigen::MatrixXd> cp;
  double start_delay;
  double scale;

  Eigen::MatrixXd expand_coeffs(Eigen::MatrixXd& _coeffs);

public:
  PolyTrajectory(double _scale, double _start_delay);
  void eval(double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc, Eigen::Vector3d &jer,
            double &yaw, double &yaw_dot);

  void set_start_pos(const Eigen::Vector3d &_start_pos);

};

#endif /* TRAJECTORY_H */
