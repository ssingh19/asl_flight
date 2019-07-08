#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Dense>

namespace utils
{
  void quat2rotM(const Eigen::Vector4d& q, Eigen::Matrix3d &R);
  void rotM2quat(Eigen::Vector4d& q, const Eigen::Matrix3d &R);
  void rotM2Euler(Eigen::Vector3d& eul, const Eigen::Matrix3d &R);

  void om2er(Eigen::Vector3d& er, const Eigen::Vector3d &om, const Eigen::Vector3d &q);
}


#endif /* UTILS_H */
