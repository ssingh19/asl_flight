#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Dense>

using namespace Eigen;

namespace utils
{
  void quat2rotM(const Eigen::Vector4d& q, Eigen::Matrix3d &R);
  void rotM2quat(Eigen::Vector4d& q, const Eigen::Matrix3d &R);
}


#endif /* UTILS_H */
