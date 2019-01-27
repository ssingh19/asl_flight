#include <utils/utils.h>
#include <ros/ros.h>
#include <math.h>
#include <Eigen/Dense>

void utils::quat2rotM(const Eigen::Vector4d& q, Eigen::Matrix3d &R){


  double q1 = q(0);
  double q2 = q(1);
  double q3 = q(2);
  double q4 = q(3);

  double q1s = q1*q1;
  double q2s = q2*q2;
  double q3s = q3*q3;
  double q4s = q4*q4;

  R <<
    q1s+q2s-q3s-q4s, 2.0*(q2*q3-q1*q4) ,2.0*(q2*q4+q1*q3),
    2.0*(q2*q3+q1*q4), q1s-q2s+q3s-q4s, 2.0*(q3*q4-q1*q2),
    2.0*(q2*q4-q1*q3), 2.0*(q3*q4+q1*q2), q1s-q2s-q3s+q4s;
}

void utils::rotM2quat(Eigen::Vector4d& q, const Eigen::Matrix3d &R){

  double T = R.trace();
  Eigen::Vector4d case_vec(T, R(0,0), R(1,1), R(2,2));

  std::ptrdiff_t i;
  case_vec.maxCoeff(&i);

  double den = 0.0;

  switch (i)   {
    case 0:
      den = sqrt(1.0+T);
      q(0) = 0.5*den;
      q(1) = 0.5*(R(2,1)-R(1,2))/den;
      q(2) = 0.5*(R(0,2)-R(2,0))/den;
      q(3) = 0.5*(R(1,0)-R(0,1))/den;
      break;

    case 1:
      den = sqrt(1+R(0,0)-R(1,1)-R(2,2));
      q(0) = 0.5*(R(2,1)-R(1,2))/den;
      q(1) = 0.5*den;
      q(2) = 0.5*(R(0,1)+R(1,0))/den;
      q(3) = 0.5*(R(2,0)+R(0,2))/den;
      break;

    case 2:
      den = sqrt(1-R(0,0)+R(1,1)-R(2,2));
      q(0) = 0.5*(R(0,2)-R(2,0))/den;
      q(1) = 0.5*(R(0,1)+R(1,0))/den;
      q(2) = 0.5*den;
      q(3) = (R(1,2)+R(2,1))/den;
      break;
      
    default:
      den = sqrt(1-R(0,0)-R(1,1)+R(2,2));
      q(0) = 0.5*(R(1,0)-R(0,1))/den;
      q(1) = 0.5*(R(2,0)+R(0,2))/den;
      q(2) = 0.5*(R(2,1)+R(1,2))/den;
      q(3) = 0.5*den;
  }



}
