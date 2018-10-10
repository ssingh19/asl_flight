#ifndef GEODESIC_H
#define GEODESIC_H

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "Metric.h"

static const double PI = 3.14159265358979323846;

namespace geodesic {
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

void
compute_W(const VectorXd &x, MatrixXd &W);

void
compute_dW(const VectorXd &x, MatrixXd &dW, const int j);

double
mono_eval(const Vector3d &x, const Vector3d &p, const bool d, const int j);

/*
void
compute_cheby(MatrixXd &C, const int N, const int K, const VectorXd &t);

void
compute_cheby_d(MatrixXd &C, const int N, const int K, const VectorXd &t);

void
clencurt(int K, VectorXd &t, VectorXd &w);
*/

}


#endif
