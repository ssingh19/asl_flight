#include <GeoProb/Geodesic.h>
#include <GeoProb/Metric.h>

namespace geodesic {

void
compute_W(const VectorXd &x, MatrixXd &W){
  // first monomial = 1
  W = w_coeff[0];
  for (int i = 1; i<N_mon; i++){
    W += w_coeff[i]*mono_eval(x.segment(6,3),w_mon.row(i),false,0);
  }
}

void
compute_dW(const VectorXd &x, MatrixXd &dW, const int j){
  // first monomial deriv = 0
  dW.setZero();
  for (int i = 1; i<N_mon; i++){
    dW += w_coeff[i]*mono_eval(x.segment(6,3),w_mon.row(i),true,j-6);
  }
}

double
mono_eval(const Vector3d &x, const Vector3d &p, const bool d, const int j){
  // assumes given x(6:8) and associated power indices p
  // at least one of p(i) > 0
  double val = 1.0;
  if (!d){ // regular monomial computation
    for (int i=0; i<3; i++){ val *= std::pow(x(i),p(i)); }
  } else { // compute monomial derivative
    if (p(j)<1.0) { return 0.0; }
    else {
      for (int i=0; i<3; i++){
        (i!=j) ? val *= std::pow(x(i),p(i)) : val *= p(i)*std::pow(x(i),p(i)-1.0);
      }
    }
  }
  return val;
}

void
compute_cheby(MatrixXd &C, const int N, const int K, const VectorXd &t){
  C.setZero();
  C.row(0) = VectorXd::Constant(K+1,1.0);
  C.row(1) = t.transpose();
  for (int i=2; i<N+1; i++){
    C.row(i) = 2.0*t.transpose().cwiseProduct(C.row(i-1)) - C.row(i-2);
  }
}

void
compute_cheby_d(MatrixXd &C, const int N, const int K, const VectorXd &t){
  MatrixXd U(N+1,K+1);
  U.setZero();
  U.row(0) = VectorXd::Constant(K+1,1.0);
  U.row(1) = 2.0*t.transpose();

  C.setZero();
  C.row(1)  = VectorXd::Constant(K+1,1.0);

  for (int i=2; i<N+1; i++){
    U.row(i) = 2.0*t.transpose().cwiseProduct(U.row(i-1)) - U.row(i-2);
    C.row(i) = i*U.row(i-1);
  }
}

void
clencurt(int K, VectorXd &t, VectorXd &w){
  Eigen::ArrayXd theta(K+1);
  for (int k=0; k<K+1; k++){ theta(k) = PI*k/K; }
  for (int k=0; k<K+1; k++){ t(k) = std::cos(theta(K-k)); }

  Eigen::ArrayXd v = (VectorXd::Constant(K-1,1.0)).array();
  Eigen::ArrayXd theta_ii = theta.segment(1,K-1);
  Eigen::ArrayXd theta_ii_k = theta_ii;

  // assumes K even
  w(0) = 1.0/(1.0*K*K-1.0);
  w(K) = w(0);
  for (int k=1; k<(K/2); k++){
    theta_ii_k = 2.0*k*theta_ii;
    v -= (2.0/( 4.0*k*k-1.0 ))*theta_ii_k.cos();
  }
  theta_ii_k = 1.0*K*theta_ii;
  v -= (1.0/(1.0*K*K-1.0))*theta_ii_k.cos();

  w.segment(1,K-1) = (2.0/(1.0*K))*v.matrix();
}

} // namespace geodesic
