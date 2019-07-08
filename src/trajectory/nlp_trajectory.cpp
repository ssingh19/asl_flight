#include <trajectory/trajectory.h>
#include <math.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <trajectory/nlp_coeffs.h>


NLPTrajectory::NLPTrajectory(int _n, int _m, double _dt):
  n(_n), m(_m), N(NLP_N), dt(_dt), t_end(NLP_T),
  tau_grid(NLP_N+1){

  // start pos
  start_pos.setZero();

  // Lagrange Matrix
  Lagrange_interp = Eigen::MatrixXd(1,N+1);

  // collocation grid points
  for (int k=0; k<N+1; k++){ tau_grid(k) = std::cos(M_PI*(N-k)/N) ; }

}

void NLPTrajectory::compute_lagrange(const double t) {

  // convert to normalized time
  double tau = (2*t-t_end)/t_end;

  Lagrange_interp.setOnes();

  for (int j=0; j<N+1; j++){
    for (int k=0; k<N+1; k++){
      if (k!=j){
        Lagrange_interp(j) = Lagrange_interp(j) * (tau-tau_grid(k))/(tau_grid(j)-tau_grid(k));
      }
    }
  }

}

void NLPTrajectory::eval_explicit(double t, Eigen::VectorXd &_state_nom, Eigen::Vector2d &_cntrl_nom, int &ind_seg){

  // double local time within segments
  double t_;

  if (t <= t_end) {
    t_ = t;
    ind_seg = 1;
  } else if (t > t_end && t < 2.0*t_end) {
    t_ = t - t_end;
    ind_seg = 2;
  } else {
    t_ = t - 2.0*t_end;
    ind_seg = 3;
  }

  // compute lagrange polynomials
  compute_lagrange(std::min(t_,t_end));

  // get states at the two indices
  if (ind_seg ==1) {
    _state_nom = (Lagrange_interp * lagrange_x_ramp).transpose();
    _cntrl_nom = (Lagrange_interp * lagrange_u_ramp).transpose();
  } else if (ind_seg==2){
    _state_nom = (Lagrange_interp * lagrange_x_flat).transpose();
    _cntrl_nom = (Lagrange_interp * lagrange_u_flat).transpose();
  } else {
    _state_nom = (Lagrange_interp * lagrange_x_slow).transpose();
    _cntrl_nom = (Lagrange_interp * lagrange_u_slow).transpose();
  }

  // only adjust y
  _state_nom(0) = _state_nom(0) + start_pos(1);

}

void NLPTrajectory::set_start_pos(const Eigen::Vector3d &_start_pos)
{
	start_pos = _start_pos;
}
