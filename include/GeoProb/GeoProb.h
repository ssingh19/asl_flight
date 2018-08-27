#ifndef GEOPROB_H
#define GEOPROB_H

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include "Geodesic.h"

namespace ifopt {
using Eigen::MatrixXd;
using Eigen::RowVectorXd;


class GeoVariables : public VariableSet {
public:
  GeoVariables(int N) : GeoVariables("geo_var", N) {N_ = N;};
  // Raw initialize
  GeoVariables(const std::string& name, const int N) : VariableSet(9*(N+1), name)
  {
    x_ = VectorXd::Constant(9*(N+1),0.0);
    bounds_ = VecBound(9*(N+1),NoBound);

    std::cout << "init geo vars" << std::endl;
  }

  // Define initialization function
  void UpdateInit(const VectorXd& x_s, const VectorXd& x_end){
    for (int i=0; i<9; i++) {
      x_((N_+1)*i) = 0.5*(x_s(i)+x_end(i));
      x_((N_+1)*i+1) = -0.5*(x_s(i)-x_end(i));
    }
  }

  void SetVariables(const VectorXd& x) override { x_ = x; };

  VectorXd GetValues() const override { return x_; };

  // Return bounds
  VecBound GetBounds() const override { return bounds_;}
private:
  int N_;
  VectorXd x_;
  VecBound bounds_;
};


class GeoConstraint : public ConstraintSet {
public:
  GeoConstraint(int N) : GeoConstraint("geo_con", N) {}

  GeoConstraint(const std::string& name, const int N) : ConstraintSet(18, name) {
    // constants
    int K = 2*N;

    // Chebyshev basis for endpoint constraints
    MatrixXd Phi_EndP(N+1,2);
    Eigen::Vector2d t(-1.0,1.0);

    geodesic::compute_cheby(Phi_EndP,N,1,t);

    // Constraint
    g_ = VectorXd(18);

    // Constraint jacobian
    A_eq_ = Jacobian(18,(N+1)*9);
    A_eq_.reserve(Eigen::VectorXi::Constant(18,N+1));
    for (int i=0; i<9; i++){
      for (int j=0; j<N+1; j++){
        A_eq_.insert(i,i*(N+1)+j) = Phi_EndP(j,0);
      }
    }
    for (int i=9; i<18; i++){
      for (int j=0; j<N+1; j++){
        A_eq_.insert(i,(i-9)*(N+1)+j) = Phi_EndP(j,1);
      }
    }

    // Bounds
    b_ = VecBound(18,BoundZero);

    std::cout << "init geo con" << std::endl;
  }

  // Update constraint bound
  void update_bounds(const VectorXd& x_s, const VectorXd& x_end){

    for (int i = 0; i<9; i++){
      b_[i].lower_ = x_s(i);
      b_[i].upper_ = x_s(i);
    }
    for (int i = 9; i<18; i++){
      b_[i].lower_ = x_end(i-9);
      b_[i].upper_ = x_end(i-9);
    }

    //std::cout << "updated bounds" << std::endl;
  }

  VectorXd GetValues() const override
  {
    g_ = A_eq_*(GetVariables()->GetComponent("geo_var")->GetValues());
    return g_;
  };

  VecBound GetBounds() const override { return b_; }

  void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override
  {
    if (var_set == "geo_var") {
      jac_block = A_eq_;
    }
  }

private:
  mutable VectorXd g_;
  Jacobian A_eq_;
  VecBound b_;
};


class GeoCost: public CostTerm {
public:
  GeoCost(int N) : GeoCost("geo_cost", N) {}
  GeoCost(const std::string& name, const int N) : CostTerm(name) {
    // constants
    N_ = N; n_ = 9; K_ = 2*N_;

    // integration knot points
    VectorXd t(K_+1);
    w_ = VectorXd(K_+1);
    geodesic::clencurt(K_,t,w_);

    // Chebyshev basis
    T_ = MatrixXd(N_+1,K_+1);
    MatrixXd T_dot(N_+1,K_+1);
    geodesic::compute_cheby(T_,N_,K_,t);
    geodesic::compute_cheby_d(T_dot,N_,K_,t);

    Jacobian basis(n_,n_*(N_+1));
    basis.reserve(Eigen::VectorXi::Constant(n_,N_+1));

    Phi_ = std::vector<Jacobian>(K_+1,basis);
    Phi_dot_ = std::vector<Jacobian>(K_+1,basis);
    for (int k=0; k<K_+1; k++){
      for (int i=0; i<n_; i++){
        for (int j=0; j<N_+1; j++){
          Phi_[k].insert(i,i*(N_+1)+j) = T_(j,k);
          Phi_dot_[k].insert(i,i*(N_+1)+j) = 2.0*T_dot(j,k);
        }
      }
    }

    // Intermediates
    X_ = MatrixXd(n_,K_+1);
    MXD_ = MatrixXd(n_,K_+1);
    W_ = MatrixXd::Identity(n_,n_);
    dW_ = MatrixXd::Identity(n_,n_);

    // jacobian
    J_ = RowVectorXd::Constant(n_*(N_+1),0.0);

    // Coeffs
    c_ = VectorXd::Constant(n_*(N_+1),0.0);

    std::cout << "init geo cost" << std::endl;
  }

  double GetCost() const override {
    c_ = GetVariables()->GetComponent("geo_var")->GetValues();
    double J = 0.0;
    VectorXd xdot(9);
    for (int k=0;k<K_+1;k++) {
      X_.col(k) = Phi_[k]*c_;
      xdot = Phi_dot_[k]*c_;

      geodesic::compute_W(X_.col(k),W_);
      MXD_.col(k) = W_.inverse()*xdot;

      J += 0.5*w_(k)*xdot.dot(MXD_.col(k));
    }
  }

  void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override {

    if (var_set == "geo_var") {
      J_.setZero();
      for (int k=0; k<K_+1; k++) {
        J_ += w_(k)*MXD_.col(k).transpose()*Phi_dot_[k];

        for (int j=6; j<n_; j++){
          geodesic::compute_dW(X_.col(k),dW_,j);
          double a = MXD_.col(k).dot(dW_*MXD_.col(k));
          J_.segment(j*(N_+1),N_+1) -= 0.5*w_(k)*a*T_.col(k).transpose();
        }
      }

      // copy over
      jac_block.reserve((N_+1)*n_);
      for (int i=0; i<n_*(N_+1); i++){
        jac_block.insert(0,i) = J_(i);
      }
    }

  }

private:
  int N_, n_, K_;
  VectorXd w_;
  mutable VectorXd c_;
  mutable RowVectorXd J_;
  MatrixXd T_;
  mutable MatrixXd X_, MXD_, W_, dW_;
  std::vector<Jacobian> Phi_;
  std::vector<Jacobian> Phi_dot_;
};

} // namespace opt

#endif
