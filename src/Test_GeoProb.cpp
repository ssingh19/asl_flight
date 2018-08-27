/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler, ETH Zurich. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <string>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <GeoProb/GeoProb.h>
#include <GeoProb/Geodesic.h>
#include <GeoProb/Metric.h>

using namespace ifopt;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Test_GeoProb");
  ros::NodeHandle nh;

  using Jacobian = Eigen::SparseMatrix<double, Eigen::RowMajor>;

  // 1. define the problem
  int N = 3; const int n = 9;

  Problem* nlp = new Problem;
  auto geovar_ptr = std::make_shared<GeoVariables>(N);
  auto geocon_ptr = std::make_shared<GeoConstraint>(N);

  nlp->AddVariableSet  (geovar_ptr);
  nlp->AddConstraintSet(geocon_ptr);
  nlp->AddCostSet      (std::make_shared<GeoCost>(N));
  nlp->PrintCurrent();

  std::cout << "init geo prob" << std::endl;

  // 2. choose solver and options
  IpoptSolver* ipopt = new IpoptSolver;
  ipopt->SetOption("linear_solver", "mumps");
  ipopt->SetOption("jacobian_approximation", "exact");
  ipopt->SetOption("tol",0.000000001);
  ipopt->SetOption("print_level",5);
  ipopt->SetOption("sb","yes");

  // 3. Set geodesic endpoints
  Eigen::VectorXd xc_nom(10);
  Eigen::VectorXd xc(10);
  xc_nom << -0.006,0.003,-0.330,0.000,0.000,-0.200,9.807,0.000,-0.000,0.000;
  xc <<  -0.006,0.003,0.039,0.001,0.005,0.005,13.140,-0.003,0.001,0.035;

  // 4. Update Problem (initial guess and constraints)
  geovar_ptr->UpdateInit(xc_nom,xc);
  geocon_ptr->update_bounds(xc_nom,xc);

  // 5 . solve
  // ipopt->Solve(*nlp);
  // Eigen::VectorXd c_opt = nlp->GetOptVariables()->GetValues();
  Eigen::VectorXd c_opt = nlp->GetVariableValues();

  // 6. convert to CCM quantities
  Eigen::MatrixXd X_EndP(n,2);
  Eigen::MatrixXd X_dot_EndP(n,2);
  Eigen::MatrixXd W_(n,n); W_.setZero();
  std::vector< Eigen::MatrixXd> W_EndP(2,W_);

  Eigen::MatrixXd T_EndP(N+1,2);
  Eigen::MatrixXd T_dot_EndP(N+1,2);
  Eigen::Vector2d t(-1.0,1.0);
  geodesic::compute_cheby(T_EndP,N,1,t);
  geodesic::compute_cheby_d(T_dot_EndP,N,1,t);

  Jacobian basis(n,n*(N+1));
  basis.reserve(Eigen::VectorXi::Constant(n,N+1));
  std::vector<Jacobian> Phi(2,basis);
  std::vector<Jacobian> Phi_dot(2,basis);
  for (int k=0; k<2; k++){
    for (int i=0; i<n; i++){
      for (int j=0; j<N+1; j++){
        Phi[k].insert(i,i*(N+1)+j) = T_EndP(j,k);
        Phi_dot[k].insert(i,i*(N+1)+j) = 2.0*T_dot_EndP(j,k);
      }
    }
  }

  X_EndP.col(0) = Phi[0]*c_opt;
  X_EndP.col(1) = Phi[1]*c_opt;
  X_dot_EndP.col(0) = Phi_dot[0]*c_opt;
  X_dot_EndP.col(1) = Phi_dot[1]*c_opt;

  geodesic::compute_W(xc_nom, W_EndP[0]);
  geodesic::compute_W(xc, W_EndP[1]);

  std::cout << std::setprecision(5) << "X:" << std::endl;
  std::cout << X_EndP.col(0).transpose() << std::endl;
  std::cout << X_EndP.col(1).transpose() << std::endl;

  std::cout << "X_dot:" << std::endl;
  std::cout << X_dot_EndP.col(0).transpose() << std::endl;
  std::cout << X_dot_EndP.col(1).transpose() << std::endl;

  std::cout << "J_opt:" << std::endl;
  std::cout << X_dot_EndP.col(0).dot(W_EndP[0].inverse()*X_dot_EndP.col(0)) << std::endl;
  std::cout << X_dot_EndP.col(1).dot(W_EndP[1].inverse()*X_dot_EndP.col(1)) << std::endl;

  return 0;
}
