#include <control/ccmcontroller.h>
#include <Eigen/Dense>
#include <math.h>


/********* Constructor ***********/
CCMController::CCMController(void):
    MODEL("aslquad"),  M(1.04), g(9.8066), TCOEFF(4.4), KW(0.05),lambda(0.1),
    mea_pos(0.0,0.0,0.0), mea_vel(0.0,0.0,0.0), euler(0.0,0.0,0.0), mea_wb(0.0,0.0,0.0), dt(1.0),
    fz(0.0),fz_dot(0.0), fzCmd(0.0), fzCmd_prev(0.0), active(false),
    uc_fb(0.0,0.0,0.0,0.0), euler_dot(0.0,0.0,0.0), r_wb(0.0,0.0,0.0),tauCmd(0.0,0.0,0.0),
    _xc_nom(10), _uc_nom(0.0,0.0,0.0,0.0), _xic_nom(10), _xc(10), _xic(10), _xc_nom_dot(10), _xc_dot(10) {

  // handle ros parameters
  ros::param::get("~KW", KW);
  ros::param::get("~M", M);
  ros::param::get("~TCOEFF", TCOEFF);
  ros::param::get("~MODEL", MODEL);

  // W and J
  if(MODEL=="aslquad") {
    W << 1, 1, 1, 1,
        -0.12, 0.12, 0.12, -0.12,
        -0.12, 0.12, -0.12, 0.12,
        -0.06, -0.06, 0.06, 0.06;

    J = Eigen::Matrix3d::Identity(3,3);

  } else if(MODEL=="iris") {
    // Iris in Gazebo (ENU torque command)
    W << 1, 1, 1, 1,
        -0.22, 0.2, 0.22, -0.2,
        -0.13, 0.13, -0.13, 0.13,
        -0.06, -0.06, 0.06, 0.06;

   J << 0.0347563, 0.0, 0.0,
        0.0, 0.0458929, 0.0,
        0.0, 0.0, 0.0977;

  } else {
    ROS_ERROR("Unknown model name!");
  }

  // Measured quantity
  mea_R = Eigen::Matrix3d::Identity();

  // CCM variables
  _xc_nom.setZero();
  _xic_nom.setZero();
  _xc.setZero();
  _xic.setZero();
  _xc_nom_dot.setZero();
  _xc_dot.setZero();
  _xc_nom(6) = 0.0;
  _xc_nom(9) = 0.0;

  _M_nom = Eigen::MatrixXd::Identity(9,9);
  _M = Eigen::MatrixXd::Identity(9,9);

  // double d_bar = 0.0283;
  double d_bar = 0.215;
  double yaw_bound = 5.0*M_PI/180.0;
  double _M_yaw = std::pow((d_bar/yaw_bound),2.0);

  _B_ctrl = Eigen::MatrixXd(10,4);
  _B_ctrl << Eigen::MatrixXd::Zero(6,4),
             Eigen::MatrixXd::Identity(4,4);

  ROS_INFO("ccm init here");

  _M_xi = Eigen::MatrixXd(10,10);

  // _M_xi <<
 //  0.1071,    0.0000,    0.0000,    0.0755,   -0.0000,   -0.0000,    0.0144,   -0.0000,   -0.0000,         0.0,
 //  0.0000,    0.1071,    0.0000,    0.0000,    0.0755,   -0.0000,    0.0000,    0.0144,   -0.0000,         0.0,
 //  0.0000,    0.0000,    0.1071,    0.0000,    0.0000,    0.0755,    0.0000,    0.0000,    0.0144,         0.0,
 //  0.0755,    0.0000,    0.0000,    0.0796,   -0.0000,   -0.0000,    0.0165,   -0.0000,   -0.0000,         0.0,
 // -0.0000,    0.0755,    0.0000,   -0.0000,    0.0796,   -0.0000,   -0.0000,    0.0165,   -0.0000,         0.0,
 // -0.0000,   -0.0000,    0.0755,   -0.0000,   -0.0000,    0.0796,   -0.0000,   -0.0000,    0.0165,         0.0,
 //  0.0144,    0.0000,    0.0000,    0.0165,   -0.0000,   -0.0000,    0.0082,   -0.0000,   -0.0000,         0.0,
 // -0.0000,    0.0144,    0.0000,   -0.0000,    0.0165,   -0.0000,   -0.0000,    0.0082,   -0.0000,         0.0,
 // -0.0000,   -0.0000,    0.0144,   -0.0000,   -0.0000 ,   0.0165,   -0.0000,   -0.0000,    0.0082,         0.0,
 //  Eigen::MatrixXd::Zero(1,9), _M_yaw;

  _M_xi <<
 0.0232,   -0.0000,   -0.0000,    0.0069,   -0.0000,   -0.0000,    0.0116,   -0.0000,   -0.0000,         0,
-0.0000,    0.0232,   -0.0000,    0.0000,    0.0069,   -0.0000,   -0.0000,    0.0116,   -0.0000,         0,
-0.0000,   -0.0000,    0.0232,    0.0000,    0.0000,    0.0069,   -0.0000,   -0.0000,    0.0116,         0,
 0.0069,    0.0000,    0.0000,    0.0462,    0.0000,    0.0000,    0.0057,   -0.0000,   -0.0000,         0,
-0.0000,    0.0069,    0.0000,    0.0000,    0.0462,    0.0000,    0.0000,    0.0057,   -0.0000,         0,
-0.0000,   -0.0000,    0.0069,    0.0000,    0.0000,    0.0462,    0.0000,    0.0000,    0.0057,         0,
 0.0116,   -0.0000,   -0.0000,    0.0057,    0.0000,    0.0000,    0.0284,    0.0000,    0.0000,         0,
-0.0000,    0.0116,   -0.0000,   -0.0000,    0.0057,    0.0000,    0.0000,    0.0284,    0.0000,         0,
-0.0000,   -0.0000,    0.0116,   -0.0000,   -0.0000,    0.0057,    0.0000,    0.0000,    0.0284,         0,
Eigen::MatrixXd::Zero(1,9), _M_yaw;

  ROS_INFO("ccm init here");

  std::cout << "CCM Using the following parameters: " << std::endl;
  std::cout << "KW = " << KW << std::endl;
  std::cout << "M = " << M << std::endl;
  std::cout << "TCOEFF = " << TCOEFF << std::endl;
  std::cout << "MODEL: " << MODEL << std::endl;

}

/********* State update ***********/
void CCMController::updateState(const Eigen::Vector3d &r, const Eigen::Matrix3d &R,
                                const Eigen::Vector3d &v, const Eigen::Vector3d &w,
                                const double _dt, const int pos_type, const int vel_type) {

  dt = _dt;
  mea_pos = r;
  mea_vel = v;
  mea_R = R;
  mea_wb = w;
  R2euler_321();


  // POSE
  // if (pos_type == 0){
  //   // need to integrate
  //   Eigen::Vector3d int_vel = mea_vel;
  //   Eigen::Vector3d int_om = r_wb;
  //   if (vel_type == 1) {
  //     // use midpoint integration
  //     int_vel = 0.5*(mea_vel+v);
  //     //int_om = 0.5*(mea_wb+w);
  //   }
  //   mea_pos += int_vel*dt;
  //
  //   // Eigen::Matrix3d R_up = R_rot(int_om.norm()*dt,int_om.normalized());
  //   // mea_R = mea_R*R_up;
  //   mea_R = R;
  //
  // } else {
  //   // copy over
  //   mea_pos = r;
  //   mea_R = R;
  // }
  // R2euler_123();
  //
  // // Velocity
  // if (vel_type == 0){
  //   // need to integrate
  //   if (active && mea_pos(2)<=-0.2){ //only update if active & off the ground
  //     Eigen::Vector3d accel(0.0,0.0,g);
  //     accel -= fzCmd*mea_R.col(2);
  //     mea_vel += accel*dt;
  //   } else { mea_vel = v; }
  //   mea_wb = w;
  // } else {
  //   // type 2 - copy over
  //   mea_vel = v;
  //   mea_wb = w;
  // }

}

Eigen::Matrix3d CCMController::R_rot(const double a, const Eigen::Vector3d n){
  double c = std::cos(a/2.0);
  double s = std::sin(a/2.0);
  double q1 = c;
  double q2 = s*n(0);
  double q3 = s*n(1);
  double q4 = s*n(2);
  double q1s = q1*q1;
  double q2s = q2*q2;
  double q3s = q3*q3;
  double q4s = q4*q4;

  Eigen::Matrix3d R;
  R <<
    q1s+q2s-q3s-q4s, 2.0*(q2*q3-q1*q4) ,2.0*(q2*q4+q1*q3),
    2.0*(q2*q3+q1*q4), q1s-q2s+q3s-q4s, 2.0*(q3*q4-q1*q2),
    2.0*(q2*q4-q1*q3), 2.0*(q3*q4+q1*q2), q1s-q2s-q3s+q4s;
}

void CCMController::R2euler_123(void){
  euler(1) = std::asin(mea_R(0,2));
  euler(0) = std::atan2(-mea_R(1,2),mea_R(2,2));
  euler(2) = std::atan2(-mea_R(0,1),mea_R(0,0));
}

void CCMController::R2euler_321(void){
  euler(0) = std::atan2(mea_R(2,1),mea_R(2,2));
  euler(1) = std::atan2(-mea_R(2,0),mea_R(2,1)*std::sin(euler(0))+mea_R(2,2)*std::cos(euler(0)));
  euler(2) = std::atan2(mea_R(1,0),mea_R(0,0));
}

void CCMController::setfz(double _fz){
  fz = _fz;
}

/********* CCM fncs ***********/
void CCMController::calc_xc_uc_nom(const Eigen::Vector3d &r_pos,
                const Eigen::Vector3d &r_vel,
                const Eigen::Vector3d &r_acc,
                const Eigen::Vector3d &r_jer,
                const double yaw_des){

  _xc_nom(0) = r_pos(0); _xc_nom(1) = r_pos(1); _xc_nom(2) = r_pos(2);
  _xc_nom(3) = r_vel(0); _xc_nom(4) = r_vel(1); _xc_nom(5) = r_vel(2);

  Eigen::Vector3d th_vec(r_acc(0), r_acc(1), r_acc(2)-g);
  _xc_nom(6) = th_vec.norm();

  Eigen::Vector3d zb_des = -th_vec.normalized();
  Eigen::Vector3d yc_des(-std::sin(yaw_des), std::cos(yaw_des), 0.0);
  Eigen::Vector3d xb = yc_des.cross(zb_des);
  Eigen::Vector3d xb_des = xb.normalized();
  Eigen::Vector3d yb_des = zb_des.cross(xb_des);

  _xc_nom(7) = std::atan2(yb_des(2),zb_des(2));
  _xc_nom(8) = std::atan2(-xb_des(2),yb_des(2)*std::sin(euler(0))+zb_des(2)*std::cos(euler(0)));
  _xc_nom(9) = yaw_des;

  // _xc_nom(7) = std::atan2(r_acc(1),g-r_acc(2));
  // _xc_nom(8) = std::asin(-r_acc(0)/_xc_nom(6));
  // _xc_nom(9) = yaw_des;

  _uc_nom(0) = -zb_des.dot(r_jer);

  double om_1 = 0.0;
  double om_2 = 0.0;
  if (_xc_nom(6)>0.0) {
    om_1 = r_jer.dot(yb_des)*(1.0/_xc_nom(6));
    om_2 = -r_jer.dot(xb_des)*(1.0/_xc_nom(6));
  }

  _uc_nom(1) = om_1;
  _uc_nom(2) = om_2;
  // _uc_nom(1) = om_2*(std::cos(yaw_des)/std::cos(_xc(8))) +
  //              om_1*(std::sin(yaw_des)/std::cos(_xc(8)));
  // _uc_nom(2) = std::sin(yaw_des)*om_skew(1) - std::cos(yaw_des)*om_skew(0);
  _uc_nom(3) = 0.0;

}

void CCMController::Phi(const Eigen::VectorXd &xc, Eigen::VectorXd &xic){

  Eigen::Vector3d b_T(sin(xc(8)), -cos(xc(8))*sin(xc(7)), cos(xc(8))*cos(xc(7)));
  xic << xc(0), xc(1), xc(2), xc(3), xc(4), xc(5), -xc(6)*b_T(0), -xc(6)*b_T(1), g-xc(6)*b_T(2), xc(9);
}

void CCMController::M_fnc(const Eigen::VectorXd &xc, Eigen::MatrixXd &M){

  Eigen::Vector3d b_T(sin(xc(8)), -cos(xc(8))*sin(xc(7)), cos(xc(8))*cos(xc(7)));
  Eigen::MatrixXd db_T_q(3,2);
  db_T_q <<  0.0, cos(xc(8)),
            -cos(xc(7))*cos(xc(8)), sin(xc(7))*sin(xc(8)),
            -sin(xc(7))*cos(xc(8)), -cos(xc(7))*sin(xc(8));

  Eigen::MatrixXd Phi_jac(10,10);
  Phi_jac << Eigen::MatrixXd::Identity(6,6), Eigen::MatrixXd::Zero(6,4),
             Eigen::MatrixXd::Zero(3,6), -b_T, -db_T_q*xc(6), Eigen::MatrixXd::Zero(3,1),
             Eigen::MatrixXd::Zero(1,9), 1;

  M = _M_xi*Phi_jac;
}

void CCMController::dynamics(const Eigen::VectorXd &xc, const Eigen::Vector4d &uc, Eigen::VectorXd &dyn){

  Eigen::Vector3d b_T(sin(xc(8)), -cos(xc(8))*sin(xc(7)), cos(xc(8))*cos(xc(7)));

  Eigen::VectorXd f(10);
  f << xc(3), xc(4), xc(5), -xc(6)*b_T(0), -xc(6)*b_T(1), g-xc(6)*b_T(2), 0,0,0,0;

  dyn = f + _B_ctrl*uc;

}

void CCMController::R_om(const Eigen::Vector3d &q, const Eigen::Vector3d &q_dot){

  Eigen::Matrix3d R_om;
  R_om << std::cos(q(1))*std::cos(q(2)), std::sin(q(2)), 0.0,
         -std::cos(q(1))*std::sin(q(2)), std::cos(q(2)), 0.0,
          std::sin(q(1)), 0.0, 1.0;

  r_wb = R_om * q_dot;
}

/********* Return fncs ***********/
double CCMController::getE(){
  return E;
}

double CCMController::getfz(){
  return fzCmd;
}

Eigen::Vector3d CCMController::getEulerdot(){
  return euler_dot;
}

Eigen::Vector3d CCMController::getvel(){
  return mea_vel;
}

void CCMController::setMode(const bool _m){
  active = _m;
  if (active==false){
    // reset
    fz = 0.0;
    fzCmd = 0.0;
    fzCmd_prev = 0.0;
    fz_dot = 0.0;
    uc_fb.setZero();
    euler_dot.setZero();
    tauCmd.setZero();
  }
}

/********* CCM compute ***********/
void CCMController::calcCCM(const double &yaw_des, const Eigen::Vector3d &r_pos, const Eigen::Vector3d &r_vel,
          const Eigen::Vector3d &r_acc, const Eigen::Vector3d &r_jer) {


  // Record previous thrust
  fzCmd_prev = fzCmd;
  fz = fzCmd_prev;

  if (active){
    // Compute nominal xc and uc
    calc_xc_uc_nom(r_pos,r_vel,r_acc,r_jer,yaw_des);

    // Now compute actual xc
    _xc << mea_pos, mea_vel, fzCmd_prev, euler;

    // compute xic
    Phi(_xc_nom,_xic_nom);
    Phi(_xc,_xic);

    // compute M
    M_fnc(_xc_nom,_M_nom);
    M_fnc(_xc, _M);

    // compute dynamics
    dynamics(_xc_nom,_uc_nom,_xc_nom_dot);
    dynamics(_xc, _uc_nom, _xc_dot);

    // Now do the computation
    Eigen::VectorXd d_xic = _xic-_xic_nom;
    E = d_xic.dot(_M_xi*d_xic);

    double a = 2.0*lambda*E - 2.0*d_xic.dot(_M_nom*_xc_nom_dot) + 2.0*d_xic.dot(_M*_xc_dot);
    Eigen::MatrixXd b = (2.0*d_xic.transpose()*_M*_B_ctrl).transpose();

    uc_fb.setZero();
    if (b.norm()>1e-6  && a > 0) {
      uc_fb = -(a/b.squaredNorm()) * b;
    }

    // Debug controller comp
    /*
    ROS_INFO("xc_nom:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
             _xc_nom(0),_xc_nom(1),_xc_nom(2),
             _xc_nom(3),_xc_nom(4),_xc_nom(5),
             _xc_nom(6),_xc_nom(7),_xc_nom(8),
             _xc_nom(9));
    ROS_INFO("xc:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
              _xc(0),_xc(1),_xc(2),
              _xc(3),_xc(4),_xc(5),
              _xc(6),_xc(7),_xc(8),
              _xc(9));
    ROS_INFO("xc_dot:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
              _xc_dot(0),_xc_dot(1),_xc_dot(2),
              _xc_dot(3),_xc_dot(4),_xc_dot(5),
              _xc_dot(6),_xc_dot(7),_xc_dot(8),
              _xc_dot(9));
    ROS_INFO("E:%.3f, a:%.3f, b:(%.3f,%.3f,%.3f,%.3f)",E,a,b(0),b(1),b(2),b(3));
    */

    // Debug CCM output
    //ROS_INFO("E:%.3f, uc_fb:(%.3f,%.3f,%.3f,%.3f)",E,uc_fb(0),uc_fb(1),uc_fb(2),uc_fb(3));

    // Update fz_dot
    fz_dot = _uc_nom(0)+uc_fb(0);

    // Update thrust
    fzCmd = fzCmd_prev + fz_dot*dt;

    // Update desired euler_dot_rate
    euler_dot << _uc_nom(1)+uc_fb(1),_uc_nom(2)+uc_fb(2),_uc_nom(3)+uc_fb(3);

  } else {
    uc_fb.setZero();
    euler_dot.setZero();
    fz_dot = 0.0;
    fzCmd = 0.0;
  }

  // Convert to desired omega
  R_om(euler,euler_dot);

  Eigen::Vector3d ew = mea_wb-r_wb;
  tauCmd = -KW*ew + mea_wb.cross(J*mea_wb)- J*mea_wb.cross(r_wb);

  Eigen::Vector4d wrench(fzCmd*M, tauCmd(0), -tauCmd(1), -tauCmd(2));
  Eigen::Vector4d ffff = W.colPivHouseholderQr().solve(wrench);

  // std::cout << fzCmd << std::endl;

  for(int i=0; i<4; i++) {
    motorCmd[i] = ffff(i)/TCOEFF;
  }

}
