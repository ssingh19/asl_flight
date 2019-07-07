#include <control/ccmcontroller.h>
#include <Eigen/Dense>
#include <math.h>
#include <GeoProb/Metric.h>


/********* Constructor ***********/
CCMController::CCMController(const double N):
    active(false),
    mea_pos(0.0,0.0,0.0), mea_vel(0.0,0.0,0.0), euler(0.0,0.0,0.0), mea_wb(0.0,0.0,0.0), fz(9.8066),
    _xc_nom(10),_uc_nom(0.0,0.0,0.0,0.0),_xc_mid(10),_xc(10),_xc_nom_dot(10),_xc_dot(10),_Xc_dot(10),
    E(0.0),uc_fb(0.0,0.0,0.0,0.0),fz_dot(0.0),euler_dot(0.0,0.0,0.0),r_wb(0.0,0.0,0.0),
    fzCmd(9.8066),dt(1.0),fz_dot_sum(0.0),filter_N(N),
    g(9.8066), lambda(1.28){

  // Measured quantity
  mea_R = Eigen::Matrix3d::Identity();
  _R_des = Eigen::Matrix3d::Identity();

  // CCM variables
  _xc_nom.setZero(); _xc_nom(6) = g;
  _xc.setZero(); _xc(6) = g;
  _xc_mid.setZero(); _xc_mid(6) = g;

  _xc_nom_dot.setZero();
  _xc_dot.setZero();

  _W_nom = Eigen::MatrixXd::Identity(9,9);
  _W_mid = _W_nom;
  _W = _W_nom;
  _M_nom = Eigen::MatrixXd::Identity(10,10);
  _M_mid = _M_nom;
  _M = _M_nom;

  double d_bar = 0.0420;
  double yaw_bound = 10.0*PI/180.0;
  _M_yaw = std::pow((d_bar/yaw_bound),2.0);
  _M_nom(9,9) = _M_yaw;
  _M_mid(9,9) = _M_yaw;
  _M(9,9) = _M_yaw;

  _B_ctrl = Eigen::MatrixXd(10,4);
  _B_ctrl << Eigen::MatrixXd::Zero(6,4),
             Eigen::MatrixXd::Identity(4,4);

  // Initialize Controller
  setMode(false);

  // Readout
  std::cout << "CCM initialized " << std::endl;
}

void CCMController::setMode(const bool _m){
  active = _m;
  if (active==false){
    // reset
    fzCmd = g;
    fz_dot = 0.0;
    uc_fb.setZero();
    euler_dot.setZero();
  }
}

/********* State update ***********/
void CCMController::updateState(const Eigen::Vector3d &r, const Eigen::Matrix3d &R,
                                const Eigen::Vector3d &v, const Eigen::Vector3d &w,
                                const double _fz, const double _dt, const int pose_up, const int vel_up) {

  dt = _dt;

  if (vel_up == 0) {
    fz = fzCmd;
  } else {
    fz = _fz;
  }

  if (pose_up == 0 && active) {

    Eigen::Vector3d accel = -fz*R.col(2);
    accel(2) += g;
    mea_pos += mea_vel*dt + 0.5*accel*(dt*dt);

  } else { mea_pos = r;}

  if (vel_up == 0 && active) {

    Eigen::Vector3d accel = -fz*R.col(2);
    accel(2) += g;
    mea_vel += accel*dt;

  } else {  mea_vel = v;}

  mea_R = R;
  mea_wb = w;
  R2euler_123();

}

/********* Coordinate conversions ***********/
void CCMController::R2euler_123(void){
  euler(1) = std::asin(mea_R(0,2));
  euler(0) = std::atan2(-mea_R(1,2),mea_R(2,2));
  euler(2) = std::atan2(-mea_R(0,1),mea_R(0,0));
}

void CCMController::Euler2R_123(const double r, const double p, const double y){
  _R_des <<  cos(p)*cos(y),-cos(p)*sin(y), sin(p),
             cos(r)*sin(y)+cos(y)*sin(p)*sin(r), cos(r)*cos(y)-sin(p)*sin(r)*sin(y), -cos(p)*sin(r),
             sin(r)*sin(y)-cos(r)*cos(y)*sin(p), cos(y)*sin(r)+cos(r)*sin(p)*sin(y),  cos(p)*cos(r);
}

void CCMController::R_om(const Eigen::Vector3d &q, const Eigen::Vector3d &q_dot){

  Eigen::Matrix3d R_om;
  R_om << std::cos(q(1))*std::cos(q(2)), std::sin(q(2)), 0.0,
         -std::cos(q(1))*std::sin(q(2)), std::cos(q(2)), 0.0,
          std::sin(q(1)), 0.0, 1.0;

  r_wb = R_om * q_dot;
}

/********* CCM fncs ***********/
void CCMController::calc_xc_uc_nom(const Eigen::Vector3d &r_pos,
                const Eigen::Vector3d &r_vel,
                const Eigen::Vector3d &r_acc,
                const Eigen::Vector3d &r_jer,
                const double yaw_des,
                const double yaw_dot_des){

  // yaw_des w.r.t 321

  _xc_nom(0) = r_pos(0); _xc_nom(1) = r_pos(1); _xc_nom(2) = r_pos(2);
  _xc_nom(3) = r_vel(0); _xc_nom(4) = r_vel(1); _xc_nom(5) = r_vel(2);

  Eigen::Vector3d th_vec(r_acc(0), r_acc(1), r_acc(2)-g);
  _xc_nom(6) = th_vec.norm();

  Eigen::Vector3d zb = -th_vec.normalized();

  Eigen::Vector3d yc(-sin(yaw_des),cos(yaw_des),0.0);

  Eigen::Vector3d xb_des = yc.cross(zb);
  Eigen::Vector3d xb = xb_des.normalized();

  Eigen::Vector3d yb = zb.cross(xb);

  _xc_nom(7) = std::atan2(-zb(1),zb(2));
  _xc_nom(8) = std::asin(zb(0));
  _xc_nom(9) = std::atan2(-yb(0), xb(0));

  double om_x, om_y, om_z = 0.0;

  if (_xc_nom(6) > 0.0) {
    om_x =  (1.0/_xc_nom(6)) * ( r_jer.dot(yb) );
    om_y = -(1.0/_xc_nom(6)) * ( r_jer.dot(xb) );
  }

  Eigen::Vector3d zb_dot = om_y*xb - om_x*yb;

  Eigen::Vector3d yc_dot(-yaw_dot_des*cos(yaw_des), -yaw_dot_des*sin(yaw_des), 0.0);

  Eigen::Vector3d xb_des_dot = yc_dot.cross(zb) + yc.cross(zb_dot);

  if (xb_des.norm() > 0.0) {
    om_z = yb.dot( (1.0/xb_des.norm()) * xb_des_dot );
  }

  _uc_nom(0) = th_vec.normalized().dot(r_jer);

  _uc_nom(1) = om_x*(cos(_xc_nom(9))/cos(_xc_nom(8)))-
               om_y*(sin(_xc_nom(9))/cos(_xc_nom(8)));

  _uc_nom(2) = sin(_xc_nom(9))*om_x + cos(_xc_nom(9))*om_y;

  _uc_nom(3) = -cos(_xc_nom(9))*tan(_xc_nom(8)) * om_x +
                sin(_xc_nom(9))*tan(_xc_nom(8)) * om_y +
                om_z;
}

void CCMController::calc_CCM_dyn(const Eigen::VectorXd &xc,const Eigen::Vector4d &uc,
                                  Eigen::VectorXd &dyn){

  double r = xc(7);
  double p = xc(8);
  double y = xc(9);

  Eigen::Vector3d b_T(sin(p), -cos(p)*sin(r), cos(p)*cos(r));

  Eigen::VectorXd f(10);
  f << xc(3), xc(4), xc(5), -xc(6)*b_T(0), -xc(6)*b_T(1), g-xc(6)*b_T(2), 0.,0.,0.,0.;

  dyn = f + _B_ctrl*uc;

}

/********* Return fncs ***********/
double CCMController::getE(){
  return E;
}

double CCMController::getfz(){
  return fzCmd;
}

Eigen::Vector3d CCMController::getEr(){
  return euler_dot;
}

Eigen::Vector3d CCMController::getOm(){
  return r_wb;
}

Eigen::Vector3d CCMController::getEuler(){
  return euler;
}

double CCMController::getYawNom(){
  return _xc_nom(9);
}

/********* CCM compute ***********/
void CCMController::calcCCM(const double yaw_des, const double yaw_dot_des, const Eigen::Vector3d &r_pos, const Eigen::Vector3d &r_vel,
          const Eigen::Vector3d &r_acc, const Eigen::Vector3d &r_jer) {


  if (active){
    // Compute nominal xc and uc
    calc_xc_uc_nom(r_pos,r_vel,r_acc,r_jer,yaw_des,yaw_dot_des);

    // Now compute actual xc
    _xc << mea_pos, mea_vel, fzCmd, euler;

    // Correct yaw
    if ( std::max( abs(_xc_nom(9)) , abs(_xc(9)) ) > M_PI/2.0 ) {
      _xc_nom(9) = _xc_nom(9) > 0.0 ? _xc_nom(9) : _xc_nom(9) + 2*M_PI;
      _xc(9) = _xc(9) > 0.0 ? _xc(9) : _xc(9) + 2*M_PI;
    }
    _xc_mid = 0.5 * (_xc_nom + _xc);


    // Straight-line appproximation of geodesic
    _Xc_dot = _xc - _xc_nom;

    // Compute auxiliaries
    geodesic::compute_W(_xc_nom, _W_nom);
    geodesic::compute_W(_xc_mid, _W_mid);
    geodesic::compute_W(_xc, _W);

    _M_nom.block(0,0,9,9) = _W_nom.inverse();
    _M_mid.block(0,0,9,9) = _W_mid.inverse();
    _M.block(0,0,9,9) = _W.inverse();

    calc_CCM_dyn(_xc_nom, _uc_nom, _xc_nom_dot);
    calc_CCM_dyn(_xc, _uc_nom, _xc_dot);

    // Now do the computation
    E = 0.5*( (1.0/3.0)*_Xc_dot.dot(_M_nom*_Xc_dot)+
              (4.0/3.0)*_Xc_dot.dot(_M_mid*_Xc_dot)+
              (1.0/3.0)*_Xc_dot.dot(_M*_Xc_dot) );

    double a = 2.0*lambda*E - 2.0*_Xc_dot.dot(_M_nom*_xc_nom_dot)+
                              2.0*_Xc_dot.dot(_M*_xc_dot);
    // Eigen::Vector4d b = (2.0*_Xc_dot.transpose()*_M*_B_ctrl).transpose();
    Eigen::Vector4d b = 2.0*_B_ctrl.transpose()*_M*_Xc_dot;

    // uc_fb.setZero();
    if (b.norm()> 0.0 && a > 0.0){
      // uc_fb = -(a/b.norm()) * b.normalized();
      uc_fb = (uc_fb/2.0) - ((a+b.dot(uc_fb)/2.0)/b.norm())*b.normalized();
    } else {
      uc_fb.setZero();
    }

    // Manual yaw_rate fb
    // uc_fb(3) = 2.8 * (_xc_nom(9) - _xc(9));

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
    ROS_INFO("E:%.3f, uc_fb:(%.3f,%.3f,%.3f,%.3f)",E,uc_fb(0),uc_fb(1),uc_fb(2),uc_fb(3));
    ROS_INFO("E:%.3f, uc:(%.3f,%.3f,%.3f,%.3f)",E,uc_fb(0)+_uc_nom(0),uc_fb(1)+_uc_nom(1),uc_fb(2)+_uc_nom(2),uc_fb(3)+_uc_nom(3));
    */

    // Update fz_dot
    fz_dot_sum += _uc_nom(0)+uc_fb(0) - (fz_dot_sum/filter_N);
    fz_dot = fz_dot_sum/filter_N;

    // Update thrust
    fzCmd = fzCmd + fz_dot*dt;

    // Update desired euler_dot_rate
    for (int i = 0; i<3; i++){
      euler_dot(i) = std::max(std::min(_uc_nom(i+1)+uc_fb(i+1),5.0),-5.0);
    }

  } else {
    uc_fb.setZero();
    euler_dot.setZero();
    fz_dot = 0.0;
    fzCmd = g;
  }

  // Convert to desired omega (r_wb)
  R_om(euler,euler_dot);

}
