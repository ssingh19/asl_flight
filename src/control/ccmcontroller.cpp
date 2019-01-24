#include <control/ccmcontroller.h>
#include <Eigen/Dense>
#include <math.h>
#include <GeoProb/Metric.h>


/********* Constructor ***********/
CCMController::CCMController(void):
    active(false),
    mea_pos(0.0,0.0,0.0), mea_vel(0.0,0.0,0.0), euler(0.0,0.0,0.0), mea_wb(0.0,0.0,0.0), fz(9.8066),
    _xc_nom(10),_uc_nom(0.0,0.0,0.0,0.0),_xc(10),_xc_nom_dot(10),_xc_dot(10),
    E(0.0),uc_fb(0.0,0.0,0.0,0.0),fz_dot(0.0),euler_dot(0.0,0.0,0.0),r_w_nom(0.0,0.0,0.0),r_wb(0.0,0.0,0.0),
    fzCmd(9.8066),tauCmd(0.0,0.0,0.0),dt(1.0),
    MODEL("aslquad"),  mass(1.04), g(9.8066), TCOEFF(4.4), KW(0.05),lambda(1.25){

  // handle ros parameters
  ros::param::get("~KR", KR);
  ros::param::get("~KW", KW);
  ros::param::get("~M", mass);
  ros::param::get("~TCOEFF", TCOEFF);
  ros::param::get("~MODEL", MODEL);

  // A and J
  if(MODEL=="aslquad") {
    A << 1, 1, 1, 1,
        -0.12, 0.12, 0.12, -0.12,
        -0.12, 0.12, -0.12, 0.12,
        -0.06, -0.06, 0.06, 0.06;

    J = Eigen::Matrix3d::Identity(3,3);

  } else if(MODEL=="iris") {
    // Iris in Gazebo (ENU torque command)
    A << 1, 1, 1, 1,
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
  _R_des = Eigen::Matrix3d::Identity();

  // CCM variables
  _xc_nom.setZero(); _xc_nom(6) = g;
  _xc.setZero(); _xc(6) = g;
  _xc_nom_dot.setZero();
  _xc_dot.setZero();
  _W_nom = Eigen::MatrixXd::Identity(9,9);
  _W = _W_nom;
  _M_nom = Eigen::MatrixXd::Identity(10,10);
  _M = _M_nom;

  double d_bar = 0.0420;
  double yaw_bound = 10.0*PI/180.0;
  _M_yaw = std::pow((d_bar/yaw_bound),2.0);
  _M_nom(9,9) = _M_yaw;
  _M(9,9) = _M_yaw;

  _B_ctrl = Eigen::MatrixXd(10,4);
  _B_ctrl << Eigen::MatrixXd::Zero(6,4),
             Eigen::MatrixXd::Identity(4,4);


  // Initialize geodesic endpoints
  X_dot_EndP = Eigen::MatrixXd(10,2);

  // Initialize Controller
  setMode(false);

  // Readout
  std::cout << "CCM Using the following parameters: " << std::endl;
  std::cout << "KW = " << KW << std::endl;
  std::cout << "M = " << mass << std::endl;
  std::cout << "TCOEFF = " << TCOEFF << std::endl;
  std::cout << "MODEL: " << MODEL << std::endl;

}

void CCMController::setMode(const bool _m){
  active = _m;
  if (active==false){
    // reset
    fzCmd = g;
    fz_dot = 0.0;
    uc_fb.setZero();
    euler_dot.setZero();
    tauCmd.setZero();
  }
}

/********* State update ***********/
void CCMController::updateState(const Eigen::Vector3d &r, const Eigen::Matrix3d &R,
                                const Eigen::Vector3d &v, const Eigen::Vector3d &w,
                                const double _fz, const double _dt, const int pose_up, const int vel_up) {

  dt = _dt;
  if (pose_up == 0 && active) {
    mea_pos += 0.5*(v+mea_vel)*dt;
  } else { mea_pos = r;}
  if (vel_up == 0 && active) {
    Eigen::Vector3d accel = -fzCmd*mea_R.col(2);
    accel(2) += g;
    mea_vel += accel*dt;
    fz = fzCmd;
  } else {mea_vel = v; fz = _fz;}

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
                const double yaw_des){

  _xc_nom(0) = r_pos(0); _xc_nom(1) = r_pos(1); _xc_nom(2) = r_pos(2);
  _xc_nom(3) = r_vel(0); _xc_nom(4) = r_vel(1); _xc_nom(5) = r_vel(2);

  Eigen::Vector3d th_vec(r_acc(0), r_acc(1), r_acc(2)-g);
  _xc_nom(6) = th_vec.norm();

  _xc_nom(7) = std::atan2(r_acc(1),g-r_acc(2));
  _xc_nom(8) = std::asin(-r_acc(0)/_xc_nom(6));
  _xc_nom(9) = yaw_des;

  _uc_nom(0) = th_vec.normalized().dot(r_jer);

  Euler2R_123(_xc_nom(7),_xc_nom(8),_xc_nom(9));

  double om_1 = 0.0;
  double om_2 = 0.0;
  if (_xc_nom(6)>0.0) {
    om_1 = r_jer.dot(_R_des.col(1))*(1.0/_xc_nom(6));
    om_2 = -r_jer.dot(_R_des.col(0))*(1.0/_xc_nom(6));
  }

  _uc_nom(1) = om_1*(cos(yaw_des)/cos(_xc_nom(8)))-
               om_2*(sin(yaw_des)/cos(_xc_nom(8)));
  _uc_nom(2) = sin(yaw_des)*om_1 + cos(yaw_des)*om_2;
  _uc_nom(3) = 0.0;

  double om_3 = sin(_xc_nom(7))*_uc_nom(1)+_uc_nom(3);
  r_w_nom << om_1, om_2, om_3;

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

Eigen::Vector3d CCMController::getOm(){
  return r_wb;
}

Eigen::Vector3d CCMController::getEuler(){
  return euler;
}

/********* CCM compute ***********/
void CCMController::calcCCM(const double &yaw_des, const Eigen::Vector3d &r_pos, const Eigen::Vector3d &r_vel,
          const Eigen::Vector3d &r_acc, const Eigen::Vector3d &r_jer) {


  if (active){
    // Compute nominal xc and uc
    calc_xc_uc_nom(r_pos,r_vel,r_acc,r_jer,yaw_des);

    // Now compute actual xc
    _xc << mea_pos, mea_vel, fz, euler;

    // Straight-line appproximation of geodesic
    X_dot_EndP.col(0) = _xc - _xc_nom;
    X_dot_EndP.col(1) = _xc - _xc_nom;

    // Compute auxiliaries
    geodesic::compute_W(_xc_nom, _W_nom);
    geodesic::compute_W(_xc, _W);
    _M_nom.block(0,0,9,9) = _W_nom.inverse();
    _M.block(0,0,9,9) = _W.inverse();
    calc_CCM_dyn(_xc_nom, _uc_nom, _xc_nom_dot);
    calc_CCM_dyn(_xc, _uc_nom, _xc_dot);

    // Now do the computation
    E = 0.5*( X_dot_EndP.col(0).dot(_M_nom*X_dot_EndP.col(0))+
              X_dot_EndP.col(1).dot(_M*X_dot_EndP.col(1)) );

    double a = 2.0*lambda*E - 2.0*X_dot_EndP.col(0).dot(_M_nom*_xc_nom_dot)+
                              2.0*X_dot_EndP.col(1).dot(_M*_xc_dot);
    Eigen::Vector4d b = (2.0*X_dot_EndP.col(1).transpose()*_M*_B_ctrl).transpose();

    uc_fb.setZero();
    if (b.norm()> 0.0 && a > 0.0){
      uc_fb = -(a/b.norm()) * b.normalized();
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
    ROS_INFO("E:%.3f, uc_fb:(%.3f,%.3f,%.3f,%.3f)",E,uc_fb(0),uc_fb(1),uc_fb(2),uc_fb(3));
    ROS_INFO("E:%.3f, uc:(%.3f,%.3f,%.3f,%.3f)",E,uc_fb(0)+_uc_nom(0),uc_fb(1)+_uc_nom(1),uc_fb(2)+_uc_nom(2),uc_fb(3)+_uc_nom(3));
    */

    // Update fz_dot
    fz_dot = _uc_nom(0)+uc_fb(0);

    // Update thrust
    fzCmd = fzCmd + fz_dot*dt;

    // Update desired euler_dot_rate
    for (int i = 0; i<3; i++){
      euler_dot(i) = std::max(std::min(_uc_nom(i+1)+uc_fb(i+1),1.5),-1.5);
    }

  } else {
    uc_fb.setZero();
    euler_dot.setZero();
    fz_dot = 0.0;
    fzCmd = g;
  }

  // Convert to desired omega (r_wb)
  R_om(euler,euler_dot);

  Eigen::Vector3d ew = mea_wb-r_wb;
  tauCmd = -KW*ew; // + mea_wb.cross(J*mea_wb) -J*mea_wb.cross(r_wb);

  // Get desired forces
  Eigen::Vector4d wrench(fzCmd*mass, tauCmd(0), -tauCmd(1), -tauCmd(2));
  Eigen::Vector4d ffff = A.colPivHouseholderQr().solve(wrench);

  // Mix
  motor_mix(wrench, ffff);
}

void CCMController::motor_mix(Eigen::Vector4d &wrench, Eigen::Vector4d &ffff){
  // Find worst violator
  bool sat = false;
  double worst_viol = 0.0;
  int worst_f = 0;
  double f_min = 0.1*mass*g/4.0;
  for (int i = 0; i<4; i++){
    if (ffff(i) < f_min || ffff(i) > TCOEFF) {
      sat = true;
      double low_pen = std::abs(std::min(ffff(i)-f_min,0.0));
      double high_pen = std::abs(std::max(ffff(i)-TCOEFF,0.0));
      if ( low_pen > worst_viol || high_pen > worst_viol ) {
        worst_viol = std::max(low_pen,high_pen);
        worst_f = i;
      }
    }
  }
  // Tune down tau_z, while maintaining thrust and tau_xy
  if (sat){
    ffff(worst_f) = ffff(worst_f) < f_min ? f_min : TCOEFF;
    Eigen::Matrix4d A_up = A;
    int i_up = 0;
    for (int i=0; i<4; i++){
      if (i!=worst_f){
        A_up.col(i_up) = A.col(i);
        i_up += 1;
      }
    }
    A_up.col(3) = Eigen::Vector4d(0.0,0.0,0.0,1.0);
    wrench(3) = 0.0;
    Eigen::Vector4d ffff_up = A_up.colPivHouseholderQr().solve(wrench-ffff(worst_f)*A.col(worst_f));
    i_up = 0;
    for (int i=0; i<4; i++){
      if (i!=worst_f){
        ffff(i) = ffff_up(i_up);
        i_up += 1;
      }
    }
  }

  const double c_a =  4.67636;
  const double c_b =  1.68915;
  const double c_c = -0.05628;

  for(int i=0; i<4; i++) {
      //motorCmd[i]  = (-c_b + sqrt( (c_b*c_b) - (4*c_a*(c_c - ffff(i))) ) )/(2*c_a);
      motorCmd[i] = ffff(i)/TCOEFF;

      if (motorCmd[i] < 0) {
      motorCmd[i] = 0;
      }
      else if (motorCmd[i] > 1.0) {
      motorCmd[i] = 1.0;
      }
      else {
      // Carry On
      }
  }

}
