#include <control/se3controller.h>
#include <Eigen/Dense>



SE3Controller::SE3Controller(void):
    MODEL("aslquad"), KP(4.0), KV(6.0), KR(0.3), KW(0.05),
    mass(1.04), g(9.8), TCOEFF(4.4), mea_pos(0,0,0), mea_vel(0,0,0),
    mea_wb(0,0,0), dt(1.0), fz(9.8066) {
  // handle ros parameters
  ros::param::get("~KP", KP);
  ros::param::get("~KV", KV);
  ros::param::get("~KR", KR);
  ros::param::get("~KW", KW);
  ros::param::get("~M", mass);
  ros::param::get("~TCOEFF", TCOEFF);
  ros::param::get("~MODEL", MODEL);

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

  mea_R = Eigen::Matrix3d::Identity();


  std::cout << "SE3 Using the following parameters: " << std::endl;
  std::cout << "KP = " << KP << std::endl;
  std::cout << "KV = " << KV << std::endl;
  std::cout << "KR = " << KR << std::endl;
  std::cout << "KW = " << KW << std::endl;
  std::cout << "M = " << mass << std::endl;
  std::cout << "TCOEFF = " << TCOEFF << std::endl;
  std::cout << "MODEL: " << MODEL << std::endl;

}

/********* State update ***********/
void SE3Controller::updateState(const Eigen::Vector3d &r, const Eigen::Matrix3d &R,
                                const Eigen::Vector3d &v, const Eigen::Vector3d &w,
                                const double _fz, const double _dt, const int pose_up, const int vel_up) {

  dt = _dt;
  if (pose_up == 0) {
    mea_pos += 0.5*(v+mea_vel)*dt;
  } else { mea_pos = r;}
  if (vel_up == 0) {
    Eigen::Vector3d accel = -(FzCmd/mass)*mea_R.col(2);
    accel(2) += g;
    mea_vel += accel*dt;
  } else {mea_vel = v; fz = _fz;}

  mea_R = R;
  mea_wb = w;
}

double SE3Controller::getfz(){
  return FzCmd/mass;
}

void SE3Controller::calcSE3(const double &yaw_des, const Eigen::Vector3d &r_pos, const Eigen::Vector3d &r_vel,
          const Eigen::Vector3d &r_acc, const Eigen::Vector3d &r_jer) {

  // Compute desired thrust
  Eigen::Vector3d e3(0.0,0.0,1.0);
  Eigen::Vector3d Fdes = -KP*(mea_pos-r_pos) - KV*(mea_vel-r_vel) - mass*g*e3 + mass*r_acc;
  FzCmd = -Fdes.dot(mea_R.col(2));

  // Compute desired attitude
  Eigen::Vector3d zb_des = -Fdes / FzCmd;
  Eigen::Vector3d yc_des(-std::sin(yaw_des), std::cos(yaw_des), 0.0); // only need yaw from reference euler
  Eigen::Vector3d xb = yc_des.cross(zb_des);
  Eigen::Vector3d xb_des = xb.normalized();
  Eigen::Vector3d yb_des = zb_des.cross(xb_des);

  Eigen::MatrixXd R_des(3,3);
  R_des.col(0) = xb_des;
  R_des.col(1) = yb_des;
  R_des.col(2) = zb_des;

  // Compute desired body Rate
  Eigen::Vector3d acc = g*e3 - fz*mea_R.col(2);
  Eigen::Vector3d Fdes_dot = -KP*(mea_vel-r_vel) - KV*(acc-r_acc) + mass*r_jer;
  Eigen::Vector3d zb_des_dot = (zb_des.dot(Fdes_dot)/FzCmd)*zb_des - (1.0/FzCmd)*Fdes_dot;
  Eigen::Vector3d om_skew_des = R_des.transpose()*zb_des_dot;
  double om_z_des = (om_skew_des(0)/R_des(1,1))*R_des(1,2);
  Eigen::Vector3d r_wb(-om_skew_des(1),om_skew_des(0),om_z_des);

  Eigen::MatrixXd temp(3,3);
  temp = R_des.transpose()*mea_R - mea_R.transpose()*R_des;
  Eigen::Vector3d eR(temp(2,1), temp(0,2), temp(1,0));
  eR = 0.5*eR;

  Eigen::Vector3d ew = mea_wb-mea_R.transpose()*R_des*r_wb;
  eR(2) /= 1.0; // Note: reduce gain on yaw
  ew(2) /= 1.0;
  tauCmd = -KR*eR - KW*ew + mea_wb.cross(J*mea_wb) - J*mea_wb.cross(mea_R.transpose()*R_des*r_wb);

  //std::cout << "fzCmd = " << fzCmd << std::endl;
  //std::cout << "tauCmd = " << tauCmd << std::endl;

  // Get desired forces
  Eigen::Vector4d wrench(FzCmd, tauCmd(0), -tauCmd(1), -tauCmd(2));
  Eigen::Vector4d ffff = A.colPivHouseholderQr().solve(wrench);

  // Mix
  motor_mix(wrench, ffff);
}

void SE3Controller::motor_mix(Eigen::Vector4d &wrench, Eigen::Vector4d &ffff){
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
