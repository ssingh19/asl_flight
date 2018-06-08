#include <control/se3controller.h>
#include <Eigen/Dense>



SE3Controller::SE3Controller(void):
    MODEL("aslquad"), KP(4.0), KV(6.0), KR(0.3), KW(0.05),
    M(1.04), g(9.8), TCOEFF(4.4), mea_pos(0,0,0), mea_vel(0,0,0),
    mea_wb(0,0,0), dt(1.0), lpf(1.0), fzCmd_prev(0.0) {
  // handle ros parameters
  ros::param::get("~KP", KP);
  ros::param::get("~KV", KV);
  ros::param::get("~KR", KR);
  ros::param::get("~KW", KW);
  ros::param::get("~M", M);
  ros::param::get("~TCOEFF", TCOEFF);
  ros::param::get("~MODEL", MODEL);

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

  mea_R = Eigen::Matrix3d::Identity();


  std::cout << "Using the following parameters: " << std::endl;
  std::cout << "KP = " << KP << std::endl;
  std::cout << "KV = " << KV << std::endl;
  std::cout << "KR = " << KR << std::endl;
  std::cout << "KW = " << KW << std::endl;
  std::cout << "M = " << M << std::endl;
  std::cout << "TCOEFF = " << TCOEFF << std::endl;
  std::cout << "MODEL: " << MODEL << std::endl;

  joySub = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &SE3Controller::joyCB, this);
}

void SE3Controller::updatePose(const Eigen::Vector3d &r, const Eigen::Matrix3d &R) {
  mea_pos = r;
  mea_R = R;
}

void SE3Controller::updateVel(const Eigen::Vector3d &v, const Eigen::Vector3d &w, const double &_dt) {
  vel_prev = mea_vel;
  mea_vel = v;
  dt = _dt;

  mea_wb = w;
}

void SE3Controller::joyCB(const sensor_msgs::Joy::ConstPtr& joy) {
  joyCmd[0] = 0.2*joy->axes[3]; // [-2,2], can go either up or down
  joyCmd[1] = -0.2*joy->axes[0];
  joyCmd[2] = 0.2*joy->axes[1];
  joyCmd[3] = 1.0*joy->axes[2];
}

void SE3Controller::joySE3(void) {
  KP = 0.0; // need to disable position control
  double yaw_des =  joyCmd[3];
  Eigen::Vector3d r_wb(0, 0, 0);
  Eigen::Vector3d r_pos(0, 0, 0);
  Eigen::Vector3d r_vel(joyCmd[2], -joyCmd[1], joyCmd[0]);
  Eigen::Vector3d r_acc(0, 0, 0);
  Eigen::Vector3d r_jer(0, 0, 0);
  calcSE3(yaw_des, r_pos, r_vel, r_acc, r_jer);
}

void SE3Controller::calcSE3(const double &yaw_des, const Eigen::Vector3d &r_pos, const Eigen::Vector3d &r_vel,
          const Eigen::Vector3d &r_acc, const Eigen::Vector3d &r_jer) {

  // Compute desired thrust
  Eigen::Vector3d e3(0.0,0.0,1.0);
  Eigen::Vector3d Fdes = -KP*(mea_pos-r_pos) - KV*(mea_vel-r_vel) - M*g*e3 + M*r_acc;
  fzCmd = -Fdes.dot(mea_R.col(2));

  // Compute desired attitude
  Eigen::Vector3d zb_des = -Fdes / fzCmd;
  Eigen::Vector3d yc_des(-std::sin(yaw_des), std::cos(yaw_des), 0.0); // only need yaw from reference euler
  Eigen::Vector3d xb = yc_des.cross(zb_des);
  Eigen::Vector3d xb_des = xb.normalized();
  Eigen::Vector3d yb_des = zb_des.cross(xb_des);

  Eigen::MatrixXd R_des(3,3);
  R_des.col(0) = xb_des;
  R_des.col(1) = yb_des;
  R_des.col(2) = zb_des;

  // Compute desired body Rate
  Eigen::Vector3d acc = g*e3 - (fzCmd/M)*mea_R.col(2);
  fzCmd_prev = fzCmd;

  if (dt >= 0.001){
    acc = lpf*acc + (1.0-lpf)*(mea_vel - vel_prev)/dt;
  }
  Eigen::Vector3d Fdes_dot = -KP*(mea_vel-r_vel) - KV*(acc-r_acc) + M*r_jer;
  Eigen::Vector3d zb_des_dot = (zb_des.dot(Fdes_dot)/fzCmd)*zb_des - (1.0/fzCmd)*Fdes_dot;
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

  Eigen::Vector4d wrench(fzCmd, tauCmd(0), -tauCmd(1), -tauCmd(2));
  Eigen::Vector4d ffff = W.colPivHouseholderQr().solve(wrench);

  //std::cout << ffff << std::endl;
  for(int i=0; i<4; i++) {
    motorCmd[i] = ffff(i)/TCOEFF;
  }

}
