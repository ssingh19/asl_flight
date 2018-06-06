#include <se3controller.h>
#include <Eigen/Dense>


SE3Controller::SE3Controller(void):
    MODEL("aslquad"), KP(4.0), KV(6.0), KR(0.3), KW(0.05),
    M(1.04), g(9.8), TCOEFF(4.4) {
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
  } else if(MODEL=="iris") {
    // Iris in Gazebo (ENU torque command)
    W << 1, 1, 1, 1,
        -0.22, 0.2, 0.22, -0.2,
        -0.13, 0.13, -0.13, 0.13,
        -0.06, -0.06, 0.06, 0.06;
  } else {
    ROS_ERROR("Unknown model name!");
  }


  std::cout << "Using the following parameters: " << std::endl;
  std::cout << "KP = " << KP << std::endl;
  std::cout << "KV = " << KV << std::endl;
  std::cout << "KR = " << KR << std::endl;
  std::cout << "KW = " << KW << std::endl;
  std::cout << "M = " << M << std::endl;
  std::cout << "TCOEFF = " << TCOEFF << std::endl;
  std::cout << "MODEL: " << MODEL << std::endl;

  poseSub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &SE3Controller::poseSubCB, this);
  velSub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 1, &SE3Controller::velSubCB, this);
  actuatorPub = nh.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 1);

  joySub = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &SE3Controller::joyCB, this);
}

void SE3Controller::poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // By default, MAVROS gives local_position/pose in ENU
  mea_pos(0) = msg->pose.position.x;
  mea_pos(1) = -msg->pose.position.y;
  mea_pos(2) = -msg->pose.position.z;

  double q1 = msg->pose.orientation.w;
  double q2 = msg->pose.orientation.x;
  double q3 = -msg->pose.orientation.y;
  double q4 = -msg->pose.orientation.z;
  double q1s = q1*q1;
  double q2s = q2*q2;
  double q3s = q3*q3;
  double q4s = q4*q4;

  mea_R <<
    q1s+q2s-q3s-q4s, 2.0*(q2*q3-q1*q4) ,2.0*(q2*q4+q1*q3),
    2.0*(q2*q3+q1*q4), q1s-q2s+q3s-q4s, 2.0*(q3*q4-q1*q2),
    2.0*(q2*q4-q1*q3), 2.0*(q3*q4+q1*q2), q1s-q2s-q3s+q4s;
}

void SE3Controller::velSubCB(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  // By default, MAVROS gives local_position/velocity in ENU
  mea_vel(0) = msg->twist.linear.x;
  mea_vel(1) = -msg->twist.linear.y;
  mea_vel(2) = -msg->twist.linear.z;
  mea_wb(0) = msg->twist.angular.x;
  mea_wb(1) = -msg->twist.angular.y;
  mea_wb(2) = -msg->twist.angular.z;
}

void SE3Controller::joyCB(const sensor_msgs::Joy::ConstPtr& joy) {
  joyCmd[0] = 0.2*joy->axes[3]; // [-2,2], can go either up or down
  joyCmd[1] = -0.2*joy->axes[0];
  joyCmd[2] = 0.2*joy->axes[1];
  joyCmd[3] = 1.0*joy->axes[2];
}

void SE3Controller::joySE3(void) {
  KP = 0.0; // need to disable position control
  Eigen::Vector3d r_euler(joyCmd[1], joyCmd[2], joyCmd[3]);
  Eigen::Vector3d r_wb(0, 0, 0);
  Eigen::Vector3d r_pos(0, 0, 0);
  Eigen::Vector3d r_vel(joyCmd[2], -joyCmd[1], joyCmd[0]);
  Eigen::Vector3d r_acc(0, 0, 0);
  calcSE3(r_euler, r_wb, r_pos, r_vel, r_acc);
}

void SE3Controller::calcSE3(const Eigen::Vector3d &r_euler, const Eigen::Vector3d &r_wb,
          const Eigen::Vector3d &r_pos, const Eigen::Vector3d &r_vel,
          const Eigen::Vector3d &r_acc) {
  // Note: this code is modified to directly compute in ENU, since MAVROS gives ENU by default
  Eigen::Vector3d zw(0.0,0.0,-1.0);
  Eigen::Vector3d Fdes = -KP*(mea_pos-r_pos) - KV*(mea_vel-r_vel) + M*g*zw + M*r_acc;
  fzCmd = -Fdes.dot(mea_R.col(2));

  Eigen::Vector3d zb_des = -Fdes / Fdes.norm();
  Eigen::Vector3d yc_des(-std::sin(r_euler(2)), std::cos(r_euler(2)), 0.0); // only need yaw from reference euler
  Eigen::Vector3d xb = yc_des.cross(zb_des);
  Eigen::Vector3d xb_des = xb.normalized();
  Eigen::Vector3d yb_des = zb_des.cross(xb_des);

  Eigen::MatrixXd R_des(3,3);
  R_des.col(0) = xb_des;
  R_des.col(1) = yb_des;
  R_des.col(2) = zb_des;

  Eigen::MatrixXd temp(3,3);
  temp = R_des.transpose()*mea_R - mea_R.transpose()*R_des;
  Eigen::Vector3d eR(temp(2,1), temp(0,2), temp(1,0));
  eR = 0.5*eR;
  Eigen::Vector3d ew = mea_wb-r_wb;
  eR(2) /= 3.0; // Note: reduce gain on yaw
  ew(2) /= 3.0;
  tauCmd = -KR*eR - KW*ew;

  //std::cout << "fzCmd = " << fzCmd << std::endl;
  //std::cout << "tauCmd = " << tauCmd << std::endl;

  Eigen::Vector4d wrench(fzCmd, tauCmd(0), -tauCmd(1), -tauCmd(2));
  Eigen::Vector4d ffff = W.colPivHouseholderQr().solve(wrench);

  //std::cout << ffff << std::endl;
  for(int i=0; i<4; i++) {
    motorCmd[i] = ffff(i)/TCOEFF;
  }

  // publish commands
  mavros_msgs::ActuatorControl cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.group_mix = 0;
  for(int i=0; i<4; i++) {
    cmd.controls[i] = motorCmd[i];
  }
  cmd.controls[7] = 0.1234; // secret key to enabling direct motor control in px4
  actuatorPub.publish(cmd);
}
