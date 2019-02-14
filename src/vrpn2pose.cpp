/*!
 * \file robot_pose_publisher.cpp
 * \brief Publishes the robot's position in a geometry_msgs/Pose message.
 *
 * Publishes the robot's position in a geometry_msgs/Pose message based on the
 VRPN published pose message
 *
 * \author Sumeet Singh
 * \date Feb 12, 2019
 */

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>


/*!
 * Creates and runs the robot_pose_publisher node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */


 double pos_x, pos_y, pos_z = 0.0;
 double att_w, att_x, att_y, att_z = 0.0;
 bool pose_up;

 void poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
   // By default, MAVROS gives local_position in ENU
   pos_x = msg->pose.position.x;
   pos_y = -msg->pose.position.z;
   pos_z = msg->pose.position.y;

   // ROS quaternion
   att_x = msg->pose.orientation.x;
   att_y = -msg->pose.orientation.z;
   att_z = msg->pose.orientation.y;
   att_w = msg->pose.orientation.w;

   pose_up = true;

 }

int main(int argc, char ** argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "robot_pose_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // configuring parameters
  std::string map_frame, base_frame;
  double publish_frequency;
  bool is_stamped;
  ros::Publisher p_pub;

  nh_priv.param<std::string>("vrpn_frame",base_frame,"base_frame");
  nh_priv.param<double>("publish_frequency",publish_frequency,10);
  nh_priv.param<bool>("is_stamped", is_stamped, false);

  if(is_stamped)
    p_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 5);
  else
    p_pub = nh.advertise<geometry_msgs::Pose>("/mavros/vision_pose/pose", 5);


  ros::Subscriber poseSub = nh.subscribe<geometry_msgs::PoseStamped>(base_frame, 2, poseSubCB);

    // create value placeholders

  att_w = 1.0;

  geometry_msgs::PoseStamped pose_msg;

  ros::Rate rate(publish_frequency);

  pose_up = false;

  // continuously loop
  while (nh.ok())
  {


    while (!pose_up)
    {
      ros::spinOnce();
    }

    // ALWAYS PUBLISH SOMETHING!!

    // pose_msg.header.frame_id = map_frame;
    pose_msg.header.stamp = ros::Time::now();

    pose_msg.pose.orientation.x = att_x;
    pose_msg.pose.orientation.y = att_y;
    pose_msg.pose.orientation.z = att_z;
    pose_msg.pose.orientation.w = att_w;

    pose_msg.pose.position.x = pos_x;
    pose_msg.pose.position.y = pos_y;
    pose_msg.pose.position.z = pos_z;

    if(is_stamped)
      p_pub.publish(pose_msg);
    else
      p_pub.publish(pose_msg.pose);

    pose_up = false;

    rate.sleep();


  }

  return EXIT_SUCCESS;
}
