/*!
 * \file robot_pose_publisher.cpp
 * \brief Publishes the robot's position in a geometry_msgs/Pose message.
 *
 * Publishes the robot's position in a geometry_msgs/Pose message based on the TF
 * difference between /map and /base_link.
 *
 * \author Russell Toris - rctoris@wpi.edu
 * \date April 3, 2014
 */

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>


/*!
 * Creates and runs the robot_pose_publisher node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
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

  nh_priv.param<std::string>("map_frame",map_frame,"/map");
  nh_priv.param<std::string>("base_frame",base_frame,"/base_link");
  nh_priv.param<double>("publish_frequency",publish_frequency,10);
  nh_priv.param<bool>("is_stamped", is_stamped, false);

  if(is_stamped)
    p_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 5);
  else
    p_pub = nh.advertise<geometry_msgs::Pose>("/mavros/vision_pose/pose", 5);

  // create the listener
  tf::TransformListener listener;
  listener.waitForTransform(map_frame, base_frame, ros::Time(), ros::Duration(1.0));

  // create value placeholders
  double pos_x, pos_y, pos_z = 0.0;
  double att_w, att_x, att_y, att_z = 0.0;
  att_w = 1.0;
  tf::StampedTransform transform;

  geometry_msgs::PoseStamped pose_stamped;

  // artifical noise gen
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0,0.01);
  double rand_x, rand_y, rand_z = 0.0;


  ros::Rate rate(publish_frequency);

  // continuously loop
  while (nh.ok())
  {
    try
    {
      listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);

      // if successful, store the values
      pos_x = transform.getOrigin().getX();
      pos_y = transform.getOrigin().getY();
      pos_z = transform.getOrigin().getZ();

      att_w = transform.getRotation().getW();
      att_x = transform.getRotation().getX();
      att_y = transform.getRotation().getY();
      att_z = transform.getRotation().getZ();

    }
    catch (tf::TransformException &ex)
    {
      // use last recorded values
      ROS_INFO("didn't see vicon tf!");

      rand_x = distribution(generator);
      rand_y = distribution(generator);
      rand_z = distribution(generator);

      pos_x += rand_x;
      pos_y += rand_y;
      pos_z += rand_z;

    }

    // ALWAYS PUBLISH SOMETHING!!

    pose_stamped.header.frame_id = map_frame;
    pose_stamped.header.stamp = ros::Time::now();

    pose_stamped.pose.orientation.x = att_x;
    pose_stamped.pose.orientation.y = att_y;
    pose_stamped.pose.orientation.z = att_z;
    pose_stamped.pose.orientation.w = att_w;

    pose_stamped.pose.position.x = pos_x;
    pose_stamped.pose.position.y = pos_y;
    pose_stamped.pose.position.z = pos_z;

    if(is_stamped)
      p_pub.publish(pose_stamped);
    else
      p_pub.publish(pose_stamped.pose);

    rate.sleep();

  }

  return EXIT_SUCCESS;
}
