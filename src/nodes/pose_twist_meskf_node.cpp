/**
 * @file
 * @brief ROS pose-twist multiplicative error state Kalman filter (node version).
 */


#include <ros/ros.h>
#include "pose_twist_meskf_node_base.h"

int main(int argc, char **argv)
{
  // ROS initialization.
  ros::init(argc, argv, "pose_twist_meskf_node");

  ros::NodeHandle node;
  ros::NodeHandle priv("~");

  pose_twist_meskf::PoseTwistMESKFNodeBase pose_twist_meskf_node(node,priv);

  // Subscription is handled at start and stop service callbacks.
  ros::spin();

  return 0;
}
