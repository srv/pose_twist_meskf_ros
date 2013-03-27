/**
 * @file
 * @brief ROS pose-twist multiplicative error state Kalman filter (nodelet version).
 */


#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "pose_twist_meskf_node_base.h"

namespace pose_twist_meskf
{

/**
 * @brief Nodelet interface for pose-twist multiplicative error state Kalman filtering.
 */
class PoseTwistMESKFNodelet: public nodelet::Nodelet
{
public:
  PoseTwistMESKFNodelet();
private:
  virtual void onInit();
  boost::shared_ptr<PoseTwistMESKFNodeBase> pose_twist_meskf_node_;
};

} // namespace

/**
 * @brief Default constructor (doing nothing).
 * @return
 */
pose_twist_meskf::PoseTwistMESKFNodelet::PoseTwistMESKFNodelet()
{}

/** Nodelet initialization.
 *  @note Must return immediately.
 */
void pose_twist_meskf::PoseTwistMESKFNodelet::onInit()
{
  ros::NodeHandle node(getNodeHandle());
  ros::NodeHandle priv(getPrivateNodeHandle());

  pose_twist_meskf_node_.reset(new PoseTwistMESKFNodeBase(node, priv));
}


// Register this plugin with pluginlib.
// Names must match *nodelet.xml in package root.
// Parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(pose_twist_meskf, pose_twist_meskf_nodelet, pose_twist_meskf::PoseTwistMESKFNodelet, nodelet::Nodelet)
