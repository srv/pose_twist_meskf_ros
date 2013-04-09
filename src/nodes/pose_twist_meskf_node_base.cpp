/**
 * @file
 * @brief Pose twist error state extended Kalman filter
 * with multiplicative orientation error node class (implementation).
 */

#include "pose_twist_meskf_node_base.h"
#include "nominal_state_vector.h"
#include "input_vector.h"
#include "visual_measurement_vector.h"
#include "depth_measurement_vector.h"
#include "error_state_vector.h"
#include "visual_measurement_error_vector.h"
#include "depth_measurement_error_vector.h"

/**
 * @brief Default constructor.
 * @param node node handle to node's namespace.
 * @param priv node handle to node's private namespace.
 * @return
 */
pose_twist_meskf::PoseTwistMESKFNodeBase::
PoseTwistMESKFNodeBase(const ros::NodeHandle& node,
                       const ros::NodeHandle& priv)
: node_(node), priv_(priv), filter_initialized_(false)
{ 
  // Node parameters.
  initializeParameters();

  // Subscribe to topics
  subscribeTopics();

  // Advertise output topics.
  advertiseTopics();

  // Initialize the update timer
  if (update_timer_.isValid() && update_timer_.hasPending())
    ROS_INFO_STREAM("Filter reset!");
  else
  {
    if (update_timer_.isValid())
    {
      update_timer_.start();
    }
    else
    {
      update_timer_ = node_.createTimer(ros::Duration(update_rate_),
                                        boost::bind(&PoseTwistMESKFNodeBase::updateCallback, this));
      ROS_INFO_STREAM("Filter started!");
    }
  }
}

/**
 * @brief Initialize the meskf filter.
 */
void pose_twist_meskf::PoseTwistMESKFNodeBase::initializeMeskf()
{
  // Initialize the nominal stat
  PoseTwistMESKF::Vector state(NominalStateVector::DIMENSION);
  state(NominalStateVector::POSITION_X) = last_visual_msg_->pose.pose.position.x;
  state(NominalStateVector::POSITION_Y) = last_visual_msg_->pose.pose.position.y;
  state(NominalStateVector::POSITION_Z) = last_depth_msg_->depth;
  state(NominalStateVector::ORIENTATION_W) = last_visual_msg_->pose.pose.orientation.w;
  state(NominalStateVector::ORIENTATION_X) = last_visual_msg_->pose.pose.orientation.x;
  state(NominalStateVector::ORIENTATION_Y) = last_visual_msg_->pose.pose.orientation.y;
  state(NominalStateVector::ORIENTATION_Z) = last_visual_msg_->pose.pose.orientation.z;
  state(NominalStateVector::LIN_VEL_X) = last_visual_msg_->twist.twist.linear.x;
  state(NominalStateVector::LIN_VEL_Y) = last_visual_msg_->twist.twist.linear.y;
  state(NominalStateVector::LIN_VEL_Z) = last_visual_msg_->twist.twist.linear.z;
  state(NominalStateVector::ANG_VEL_X) = last_visual_msg_->twist.twist.angular.x;
  state(NominalStateVector::ANG_VEL_Y) = last_visual_msg_->twist.twist.angular.y;
  state(NominalStateVector::ANG_VEL_Z) = last_visual_msg_->twist.twist.angular.z;
  state(NominalStateVector::LIN_ACC_X) = 0.0;
  state(NominalStateVector::LIN_ACC_Y) = 0.0;
  state(NominalStateVector::LIN_ACC_Z) = 0.0;
  state(NominalStateVector::GYRO_DRIFT_X) = 0.0;
  state(NominalStateVector::GYRO_DRIFT_Y) = 0.0;
  state(NominalStateVector::GYRO_DRIFT_Z) = 0.0;
  state(NominalStateVector::ACC_BIAS_X) = 0.0;
  state(NominalStateVector::ACC_BIAS_Y) = 0.0;
  state(NominalStateVector::ACC_BIAS_Z) = 0.0;

  // Initialize the error state vector covariance
  PoseTwistMESKF::SymmetricMatrix covariance(ErrorStateVector::DIMENSION);
  if (use_topic_cov_)
  {
    covariance = 0.0;
    for (int i=0; i<3; i++)
    {
      for (int j=0; j<3; j++)
      {
        covariance(ErrorStateVector::D_POSITION_X + i,
                   ErrorStateVector::D_POSITION_X + j) = last_visual_msg_->pose.covariance[6*i + j];
        covariance(ErrorStateVector::D_POSITION_X + i,
                   ErrorStateVector::D_ORIENTATION_X + j) = last_visual_msg_->pose.covariance[6*i + j + 3];
        covariance(ErrorStateVector::D_ORIENTATION_X + i,
                   ErrorStateVector::D_POSITION_X + j) = last_visual_msg_->pose.covariance[6*i + j + 18];
        covariance(ErrorStateVector::D_ORIENTATION_X + i,
                   ErrorStateVector::D_ORIENTATION_X + j) = last_visual_msg_->pose.covariance[6*i + j + 21];
        covariance(ErrorStateVector::D_LIN_VEL_X + i,
                   ErrorStateVector::D_LIN_VEL_X + j) = last_visual_msg_->twist.covariance[6*i + j];
        covariance(ErrorStateVector::D_GYRO_DRIFT_X + i,
                   ErrorStateVector::D_GYRO_DRIFT_X + j) = 0.0;
        covariance(ErrorStateVector::D_GYRO_DRIFT_X + i,
                   ErrorStateVector::D_ACC_BIAS_X + j) = 0.0;
        covariance(ErrorStateVector::D_ACC_BIAS_X + i,
                   ErrorStateVector::D_GYRO_DRIFT_X + j) = 0.0;
        covariance(ErrorStateVector::D_ACC_BIAS_X + i,
                   ErrorStateVector::D_ACC_BIAS_X + j) = 0.0;
      }
    }
  }
  else
  {
    covariance = COV_ERROR_STATE_;
  }

  double timestamp = last_visual_msg_->header.stamp.toSec();

  filter_.setUpSystem(VAR_ACC_,VAR_GYRO_,VAR_ACC_BIAS_,VAR_GYRO_DRIFT_,G_VEC_);

  filter_.setUpMeasurementModels();

  filter_.initialize(state, covariance, timestamp);

  filter_initialized_ = true;
}

/**
 * @brief Initialize parameters from parameter server or default values.
 */
void pose_twist_meskf::PoseTwistMESKFNodeBase::initializeParameters()
{
  priv_.param<std::string>("frame_id", frame_id_, "map");
  priv_.param<std::string>("child_frame_id", child_frame_id_, "base_link");
  priv_.param<std::string>("visual_odometry_frame_id", visual_odom_frame_id_,
                           "visual_odom");
  priv_.param("update_rate", update_rate_, 0.1);
  priv_.param("use_topic_cov", use_topic_cov_, false);
  priv_.param("gravity_x", G_VEC_(0), 0.0);
  priv_.param("gravity_y", G_VEC_(1), 0.0);
  priv_.param("gravity_z", G_VEC_(2), 9.80665);
  priv_.param("var_acc", VAR_ACC_, 4e-5);
  priv_.param("var_acc_bias", VAR_ACC_BIAS_, 1e-10);
  priv_.param("var_gyro", VAR_GYRO_, 1e-6);
  priv_.param("var_gyro_drift", VAR_GYRO_DRIFT_, 1e-10);

  // Read user-defined covariances
  if (!use_topic_cov_)
  {
    // Error state covariance
    XmlRpc::XmlRpcValue cov_params;
    priv_.getParam("cov_error_state", cov_params);
    ROS_ASSERT(cov_params.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(cov_params.size() == 6);
    for (int i=0; i<cov_params.size(); i++)
      ROS_ASSERT(cov_params[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    PoseTwistMESKF::SymmetricMatrix cov_error_state(ErrorStateVector::DIMENSION);
    cov_error_state = 0.0;
    for (int i=0; i<3; i++)
    {
      cov_error_state(ErrorStateVector::D_POSITION_X + i,
         ErrorStateVector::D_POSITION_X + i) = cov_params[0];
      cov_error_state(ErrorStateVector::D_LIN_VEL_X + i,
         ErrorStateVector::D_LIN_VEL_X + i) = cov_params[1];
      cov_error_state(ErrorStateVector::D_ACC_BIAS_X + i,
         ErrorStateVector::D_ACC_BIAS_X + i) = cov_params[2];
      cov_error_state(ErrorStateVector::D_ORIENTATION_X + i,
         ErrorStateVector::D_ORIENTATION_X + i) = cov_params[3];
      cov_error_state(ErrorStateVector::D_GYRO_DRIFT_X + i,
         ErrorStateVector::D_GYRO_DRIFT_X + i) = cov_params[4];
    }
    cov_error_state(3,3) = cov_params[5];
    COV_ERROR_STATE_ = cov_error_state;

    // Visual odometry covariance
    priv_.getParam("cov_visual_odom", cov_params);
    ROS_ASSERT(cov_params.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(cov_params.size() == 6);
    for (int i=0; i<cov_params.size(); i++)
      ROS_ASSERT(cov_params[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    PoseTwistMESKF::SymmetricMatrix cov_visual_odom(VisualMeasurementErrorVector::DIMENSION);
    cov_visual_odom = 0.0;
    for (int i=0; i<3; i++)
    {
      cov_visual_odom(VisualMeasurementErrorVector::D_POSITION_X + i,
        VisualMeasurementErrorVector::D_POSITION_X + i) = cov_params[0];
      cov_visual_odom(VisualMeasurementErrorVector::D_ORIENTATION_X + i,
        VisualMeasurementErrorVector::D_ORIENTATION_X + i) = cov_params[1];
      cov_visual_odom(VisualMeasurementErrorVector::D_LIN_VEL_X + i,
        VisualMeasurementErrorVector::D_LIN_VEL_X + i) = cov_params[2];
      cov_visual_odom(VisualMeasurementErrorVector::D_ACC_BIAS_X + i,
        VisualMeasurementErrorVector::D_ACC_BIAS_X + i) = cov_params[3];
      cov_visual_odom(VisualMeasurementErrorVector::D_GYRO_DRIFT_X + i,
        VisualMeasurementErrorVector::D_GYRO_DRIFT_X + i) = cov_params[4];
    }
    cov_visual_odom(3,3) = cov_params[5];
    COV_VISUAL_ODOM_ = cov_visual_odom;

    // Visual odometry covariance when failure
    priv_.getParam("cov_visual_odom_failure", cov_params);
    ROS_ASSERT(cov_params.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(cov_params.size() == 6);
    for (int i=0; i<cov_params.size(); i++)
      ROS_ASSERT(cov_params[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    PoseTwistMESKF::SymmetricMatrix cov_visual_odom_failure(VisualMeasurementErrorVector::DIMENSION);
    cov_visual_odom_failure = 0.0;
    for (int i=0; i<3; i++)
    {
      cov_visual_odom_failure(VisualMeasurementErrorVector::D_POSITION_X + i,
        VisualMeasurementErrorVector::D_POSITION_X + i) = cov_params[0];
      cov_visual_odom_failure(VisualMeasurementErrorVector::D_ORIENTATION_X + i,
        VisualMeasurementErrorVector::D_ORIENTATION_X + i) = cov_params[1];
      cov_visual_odom_failure(VisualMeasurementErrorVector::D_LIN_VEL_X + i,
        VisualMeasurementErrorVector::D_LIN_VEL_X + i) = cov_params[2];
      cov_visual_odom_failure(VisualMeasurementErrorVector::D_ACC_BIAS_X + i,
        VisualMeasurementErrorVector::D_ACC_BIAS_X + i) = cov_params[3];
      cov_visual_odom_failure(VisualMeasurementErrorVector::D_GYRO_DRIFT_X + i,
        VisualMeasurementErrorVector::D_GYRO_DRIFT_X + i) = cov_params[4];
    }
    cov_visual_odom_failure(3,3) = cov_params[5];
    COV_VISUAL_ODOM_FAILURE_ = cov_visual_odom_failure;

    // Depth covariance
    double cov_depth_val;
    priv_.param("cov_depth", cov_depth_val, 1.0e-12);
    PoseTwistMESKF::SymmetricMatrix cov_depth(DepthMeasurementErrorVector::DIMENSION);
    cov_depth(DepthMeasurementErrorVector::D_DEPTH_Z, 
      DepthMeasurementErrorVector::D_DEPTH_Z) = cov_depth_val;
    COV_DEPTH_ = cov_depth;
  }

  // Info
  ROS_INFO_STREAM("Reference frame: " << frame_id_ << ".");
  ROS_INFO_STREAM("Body-fixed frame: " << child_frame_id_ << ".");
  ROS_INFO_STREAM("Visual odometry frame: " << visual_odom_frame_id_ << ".");
  ROS_INFO_STREAM("Update rate: " << update_rate_ << " Hz.");
  ROS_INFO_STREAM("Use topic covariances: " << use_topic_cov_ << ".");
}

/**
 * @brief Advertise combined odometry and IMU bias topics.
 */
void pose_twist_meskf::PoseTwistMESKFNodeBase::advertiseTopics()
{
  publ_odom_ = priv_.advertise<nav_msgs::Odometry>("odometry", 10);
  publ_odom_pre_ = priv_.advertise<nav_msgs::Odometry>("odometry_pre", 10);
  publ_gyro_drift_ = priv_.advertise<geometry_msgs::Vector3Stamped>("gyroscope_bias", 10);
  publ_accel_bias_ = priv_.advertise<geometry_msgs::Vector3Stamped>("accelerometer_bias", 10);
}

/**
 * @brief Subscribe to IMU, visual odometry and depth topics.
 */
void pose_twist_meskf::PoseTwistMESKFNodeBase::subscribeTopics()
{
  subs_imu_ = node_.subscribe("imu", 10,
                              &PoseTwistMESKFNodeBase::IMUCallback,
                              this);
  subs_visual_odom_ = node_.subscribe("visual_odometry", 10,
                                      &PoseTwistMESKFNodeBase::visualCallback,
                                      this);
  subs_depth_ = node_.subscribe("depth", 10,
                                &PoseTwistMESKFNodeBase::depthCallback,
                                this);
}

/**
 * @brief IMU reading callback.
 * @param msg new IMU reading to be enqueued.
 */
void pose_twist_meskf::PoseTwistMESKFNodeBase::
IMUCallback(const sensor_msgs::ImuConstPtr& msg)
{
  if (!filter_initialized_) return;

  PoseTwistMESKF::Vector input(6);

  // Set the input
  input(InputVector::LIN_ACC_X) = msg->linear_acceleration.x;
  input(InputVector::LIN_ACC_Y) = msg->linear_acceleration.y;
  input(InputVector::LIN_ACC_Z) = msg->linear_acceleration.z;
  input(InputVector::ANG_VEL_X) = msg->angular_velocity.x;
  input(InputVector::ANG_VEL_Y) = msg->angular_velocity.y;
  input(InputVector::ANG_VEL_Z) = msg->angular_velocity.z;

  filter_.addInput(msg->header.stamp.toSec(), input);
}


/**
 * @brief Visual measurement callback.
 * @param msg new visual measurement to be enqueued.
 */
void
pose_twist_meskf::PoseTwistMESKFNodeBase::
visualCallback(const nav_msgs::OdometryConstPtr& msg)
{
  if (!filter_initialized_)
  {
    last_visual_msg_.reset(new nav_msgs::Odometry(*msg));
    return;
  }

  double timestamp = msg->header.stamp.toSec();
  PoseTwistMESKF::Vector measurement(VisualMeasurementVector::DIMENSION);
  measurement(VisualMeasurementVector::POSITION_X) = msg->pose.pose.position.x;
  measurement(VisualMeasurementVector::POSITION_Y) = msg->pose.pose.position.y;
  measurement(VisualMeasurementVector::POSITION_Z) = msg->pose.pose.position.z;
  measurement(VisualMeasurementVector::ORIENTATION_W) = msg->pose.pose.orientation.w;
  measurement(VisualMeasurementVector::ORIENTATION_X) = msg->pose.pose.orientation.x;
  measurement(VisualMeasurementVector::ORIENTATION_Y) = msg->pose.pose.orientation.y;
  measurement(VisualMeasurementVector::ORIENTATION_Z) = msg->pose.pose.orientation.z;
  measurement(VisualMeasurementVector::ANG_VEL_X) = msg->twist.twist.angular.x;
  measurement(VisualMeasurementVector::ANG_VEL_Y) = msg->twist.twist.angular.y;
  measurement(VisualMeasurementVector::ANG_VEL_Z) = msg->twist.twist.angular.z;
  measurement(VisualMeasurementVector::LIN_VEL_X) = msg->twist.twist.linear.x;
  measurement(VisualMeasurementVector::LIN_VEL_Y) = msg->twist.twist.linear.y;
  measurement(VisualMeasurementVector::LIN_VEL_Z) = msg->twist.twist.linear.z;

  PoseTwistMESKF::SymmetricMatrix covariance(VisualMeasurementErrorVector::DIMENSION);
  if (use_topic_cov_)
  {
    for (int i=0; i<3; i++)
    {
      for (int j=0; j<3; j++)
      {
        covariance(VisualMeasurementErrorVector::D_LIN_VEL_X + i,
                   VisualMeasurementErrorVector::D_LIN_VEL_X + j) = msg->twist.covariance[6*i + j];
        covariance(VisualMeasurementErrorVector::D_LIN_VEL_X + i,
                   VisualMeasurementErrorVector::D_GYRO_DRIFT_X + j) = msg->twist.covariance[6*i + j + 3];
        covariance(VisualMeasurementErrorVector::D_GYRO_DRIFT_X + i,
                   VisualMeasurementErrorVector::D_LIN_VEL_X + j) = msg->twist.covariance[6*i + j + 18];
        covariance(VisualMeasurementErrorVector::D_GYRO_DRIFT_X + i,
                   VisualMeasurementErrorVector::D_GYRO_DRIFT_X + j) = msg->twist.covariance[6*i + j + 21];
      }
    }
  }
  else
  {
    // Odometry failure?
    if (msg->pose.pose.position.x == 0.0 && msg->pose.pose.position.y == 0.0
      && msg->pose.pose.position.z == 0.0)
    {
      covariance = COV_VISUAL_ODOM_FAILURE_;
    }
    else
    {
      covariance = COV_VISUAL_ODOM_;
    }
  }
  
  filter_.addMeasurement(filter_.VISUAL, measurement, covariance, timestamp);
}

/**
 * @brief Depth measurement callback.
 * @param msg new depth measurement to be enqueued.
 */
void
pose_twist_meskf::PoseTwistMESKFNodeBase::
depthCallback(const auv_sensor_msgs::Depth& msg)
{
  if (!filter_initialized_)
  {
    last_depth_msg_.reset(new auv_sensor_msgs::Depth(msg));
    return;
  }

  double timestamp = msg.header.stamp.toSec();

  PoseTwistMESKF::Vector measurement(DepthMeasurementVector::DIMENSION);
  measurement(DepthMeasurementVector::DEPTH_Z) = msg.depth;

  PoseTwistMESKF::SymmetricMatrix covariance(DepthMeasurementErrorVector::DIMENSION);

  // AUV_sensor_msgs::Depth has no covariance field

  filter_.addMeasurement(filter_.DEPTH, measurement, COV_DEPTH_, timestamp);
}


/**
 * @brief Update callback.
 *
 * Filter is updated processing inputs and measurements in all queues until
 * is processed. Final estimates are published on respective topics.
 */
void pose_twist_meskf::PoseTwistMESKFNodeBase::updateCallback()
{
  // Check if filter is initialized
  if (!filter_initialized_)
  {
    if (last_visual_msg_ && last_depth_msg_)
    {
      // Filter needs one visual odometry message 
      // and one depth message to be initialized
      initializeMeskf();
    }
    return;
  }

  // Update
  bool success = filter_.updateAll();

  if (!success)
  {
    ROS_ERROR_STREAM("Could not update filter.");
    return;
  }

  // Get results
  const double timestamp = filter_.getFilterTime();
  const PoseTwistMESKF::Vector state = filter_.getEstimate();
  const PoseTwistMESKF::SymmetricMatrix covariance = filter_.getCovariance();

  // Publish messages
  nav_msgs::Odometry odometry_msg;
  odometry_msg.header.stamp.fromSec(timestamp);
  odometry_msg.header.frame_id = frame_id_;
  odometry_msg.child_frame_id = child_frame_id_;
  odometry_msg.pose.pose.position.x = state(NominalStateVector::POSITION_X);
  odometry_msg.pose.pose.position.y = state(NominalStateVector::POSITION_Y);
  odometry_msg.pose.pose.position.z = state(NominalStateVector::POSITION_Z);
  odometry_msg.pose.pose.orientation.w = state(NominalStateVector::ORIENTATION_W);
  odometry_msg.pose.pose.orientation.x = state(NominalStateVector::ORIENTATION_X);
  odometry_msg.pose.pose.orientation.y = state(NominalStateVector::ORIENTATION_Y);
  odometry_msg.pose.pose.orientation.z = state(NominalStateVector::ORIENTATION_Z);
  odometry_msg.twist.twist.linear.x = state(NominalStateVector::LIN_VEL_X);
  odometry_msg.twist.twist.linear.y = state(NominalStateVector::LIN_VEL_Y);
  odometry_msg.twist.twist.linear.z = state(NominalStateVector::LIN_VEL_Z);
  odometry_msg.twist.twist.angular.x = state(NominalStateVector::ANG_VEL_X);
  odometry_msg.twist.twist.angular.y = state(NominalStateVector::ANG_VEL_Y);
  odometry_msg.twist.twist.angular.z = state(NominalStateVector::ANG_VEL_Z);
  for (int i=0; i<3; i++)
  {
    for (int j=0; j<3; j++)
    {
      odometry_msg.pose.covariance[6*i + j] =
          covariance(ErrorStateVector::D_POSITION_X + i, ErrorStateVector::D_POSITION_X + j);
      odometry_msg.pose.covariance[6*i + j + 3] =
          covariance(ErrorStateVector::D_POSITION_X + i, ErrorStateVector::D_ORIENTATION_X + j);
      odometry_msg.pose.covariance[6*i + j + 18] =
          covariance(ErrorStateVector::D_ORIENTATION_X + i, ErrorStateVector::D_POSITION_X + j);
      odometry_msg.pose.covariance[6*i + j + 21] =
          covariance(ErrorStateVector::D_ORIENTATION_X + i, ErrorStateVector::D_ORIENTATION_X + j);
      odometry_msg.twist.covariance[6*i + j] =
          covariance(ErrorStateVector::D_LIN_VEL_X + i, ErrorStateVector::D_LIN_VEL_X + j);
      odometry_msg.twist.covariance[6*i + j + 3] =
          covariance(ErrorStateVector::D_LIN_VEL_X + i, ErrorStateVector::D_GYRO_DRIFT_X + j);
      odometry_msg.twist.covariance[6*i + j + 18] =
          covariance(ErrorStateVector::D_GYRO_DRIFT_X + i, ErrorStateVector::D_LIN_VEL_X + j);
      odometry_msg.twist.covariance[6*i + j + 21] =
          covariance(ErrorStateVector::D_GYRO_DRIFT_X + i, ErrorStateVector::D_GYRO_DRIFT_X + j);
    }
  }
  publ_odom_.publish(odometry_msg);

  geometry_msgs::Vector3Stamped accel_bias_msg;
  accel_bias_msg.header.stamp.fromSec(timestamp);
  accel_bias_msg.header.frame_id = frame_id_;
  accel_bias_msg.vector.x = state(NominalStateVector::ACC_BIAS_X);
  accel_bias_msg.vector.y = state(NominalStateVector::ACC_BIAS_Y);
  accel_bias_msg.vector.z = state(NominalStateVector::ACC_BIAS_Z);
  publ_accel_bias_.publish(accel_bias_msg);

  geometry_msgs::Vector3Stamped gyro_drift_msg;
  gyro_drift_msg.header.stamp.fromSec(timestamp);
  gyro_drift_msg.header.frame_id = frame_id_;
  gyro_drift_msg.vector.x = state(NominalStateVector::GYRO_DRIFT_X);
  gyro_drift_msg.vector.y = state(NominalStateVector::GYRO_DRIFT_Y);
  gyro_drift_msg.vector.z = state(NominalStateVector::GYRO_DRIFT_Z);
  publ_gyro_drift_.publish(gyro_drift_msg);

}
