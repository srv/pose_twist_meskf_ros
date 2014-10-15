/**
 * @file
 * @brief Pose twist error state extended Kalman filter
 * with multiplicative orientation error node class (implementation).
 */

#include "pose_twist_meskf_node_base.h"
#include "nominal_state_vector.h"
#include "input_vector.h"
#include "depth_measurement_vector.h"
#include "error_state_vector.h"
#include "visual_measurement_error_vector.h"
#include "depth_measurement_error_vector.h"
#include <exception>

/**
 * @brief Default constructor.
 * @return
 */
pose_twist_meskf::PoseTwistMESKFNodeBase::
PoseTwistMESKFNodeBase(): filter_initialized_(false)
{
  // Node handlers
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  // Node parameters.
  initializeParameters(local_nh);

  // Subscribe to topics
  subs_imu_ = nh.subscribe("imu", 10,
                              &PoseTwistMESKFNodeBase::IMUCallback,
                              this);
  subs_visual_odom_ = nh.subscribe("visual_odometry", 10,
                                      &PoseTwistMESKFNodeBase::visualCallback,
                                      this);
  subs_depth_ = nh.subscribe("depth", 10,
                                &PoseTwistMESKFNodeBase::depthCallback,
                                this);

  // Advertise output topics.
  publ_pose_ = local_nh.advertise<nav_msgs::Odometry>("odometry", 10);
  publ_gyro_drift_ = local_nh.advertise<geometry_msgs::Vector3Stamped>("gyroscope_bias", 10);
  publ_accel_bias_ = local_nh.advertise<geometry_msgs::Vector3Stamped>("accelerometer_bias", 10);

  // Initialize the update timer
  if (update_timer_.isValid() && update_timer_.hasPending())
    ROS_INFO("Filter reset!");
  else
  {
    if (update_timer_.isValid())
    {
      update_timer_.start();
    }
    else
    {
      update_timer_ = nh.createTimer(ros::Duration(update_rate_),
                                        boost::bind(&PoseTwistMESKFNodeBase::updateCallback, this));
      ROS_INFO("Filter started!");
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
  if (use_depth_)
    state(NominalStateVector::POSITION_Z) = -last_depth_msg_->depth;
  else
    state(NominalStateVector::POSITION_Z) = last_visual_msg_->pose.pose.position.z;
  state(NominalStateVector::ORIENTATION_X) = last_visual_msg_->pose.pose.orientation.x;
  state(NominalStateVector::ORIENTATION_Y) = last_visual_msg_->pose.pose.orientation.y;
  state(NominalStateVector::ORIENTATION_Z) = last_visual_msg_->pose.pose.orientation.z;
  state(NominalStateVector::ORIENTATION_W) = last_visual_msg_->pose.pose.orientation.w;
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
        covariance(ErrorStateVector::D_LIN_VEL_X + i,
                   ErrorStateVector::D_LIN_VEL_X + j) = last_visual_msg_->twist.covariance[6*i + j];
        covariance(ErrorStateVector::D_ORIENTATION_X + i,
                   ErrorStateVector::D_ORIENTATION_X + j) = last_visual_msg_->pose.covariance[6*i + j + 21];
        covariance(ErrorStateVector::D_GYRO_DRIFT_X + i,
                   ErrorStateVector::D_GYRO_DRIFT_X + j) = last_visual_msg_->twist.covariance[6*i + j + 21];
      }
    }
  }
  else
  {
    covariance = COV_ERROR_STATE_;
  }

  // Save for acceleration computation
  double timestamp = last_visual_msg_->header.stamp.toSec();
  vm_prev_.orientation_.x() = last_visual_msg_->pose.pose.orientation.x;
  vm_prev_.orientation_.y() = last_visual_msg_->pose.pose.orientation.y;
  vm_prev_.orientation_.z() = last_visual_msg_->pose.pose.orientation.z;
  vm_prev_.orientation_.w() = last_visual_msg_->pose.pose.orientation.w;
  vm_prev_.lin_vel_.x() = last_visual_msg_->twist.twist.linear.x;
  vm_prev_.lin_vel_.y() = last_visual_msg_->twist.twist.linear.y;
  vm_prev_.lin_vel_.z() = last_visual_msg_->twist.twist.linear.z;
  vm_timestamp_prev_ = timestamp;

  try
  {
    filter_.setUpSystem(VAR_ACC_, VAR_GYRO_, VAR_ACC_BIAS_, VAR_GYRO_DRIFT_, G_VEC_);
    filter_.setUpMeasurementModels();
    filter_.initialize(state, covariance, ros::Time::now().toSec());
    filter_initialized_ = true;
  }
  catch(std::exception& e)
  {
    ROS_WARN("A problem occurred while initializing the filter, I will try again...");
    filter_initialized_ = false;
  }
}

/**
 * @brief Initialize parameters from parameter server or default values.
 * @param local_nh node handler for retrieving the parameters.
 */
void pose_twist_meskf::PoseTwistMESKFNodeBase::initializeParameters(const ros::NodeHandle& local_nh)
{
  std::string use_topic_cov, use_depth, tf_with_odom_stamp;
  local_nh.param<std::string>("frame_id", frame_id_, "/map");
  local_nh.param<std::string>("child_frame_id", child_frame_id_, "/base_link");
  local_nh.param<std::string>("visual_odometry_frame_id", visual_odom_frame_id_,"/odom");
  local_nh.param<std::string>("use_topic_cov", use_topic_cov, "false");
  local_nh.param<std::string>("use_depth", use_depth, "true");
  local_nh.param<std::string>("tf_with_odom_stamp", tf_with_odom_stamp, "false");
  std::istringstream is_1(use_topic_cov);
  is_1 >> std::boolalpha >> use_topic_cov_;
  std::istringstream is_2(use_depth);
  is_2 >> std::boolalpha >> use_depth_;
  std::istringstream is_3(tf_with_odom_stamp);
  is_3 >> std::boolalpha >> tf_with_odom_stamp_;
  local_nh.param("update_rate", update_rate_, 0.1);
  G_VEC_(0) = readDoubleParameter(local_nh, "gravity_x", "0.0");
  G_VEC_(1) = readDoubleParameter(local_nh, "gravity_y", "0.0");
  G_VEC_(2) = readDoubleParameter(local_nh, "gravity_z", "9.80665");
  VAR_ACC_ = readDoubleParameter(local_nh, "var_acc", "4e-5");
  VAR_ACC_BIAS_ = readDoubleParameter(local_nh, "var_acc_bias", "1e-10");
  VAR_GYRO_ = readDoubleParameter(local_nh, "var_gyro", "1e-6");
  VAR_GYRO_DRIFT_ = readDoubleParameter(local_nh, "var_gyro_drift", "1e-10");

  // Error state covariance
  XmlRpc::XmlRpcValue cov_params;
  local_nh.getParam("cov_error_state", cov_params);
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
  local_nh.getParam("cov_visual_odom", cov_params);
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
  local_nh.getParam("cov_visual_odom_failure", cov_params);
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
  double cov_depth_val = readDoubleParameter(local_nh, "cov_depth", "1.0e-12");
  PoseTwistMESKF::SymmetricMatrix cov_depth(DepthMeasurementErrorVector::DIMENSION);
  cov_depth(DepthMeasurementErrorVector::D_DEPTH_Z,
    DepthMeasurementErrorVector::D_DEPTH_Z) = cov_depth_val;
  COV_DEPTH_ = cov_depth;

  // Info
  ROS_INFO_STREAM("Reference frame: " << frame_id_ << ".");
  ROS_INFO_STREAM("Body-fixed frame: " << child_frame_id_ << ".");
  ROS_INFO_STREAM("Visual odometry frame: " << visual_odom_frame_id_ << ".");
  ROS_INFO_STREAM("Update rate: " << update_rate_ << " ms.");
  ROS_INFO_STREAM("Use topic covariances: " << use_topic_cov_ << ".");
  ROS_INFO_STREAM("Use depth messages: " << use_depth_ << ".");
}

/**
 * @brief Read double parameters from parameter server.
 * @param local_nh node handler for retrieving the parameters.
 * @param paramName is the name of the parameter in the parameter server
 * @param defaultValue is the default value of the parameter (in string)
 */
double pose_twist_meskf::PoseTwistMESKFNodeBase::readDoubleParameter(
  const ros::NodeHandle& local_nh, std::string paramName, std::string defaultValue)
{
  double val = -1.0;
  std::string param;
  local_nh.param<std::string>(paramName, param, defaultValue);
  try
  {
    val = boost::lexical_cast<double>(param);
  }
  catch( boost::bad_lexical_cast const& ) {}
  return val;
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

  filter_.addInput(ros::Time::now().toSec(), input);
  ROS_DEBUG("IMU measurement encued");
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

  // Save odometry timestamp to sync odometry with the output of the filter
  visual_msg_timestamp_ = msg->header.stamp;

  // Compute acceleration
  double timestamp = msg->header.stamp.toSec();
  VisualMeasurementVector vm_curr;
  vm_curr.orientation_.x() = msg->pose.pose.orientation.x;
  vm_curr.orientation_.y() = msg->pose.pose.orientation.y;
  vm_curr.orientation_.z() = msg->pose.pose.orientation.z;
  vm_curr.orientation_.w() = msg->pose.pose.orientation.w;
  vm_curr.lin_vel_.x() = msg->twist.twist.linear.x;
  vm_curr.lin_vel_.y() = msg->twist.twist.linear.y;
  vm_curr.lin_vel_.z() = msg->twist.twist.linear.z;
  vm_curr.lin_acc_ = ( (vm_prev_.orientation_.inverse()*vm_curr.orientation_).toRotationMatrix()
    * vm_curr.lin_vel_- vm_prev_.lin_vel_ ) / (timestamp - vm_timestamp_prev_);

  // Save measurement
  PoseTwistMESKF::Vector measurement(VisualMeasurementVector::DIMENSION);
  measurement(VisualMeasurementVector::POSITION_X) = msg->pose.pose.position.x;
  measurement(VisualMeasurementVector::POSITION_Y) = msg->pose.pose.position.y;
  measurement(VisualMeasurementVector::POSITION_Z) = msg->pose.pose.position.z;
  measurement(VisualMeasurementVector::ORIENTATION_X) = msg->pose.pose.orientation.x;
  measurement(VisualMeasurementVector::ORIENTATION_Y) = msg->pose.pose.orientation.y;
  measurement(VisualMeasurementVector::ORIENTATION_Z) = msg->pose.pose.orientation.z;
  measurement(VisualMeasurementVector::ORIENTATION_W) = msg->pose.pose.orientation.w;
  measurement(VisualMeasurementVector::LIN_VEL_X) = msg->twist.twist.linear.x;
  measurement(VisualMeasurementVector::LIN_VEL_Y) = msg->twist.twist.linear.y;
  measurement(VisualMeasurementVector::LIN_VEL_Z) = msg->twist.twist.linear.z;
  measurement(VisualMeasurementVector::ANG_VEL_X) = msg->twist.twist.angular.x;
  measurement(VisualMeasurementVector::ANG_VEL_Y) = msg->twist.twist.angular.y;
  measurement(VisualMeasurementVector::ANG_VEL_Z) = msg->twist.twist.angular.z;
  measurement(VisualMeasurementVector::LIN_ACC_X) = vm_curr.lin_acc_.x();
  measurement(VisualMeasurementVector::LIN_ACC_Y) = vm_curr.lin_acc_.y();
  measurement(VisualMeasurementVector::LIN_ACC_Z) = vm_curr.lin_acc_.z();

  // Save for acceleration computation
  vm_prev_.orientation_.x() = msg->pose.pose.orientation.x;
  vm_prev_.orientation_.y() = msg->pose.pose.orientation.y;
  vm_prev_.orientation_.z() = msg->pose.pose.orientation.z;
  vm_prev_.orientation_.w() = msg->pose.pose.orientation.w;
  vm_prev_.lin_vel_.x() = msg->twist.twist.linear.x;
  vm_prev_.lin_vel_.y() = msg->twist.twist.linear.y;
  vm_prev_.lin_vel_.z() = msg->twist.twist.linear.z;
  vm_timestamp_prev_ = timestamp;

  PoseTwistMESKF::SymmetricMatrix covariance(VisualMeasurementErrorVector::DIMENSION);
  if (use_topic_cov_)
  {
    for (int i=0; i<3; i++)
    {
      for (int j=0; j<3; j++)
      {
        covariance(VisualMeasurementErrorVector::D_POSITION_X + i,
                   VisualMeasurementErrorVector::D_POSITION_X + j) = msg->pose.covariance[6*i + j];
        covariance(VisualMeasurementErrorVector::D_LIN_VEL_X + i,
                   VisualMeasurementErrorVector::D_LIN_VEL_X + j) = msg->twist.covariance[6*i + j];
        covariance(VisualMeasurementErrorVector::D_ORIENTATION_X + i,
                   VisualMeasurementErrorVector::D_ORIENTATION_X + j) = msg->pose.covariance[6*i + j + 21];
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
      ROS_WARN("Visual odometry failure. Using covariance: COV_VISUAL_ODOM_FAILURE");
    }
    else
    {
      covariance = COV_VISUAL_ODOM_;
    }
  }

  filter_.addMeasurement(filter_.VISUAL, measurement, covariance, ros::Time::now().toSec());
  ROS_DEBUG("VISUAL_ODOM measurement encued");
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

  PoseTwistMESKF::Vector measurement(DepthMeasurementVector::DIMENSION);
  measurement(DepthMeasurementVector::DEPTH_Z) = -msg.depth;

  PoseTwistMESKF::SymmetricMatrix covariance(DepthMeasurementErrorVector::DIMENSION);

  // AUV_sensor_msgs::Depth has no covariance field
  filter_.addMeasurement(filter_.DEPTH, measurement, COV_DEPTH_, ros::Time::now().toSec());
  ROS_DEBUG("DEPTH measurement encued");
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
    if (last_visual_msg_ && use_depth_ && last_depth_msg_)
    {
      // Filter needs one visual odometry message
      // and one depth message to be initialized
      initializeMeskf();
    }
    else
    {
      if (last_visual_msg_ && !use_depth_)
      {
        initializeMeskf();
      }
    }

    return;
  }

  // Update
  bool success = filter_.updateAll();

  if (!success)
  {
    ROS_WARN("Could not update filter.");
    return;
  }
  ROS_INFO("Filter updated!");

  // Get results
  const PoseTwistMESKF::Vector state = filter_.getEstimate();
  const PoseTwistMESKF::SymmetricMatrix covariance = filter_.getCovariance();
  ros::Time timestamp = ros::Time::now();

  // Publish messages
  nav_msgs::Odometry pose_msg;
  pose_msg.header.stamp = timestamp;
  pose_msg.header.frame_id = frame_id_;
  pose_msg.child_frame_id = child_frame_id_;
  pose_msg.pose.pose.position.x = state(NominalStateVector::POSITION_X);
  pose_msg.pose.pose.position.y = state(NominalStateVector::POSITION_Y);
  pose_msg.pose.pose.position.z = state(NominalStateVector::POSITION_Z);
  pose_msg.pose.pose.orientation.w = state(NominalStateVector::ORIENTATION_W);
  pose_msg.pose.pose.orientation.x = state(NominalStateVector::ORIENTATION_X);
  pose_msg.pose.pose.orientation.y = state(NominalStateVector::ORIENTATION_Y);
  pose_msg.pose.pose.orientation.z = state(NominalStateVector::ORIENTATION_Z);
  pose_msg.twist.twist.linear.x = state(NominalStateVector::LIN_VEL_X);
  pose_msg.twist.twist.linear.y = state(NominalStateVector::LIN_VEL_Y);
  pose_msg.twist.twist.linear.z = state(NominalStateVector::LIN_VEL_Z);
  pose_msg.twist.twist.angular.x = state(NominalStateVector::ANG_VEL_X);
  pose_msg.twist.twist.angular.y = state(NominalStateVector::ANG_VEL_Y);
  pose_msg.twist.twist.angular.z = state(NominalStateVector::ANG_VEL_Z);
  for (int i=0; i<3; i++)
  {
    for (int j=0; j<3; j++)
    {
      pose_msg.pose.covariance[6*i + j] =
          covariance(ErrorStateVector::D_POSITION_X + i, ErrorStateVector::D_POSITION_X + j);
      pose_msg.pose.covariance[6*i + j + 3] =
          covariance(ErrorStateVector::D_POSITION_X + i, ErrorStateVector::D_ORIENTATION_X + j);
      pose_msg.pose.covariance[6*i + j + 18] =
          covariance(ErrorStateVector::D_ORIENTATION_X + i, ErrorStateVector::D_POSITION_X + j);
      pose_msg.pose.covariance[6*i + j + 21] =
          covariance(ErrorStateVector::D_ORIENTATION_X + i, ErrorStateVector::D_ORIENTATION_X + j);
      pose_msg.twist.covariance[6*i + j] =
          covariance(ErrorStateVector::D_LIN_VEL_X + i, ErrorStateVector::D_LIN_VEL_X + j);
      pose_msg.twist.covariance[6*i + j + 3] =
          covariance(ErrorStateVector::D_LIN_VEL_X + i, ErrorStateVector::D_GYRO_DRIFT_X + j);
      pose_msg.twist.covariance[6*i + j + 18] =
          covariance(ErrorStateVector::D_GYRO_DRIFT_X + i, ErrorStateVector::D_LIN_VEL_X + j);
      pose_msg.twist.covariance[6*i + j + 21] =
          covariance(ErrorStateVector::D_GYRO_DRIFT_X + i, ErrorStateVector::D_GYRO_DRIFT_X + j);
    }
  }
  publ_pose_.publish(pose_msg);

  tf::Vector3 t   ( state(NominalStateVector::POSITION_X),
                    state(NominalStateVector::POSITION_Y),
                    state(NominalStateVector::POSITION_Z));
  tf::Quaternion q( state(NominalStateVector::ORIENTATION_X),
                    state(NominalStateVector::ORIENTATION_Y),
                    state(NominalStateVector::ORIENTATION_Z),
                    state(NominalStateVector::ORIENTATION_W));
  tf::Transform transform(q, t);

  geometry_msgs::Vector3Stamped accel_bias_msg;
  accel_bias_msg.header.stamp = timestamp;
  accel_bias_msg.header.frame_id = frame_id_;
  accel_bias_msg.vector.x = state(NominalStateVector::ACC_BIAS_X);
  accel_bias_msg.vector.y = state(NominalStateVector::ACC_BIAS_Y);
  accel_bias_msg.vector.z = state(NominalStateVector::ACC_BIAS_Z);
  publ_accel_bias_.publish(accel_bias_msg);

  geometry_msgs::Vector3Stamped gyro_drift_msg;
  gyro_drift_msg.header.stamp = timestamp;
  gyro_drift_msg.header.frame_id = frame_id_;
  gyro_drift_msg.vector.x = state(NominalStateVector::GYRO_DRIFT_X);
  gyro_drift_msg.vector.y = state(NominalStateVector::GYRO_DRIFT_Y);
  gyro_drift_msg.vector.z = state(NominalStateVector::GYRO_DRIFT_Z);
  publ_gyro_drift_.publish(gyro_drift_msg);

  // Publish transform
  if (tf_with_odom_stamp_ == true)
    timestamp = visual_msg_timestamp_;
  tf_broadcaster_.sendTransform(
      tf::StampedTransform(transform, timestamp,
      frame_id_, child_frame_id_));
}
