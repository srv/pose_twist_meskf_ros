/**
 * @file
 * @brief Nominal state vector for the pose-twist filter (presentation).
 * @author Joan Pau Beltran
 *
 * This file describes the nominal state vector for the filter,
 * and provides conversions to the vector format used by the BFL library.
 */

#ifndef NOMINAL_STATE_VECTOR_H
#define NOMINAL_STATE_VECTOR_H

#include <wrappers/matrix/vector_wrapper.h> // BFL vector class.
#include <Eigen/Geometry> // Eigen vector and quaternion classes.

namespace pose_twist_meskf
{

/**
 * @brief Nominal state vector.
 */
struct NominalStateVector
{
  void fromVector(const MatrixWrapper::ColumnVector& x);
  MatrixWrapper::ColumnVector toVector() const;

  Eigen::Vector3d position_;       //!< Position in reference frame.
  Eigen::Vector3d lin_vel_;        //!< Linear velocity in body-fixed frame.
  Eigen::Quaterniond orientation_; //!< Orientation in reference frame.
  Eigen::Vector3d acc_bias_;       //!< Accelerometers' bias.
  Eigen::Vector3d gyro_drift_;     //!< Gyroscopes' drift.
  Eigen::Vector3d lin_acc_;        //!< Linear acceleration in body-fixed frame.
  Eigen::Vector3d ang_vel_;        //!< Angular velocity in body-fixed frame.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Index for the nominal state vector
  static const int DIMENSION = 22;  //!< Nominal state vector dimension.
  enum Index
  {
    POSITION_X = 1,
    POSITION_Y,
    POSITION_Z,
    LIN_VEL_X,
    LIN_VEL_Y,
    LIN_VEL_Z,
    ACC_BIAS_X,
    ACC_BIAS_Y,
    ACC_BIAS_Z,
    ORIENTATION_X,
    ORIENTATION_Y,
    ORIENTATION_Z,
    ORIENTATION_W,
    GYRO_DRIFT_X,
    GYRO_DRIFT_Y,
    GYRO_DRIFT_Z,
    LIN_ACC_X,
    LIN_ACC_Y,
    LIN_ACC_Z,
    ANG_VEL_X,
    ANG_VEL_Y,
    ANG_VEL_Z
  };  // enum

}; // struct

} // namespace

#endif // NOMINAL_STATE_VECTOR_H
