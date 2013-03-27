/**
 * @file
 * @brief Visual measurement error vector for the pose-twist filter (presentation).
 * @author Joan Pau Beltran
 *
 * This file describes the visual measurement error vector for the filter,
 * and provides conversions to the vector format used by the BFL library.
 */

#ifndef VISUAL_MEASUREMENT_ERROR_VECTOR_H
#define VISUAL_MEASUREMENT_ERROR_VECTOR_H

#include <wrappers/matrix/vector_wrapper.h> // BFL vector class
#include <Eigen/Geometry> // Eigen vector and quaternion classes

namespace pose_twist_meskf
{

/**
 * @brief Visual measurement error vector.
 */
struct VisualMeasurementErrorVector
{
  void fromVector(const MatrixWrapper::ColumnVector& e);
  MatrixWrapper::ColumnVector toVector() const;

  Eigen::Vector3d d_position_;       //!< Position error estimate.
  Eigen::Vector3d d_lin_vel_;        //!< Linear velocity error estimate.
  Eigen::Vector3d d_orientation_;    //!< Orientation error estimate.
  Eigen::Vector3d d_acc_bias_;       //!< Accelerometers' bias error estimate.
  Eigen::Vector3d d_gyro_drift_;     //!< Gyroscopes' drift error estimate.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Index for the input vector
  static const int DIMENSION = 15; //!< Visual measurement error vector dimension.
  enum Index
  {
    D_POSITION_X = 1,
    D_POSITION_Y,
    D_POSITION_Z,
    D_LIN_VEL_X,
    D_LIN_VEL_Y,
    D_LIN_VEL_Z,
    D_ACC_BIAS_X,
    D_ACC_BIAS_Y,
    D_ACC_BIAS_Z,
    D_ORIENTATION_X,
    D_ORIENTATION_Y,
    D_ORIENTATION_Z,
    D_GYRO_DRIFT_X,
    D_GYRO_DRIFT_Y,
    D_GYRO_DRIFT_Z
  }; // enum

}; // struct

} // namespace

#endif // VISUAL_MEASUREMEMT_ERROR_VECTOR_H
