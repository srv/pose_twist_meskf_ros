/**
 * @file
 * @brief Visual measurement vector for the pose-twist filter (presentation).
 * @author Joan Pau Beltran
 *
 * This file describes the visual measurement vector for the filter,
 * and provides conversions to the vector format used by the BFL library.
 */

#ifndef VISUAL_MEASUREMENT_VECTOR_H
#define VISUAL_MEASUREMENT_VECTOR_H

#include <bfl/wrappers/matrix/vector_wrapper.h> // BFL vector class
#include <Eigen/Geometry> // Eigen vector and quaternion classes

namespace pose_twist_meskf
{

/**
 * @brief Visual measurement vector.
 */
struct VisualMeasurementVector
{
  void fromVector(const MatrixWrapper::ColumnVector& z);
  MatrixWrapper::ColumnVector toVector() const;

  Eigen::Vector3d position_;       //!< Position estimate.
  Eigen::Vector3d lin_vel_;        //!< Linear velocity estimate.
  Eigen::Quaterniond orientation_; //!< Orientation estimate.
  Eigen::Vector3d lin_acc_;        //!< Linear acceleration in body-fixed frame.
  Eigen::Vector3d ang_vel_;        //!< Angular velocity in body-fixed frame.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Index for the visual measurement vector
  static const int DIMENSION = 16; //!< Visual measurement vector dimension.
  enum Index
  {
    POSITION_X = 1,
    POSITION_Y,
    POSITION_Z,
    LIN_VEL_X,
    LIN_VEL_Y,
    LIN_VEL_Z,
    LIN_ACC_X,
    LIN_ACC_Y,
    LIN_ACC_Z,
    ORIENTATION_X,
    ORIENTATION_Y,
    ORIENTATION_Z,
    ORIENTATION_W,
    ANG_VEL_X,
    ANG_VEL_Y,
    ANG_VEL_Z
  }; // enum

}; // struct

} // namespace

#endif // VISUAL_MEASUREMEMT_VECTOR_H
