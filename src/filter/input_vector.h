/**
 * @file
 * @brief Input state vector for the pose-twist filter (presentation).
 * @author Joan Pau Beltran
 *
 * This file describes the input vector for the filter,
 * and provides conversions to the vector format used by the BFL library.
 */

#ifndef INPUT_VECTOR_H
#define INPUT_VECTOR_H

#include <wrappers/matrix/vector_wrapper.h> // BFL vector class
#include <Eigen/Geometry> // Eigen vector and quaternion classes

namespace pose_twist_meskf
{

/**
 * @brief Input vector.
 */
struct InputVector
{
  void fromVector(const MatrixWrapper::ColumnVector& u);
  MatrixWrapper::ColumnVector toVector() const;

  Eigen::Vector3d lin_acc_; //!< Accelerometers' reading.
  Eigen::Vector3d ang_vel_; //!< Gyroscopes' reading.
  double time_incr_;        //!< Elapsed time since previous input.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Index for the input vector
  static const int DIMENSION = 7; //!< Input vector dimension.
  enum Index
  {
    LIN_ACC_X = 1,
    LIN_ACC_Y,
    LIN_ACC_Z,
    ANG_VEL_X,
    ANG_VEL_Y,
    ANG_VEL_Z,
    TIME_INCR
  }; // enum

}; // struct

} // namespace

#endif // INPUT_VECTOR_H
