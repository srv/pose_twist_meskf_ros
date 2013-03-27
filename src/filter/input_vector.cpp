/**
 * @file
 * @brief Input state vector for the pose-twist filter (implementation).
 * @author Joan Pau Beltran
 *
 * This file describes the input vector for the filter,
 * and provides conversions to the vector format used by the BFL library.
 */

#include "input_vector.h"

/**
 * @brief Read the input entities from a BFL vector.
 * @param u vector to decompose.
 */
void pose_twist_meskf::InputVector::fromVector(const MatrixWrapper::ColumnVector& u)
{
  lin_acc_(0) = u(LIN_ACC_X);
  lin_acc_(1) = u(LIN_ACC_Y);
  lin_acc_(2) = u(LIN_ACC_Z);
  ang_vel_(0) = u(ANG_VEL_X);
  ang_vel_(1) = u(ANG_VEL_Y);
  ang_vel_(2) = u(ANG_VEL_Z);
  time_incr_  = u(TIME_INCR);
}


/**
 * @brief Write the input entities to a BFL vector.
 * @return composed vector.
 */
MatrixWrapper::ColumnVector pose_twist_meskf::InputVector::toVector() const
{
  MatrixWrapper::ColumnVector u(DIMENSION);
  u(LIN_ACC_X) = lin_acc_(0);
  u(LIN_ACC_Y) = lin_acc_(1);
  u(LIN_ACC_Z) = lin_acc_(2);
  u(ANG_VEL_X) = ang_vel_(0);
  u(ANG_VEL_Y) = ang_vel_(1);
  u(ANG_VEL_Z) = ang_vel_(2);
  u(TIME_INCR) = time_incr_;
  return u;
}
