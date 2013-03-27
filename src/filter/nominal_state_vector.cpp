/**
 * @file
 * @brief Nominal state vector for the pose-twist filter (implementation).
 * @author Joan Pau Beltran
 *
 * This file describes the nominal state vector for the filter,
 * and provides conversions to the vector format used by the BFL library.
 */

#include "nominal_state_vector.h"

/**
 * @brief Read the nominal state entities from a BFL vector.
 * @param x vector to decompose.
 */
void pose_twist_meskf::NominalStateVector::fromVector(const MatrixWrapper::ColumnVector& x)
{
  position_(0) = x(POSITION_X);
  position_(1) = x(POSITION_Y);
  position_(2) = x(POSITION_Z);
  lin_vel_(0) = x(LIN_VEL_X);
  lin_vel_(1) = x(LIN_VEL_Y);
  lin_vel_(2) = x(LIN_VEL_Z);
  acc_bias_(0) = x(ACC_BIAS_X);
  acc_bias_(1) = x(ACC_BIAS_Y);
  acc_bias_(2) = x(ACC_BIAS_Z);
  orientation_.x() = x(ORIENTATION_X);
  orientation_.y() = x(ORIENTATION_Y);
  orientation_.z() = x(ORIENTATION_Z);
  orientation_.w() = x(ORIENTATION_W);
  gyro_drift_(0) = x(GYRO_DRIFT_X);
  gyro_drift_(1) = x(GYRO_DRIFT_Y);
  gyro_drift_(2) = x(GYRO_DRIFT_Z);
  lin_acc_(0) = x(LIN_ACC_X);
  lin_acc_(1) = x(LIN_ACC_Y);
  lin_acc_(2) = x(LIN_ACC_Z);
  ang_vel_(0) = x(ANG_VEL_X);
  ang_vel_(1) = x(ANG_VEL_Y);
  ang_vel_(2) = x(ANG_VEL_Z);
}


/**
 * @brief Write the nominal state entities to a BFL vector.
 * @return composed vector.
 */
MatrixWrapper::ColumnVector
pose_twist_meskf::NominalStateVector::toVector() const
{
  MatrixWrapper::ColumnVector x(DIMENSION);
  x(POSITION_X) = position_(0);
  x(POSITION_Y) = position_(1);
  x(POSITION_Z) = position_(2);
  x(LIN_VEL_X) = lin_vel_(0);
  x(LIN_VEL_Y) = lin_vel_(1);
  x(LIN_VEL_Z) = lin_vel_(2);
  x(ACC_BIAS_X) = acc_bias_(0);
  x(ACC_BIAS_Y) = acc_bias_(1);
  x(ACC_BIAS_Z) = acc_bias_(2);
  x(ORIENTATION_X) = orientation_.x();
  x(ORIENTATION_Y) = orientation_.y();
  x(ORIENTATION_Z) = orientation_.z();
  x(ORIENTATION_W) = orientation_.w();
  x(GYRO_DRIFT_X) = gyro_drift_(0);
  x(GYRO_DRIFT_Y) = gyro_drift_(1);
  x(GYRO_DRIFT_Z) = gyro_drift_(2);
  x(LIN_ACC_X) = lin_acc_(0);
  x(LIN_ACC_Y) = lin_acc_(1);
  x(LIN_ACC_Z) = lin_acc_(2);
  x(ANG_VEL_X) = ang_vel_(0);
  x(ANG_VEL_Y) = ang_vel_(1);
  x(ANG_VEL_Z) = ang_vel_(2);
  return x;
};
