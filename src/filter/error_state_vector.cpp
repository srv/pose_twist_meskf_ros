/**
 * @file
 * @brief Error state vector for the pose-twist filter (implementation).
 * @author Joan Pau Beltran
 *
 * This file describes the error state vector for the filter,
 * and provides conversions to the vector format used by the BFL library.
 */

#include "error_state_vector.h"

/**
 * @brief Read the nominal state entities from a BFL vector.
 * @param e vector to decompose.
 */
void pose_twist_meskf::ErrorStateVector::fromVector(const MatrixWrapper::ColumnVector& e)
{
  d_position_(0) = e(D_POSITION_X);
  d_position_(1) = e(D_POSITION_Y);
  d_position_(2) = e(D_POSITION_Z);
  d_lin_vel_(0) = e(D_LIN_VEL_X);
  d_lin_vel_(1) = e(D_LIN_VEL_Y);
  d_lin_vel_(2) = e(D_LIN_VEL_Z);
  d_acc_bias_(0) = e(D_ACC_BIAS_X);
  d_acc_bias_(1) = e(D_ACC_BIAS_Y);
  d_acc_bias_(2) = e(D_ACC_BIAS_Z);
  d_orientation_(0) = e(D_ORIENTATION_X);
  d_orientation_(1) = e(D_ORIENTATION_Y);
  d_orientation_(2) = e(D_ORIENTATION_Z);
  d_gyro_drift_(0) = e(D_GYRO_DRIFT_X);
  d_gyro_drift_(1) = e(D_GYRO_DRIFT_Y);
  d_gyro_drift_(2) = e(D_GYRO_DRIFT_Z);
}


/**
 * @brief Write the error state entities to a BFL vector.
 * @return composed vector.
 */
MatrixWrapper::ColumnVector pose_twist_meskf::ErrorStateVector::toVector() const
{
  MatrixWrapper::ColumnVector e(DIMENSION);
  e(D_POSITION_X) = d_position_(0);
  e(D_POSITION_Y) = d_position_(1);
  e(D_POSITION_Z) = d_position_(2);
  e(D_LIN_VEL_X) = d_lin_vel_(0);
  e(D_LIN_VEL_Y) = d_lin_vel_(1);
  e(D_LIN_VEL_Z) = d_lin_vel_(2);
  e(D_ACC_BIAS_X) = d_acc_bias_(0);
  e(D_ACC_BIAS_Y) = d_acc_bias_(1);
  e(D_ACC_BIAS_Z) = d_acc_bias_(2);
  e(D_ORIENTATION_X) = d_orientation_(0);
  e(D_ORIENTATION_Y) = d_orientation_(1);
  e(D_ORIENTATION_Z) = d_orientation_(2);
  e(D_GYRO_DRIFT_X) = d_gyro_drift_(0);
  e(D_GYRO_DRIFT_Y) = d_gyro_drift_(1);
  e(D_GYRO_DRIFT_Z) = d_gyro_drift_(2);
  return e;
}
