/**
 * @file
 * @brief Visual measurement vector for the pose-twist filter (implementation).
 * @author Joan Pau Beltran
 *
 * This file describes the visual measurement vector for the filter,
 * and provides conversions to the vector format used by the BFL library.
 */

#include "depth_measurement_vector.h"

/**
 * @brief Read the visual measurement entities from a BFL vector.
 * @param z vector to decompose.
 */
void pose_twist_meskf::DepthMeasurementVector::fromVector(const MatrixWrapper::ColumnVector& z)
{
  depth_= z(DEPTH_Z);
}

/**
 * @brief Write the visual measurement entities to a BFL vector.
 * @param z vector to compose (should have the correct dimension).
 */
MatrixWrapper::ColumnVector pose_twist_meskf::DepthMeasurementVector::toVector() const
{
  MatrixWrapper::ColumnVector z(DIMENSION);
  z(DEPTH_Z)=depth_;
  return z;
}
