/**
 * @file
 * @brief Visual measurement error vector for the pose-twist filter (implementation).
 * @author Joan Pau Beltran
 *
 * This file describes the visual measurement error vector for the filter,
 * and provides conversions to the vector format used by the BFL library.
 */

#include "depth_measurement_error_vector.h"

/**
 * @brief Read the visual measurement error entities from a BFL vector.
 * @param e vector to decompose.
 */
void pose_twist_meskf::DepthMeasurementErrorVector::fromVector(const MatrixWrapper::ColumnVector& e)
{
  d_depth = e(D_DEPTH_Z);
}


/**
 * @brief Write the visual measurement error entities to a BFL vector.
 * @param e vector to compose (should have the correct dimension).
 */
void pose_twist_meskf::DepthMeasurementErrorVector::toVector(MatrixWrapper::ColumnVector& e) const
{
  e(D_DEPTH_Z) = d_depth;
}
