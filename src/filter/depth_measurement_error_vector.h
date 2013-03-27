/**
 * @file
 * @brief Depth measurement error vector for the pose-twist filter (presentation).
 * @author Francisco Bonin Font
 *
 * This file describes the Depth measurement error vector for the filter,
 * and provides conversions to the vector format used by the BFL library.
 */

#ifndef DEPTH_MEASUREMENT_ERROR_VECTOR_H
#define DEPTH_MEASUREMENT_ERROR_VECTOR_H

#include <wrappers/matrix/vector_wrapper.h> // BFL vector class
#include <Eigen/Geometry> // Eigen vector and quaternion classes

namespace pose_twist_meskf
{

/**
 * @brief Depth measurement error vector.
 */
struct DepthMeasurementErrorVector
{
  void fromVector(const MatrixWrapper::ColumnVector& e);
  void toVector(MatrixWrapper::ColumnVector& e) const;
	
  double d_depth; // Depth error estimate 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Index for the input vector
  static const int DIMENSION = 1; //!< Visual measurement error vector dimension.
  enum Index
  {
    D_DEPTH_Z = 1,
  }; // enum

}; // struct

} // namespace

#endif // DEPTH_MEASUREMEMT_ERROR_VECTOR_H
