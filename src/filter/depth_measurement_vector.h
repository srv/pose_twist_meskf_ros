/**
 * @file
 * @brief Depth measurement vector for the pose-twist filter (presentation).
 * @author Fco Bonin Font
 *
 * This file describes the Depth measurement vector for the filter,
 * and provides conversions to the vector format used by the BFL library.
 */

#ifndef DEPTH_MEASUREMENT_VECTOR_H
#define DEPTH_MEASUREMENT_VECTOR_H

#include <bfl/wrappers/matrix/vector_wrapper.h> // BFL vector class
#include <Eigen/Geometry> // Eigen vector and quaternion classes

namespace pose_twist_meskf
{

/**
 * @brief Depth measurement vector.
 */
struct  DepthMeasurementVector
{
  void fromVector(const MatrixWrapper::ColumnVector& z);
  //void toVector(MatrixWrapper::ColumnVector& z) const;
  MatrixWrapper::ColumnVector toVector() const;
  double depth_;       //!< depth estimate from pressure sensor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // Index for the depth measurement vector
  static const int DIMENSION = 1; //!< depth measurement vector dimension.
  enum Index
  {
    DEPTH_Z = 1,
  }; // enum

}; // struct

} // namespace

#endif // DEPTH_MEASUREMEMT_VECTOR_H
