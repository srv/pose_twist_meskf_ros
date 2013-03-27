/**
 * @file
 * @author Francesc Bonin Font
 * @brief Conditional distribution for measurement from pressure sensor (presentation).
 */

#ifndef ANALYTICCONDITIONALGAUSSIAN_DEPTHMEASUREMENT_H
#define ANALYTICCONDITIONALGAUSSIAN_DEPTHMEASUREMENT_H

#include "analyticconditionalgaussian_errormeasurement.h"

namespace BFL
{
/**
 * @brief Conditional Gaussian for pressure sensor measurements.
 *
 */
class AnalyticConditionalGaussianDepthMeasurement
: public AnalyticConditionalGaussianErrorMeasurement
{
public:

  // Constructor for uncertainty not yet known.
    AnalyticConditionalGaussianDepthMeasurement();

  // Default copy constructor will do

  // Destructor
  virtual ~AnalyticConditionalGaussianDepthMeasurement();

  // Clone function
  virtual AnalyticConditionalGaussianDepthMeasurement* Clone() const;

  // Measurement matrix from nominal state.
  virtual MatrixWrapper::ColumnVector ExpectedValueGet() const;

  // Measurement matrix from nominal state.
  virtual MatrixWrapper::Matrix dfGet(unsigned int i) const;

  // Nominal state measurement to error state measurement conversion
  virtual MatrixWrapper::ColumnVector ErrorMeasurement(const MatrixWrapper::ColumnVector& z,
                                                       const MatrixWrapper::ColumnVector& x) const;

  // Measurement matrix from nominal state.
  MatrixWrapper::Matrix MeasurementMatrix(const MatrixWrapper::ColumnVector& x) const;

};

} // End namespace pose_twist_meskf

#endif // LINEARANALYTICCONDITIONALGAUSSIAN_DEPTHMEASUREMENT_H
