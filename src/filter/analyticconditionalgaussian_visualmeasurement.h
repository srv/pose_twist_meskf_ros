/**
 * @file
 * @author Joan Pau Beltran
 * @brief Conditional distribution for measurement from visual sensors (presentation).
 */

#ifndef ANALYTICCONDITIONALGAUSSIAN_VISUALMEASUREMENT_H
#define ANALYTICCONDITIONALGAUSSIAN_VISUALMEASUREMENT_H

#include "analyticconditionalgaussian_errormeasurement.h"

namespace BFL
{
/**
 * @brief Conditional Gaussian for visual sensor measurements.
 *
 */
class AnalyticConditionalGaussianVisualMeasurement
: public AnalyticConditionalGaussianErrorMeasurement
{
public:

  // Constructor for uncertainty not yet known.
    AnalyticConditionalGaussianVisualMeasurement();

  // Default copy constructor will do

  // Destructor
  virtual ~AnalyticConditionalGaussianVisualMeasurement();

  // Clone function
  virtual AnalyticConditionalGaussianVisualMeasurement* Clone() const;

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

#endif // LINEARANALYTICCONDITIONALGAUSSIAN_VISUALMEASUREMENT_H
