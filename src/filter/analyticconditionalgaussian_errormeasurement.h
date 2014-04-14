/**
 * @file
 * @author Joan Pau Beltran
 * @brief Conditional distribution for error measurements in ESKF (presentation).
 *
 * Provide an abstract class that extends the BFL conditional pdf class
 * for measurements in error state Kalman filtering.
 * It just adds a pure virtual member function
 * to convert nominal state measurement to an error state mesaurements.
  */

#ifndef ANALYTICCONDITIONALGAUSSIAN_ERRORMEASUREMENT_H
#define ANALYTICCONDITIONALGAUSSIAN_ERRORMEASUREMENT_H

#include <bfl/pdf/analyticconditionalgaussian_additivenoise.h>

namespace BFL
{

/**
 * @brief Analytic conditional gaussian for error measurements.
 *
 * This abstract class is intended to be used with error state Kalman filters.
 * It just adds a pure virtual member function ErrorMeasurement()
 * that converts a nominal state measurement into an error state mesaurement
 * given the current nominal state.
 */
class AnalyticConditionalGaussianErrorMeasurement
: public AnalyticConditionalGaussianAdditiveNoise
{
public:

  // Constructor when uncertainty is already known
  AnalyticConditionalGaussianErrorMeasurement(const Gaussian& additive_noise);

  // Constructor when uncertainty is not yet known
  AnalyticConditionalGaussianErrorMeasurement(int dim = 0);

  // Default copy constructor will do

  // Destructor
  virtual ~AnalyticConditionalGaussianErrorMeasurement();

  // Virtual functions are inherited from base class.

  // Pure virtual function to convert nominal measurements to error measurements.
  virtual MatrixWrapper::ColumnVector ErrorMeasurement(const MatrixWrapper::ColumnVector& z,
                                                       const MatrixWrapper::ColumnVector& x) const = 0;
};

} // End namespace BFL

#endif // LINEARANALYTICCONDITIONALGAUSSIAN_ERRORMEASUREMENT_H
