/**
 * @file
 * @author Joan Pau Beltran
 * @brief Conditional distribution for error measurements in ESKF (implementation).
 *
 * Provide an abstract class that extends the BFL conditional pdf class
 * for measurements in error state Kalman filtering.
 * It just adds a pure virtual member function
 * to convert nominal state measurement to an error state mesaurements.
  */

#include "analyticconditionalgaussian_errormeasurement.h"

/**
 * @brief Constructor bypassing its arguments to base conditional pdf constructor.
 * @param gaus additive Gaussian measurement noise pdf.
 * @return
 */
BFL::AnalyticConditionalGaussianErrorMeasurement::
AnalyticConditionalGaussianErrorMeasurement(const Gaussian& additive_noise)
: AnalyticConditionalGaussianAdditiveNoise(additive_noise, 2)
{}


// Constructor when uncertainty is not yet known
/**
 * @brief Constructor bypassing its arguments to base conditional pdf constructor.
 * @param dim measurement dimension.
 * @return
 */
BFL::AnalyticConditionalGaussianErrorMeasurement::
AnalyticConditionalGaussianErrorMeasurement(int dim)
: AnalyticConditionalGaussianAdditiveNoise(dim, 2)
{}


/**
 * @brief Destructor (doing nothing).
 * @return
 */
BFL::AnalyticConditionalGaussianErrorMeasurement::
~AnalyticConditionalGaussianErrorMeasurement()
{}


