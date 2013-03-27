/**
 * @file
 * @author Francesc Bonin Font
 * @brief Conditional distribution for measurement from pressure sensors (implementation).
 */

#include "analyticconditionalgaussian_depthmeasurement.h"
#include "nominal_state_vector.h"
#include "error_state_vector.h"
#include "depth_measurement_vector.h"
#include "depth_measurement_error_vector.h"


/**
 * @brief Constructor not setting measurement noise.
 * @param additive_noise pdf representing the additive Gaussian uncertainty.
 * @return
 * Additive noise parameters must be set later with AdditiveNoiseSigmaSet() and
 * AdditiveNoiseMuSet().
 */
BFL::AnalyticConditionalGaussianDepthMeasurement::
AnalyticConditionalGaussianDepthMeasurement()
: AnalyticConditionalGaussianErrorMeasurement(pose_twist_meskf::DepthMeasurementErrorVector::DIMENSION)
{}


/**
 * @brief Compute error measurements between depth measurements and nominal state.
 * @param z depth measurement vector.
 * @param x nominal state vector.
 * @return measurement error vector.
 */
MatrixWrapper::ColumnVector
BFL::AnalyticConditionalGaussianDepthMeasurement::ErrorMeasurement(
  const MatrixWrapper::ColumnVector& z,
  const MatrixWrapper::ColumnVector& x) const
{

  pose_twist_meskf::DepthMeasurementVector measurement;
  measurement.fromVector(z); // just take the depth value

  pose_twist_meskf::NominalStateVector nominal_state;
  nominal_state.fromVector(x);

  pose_twist_meskf::DepthMeasurementErrorVector error;

//std::cout << '\n' << "DEPTH ERROR MEASUREMENT" << measurement.depth_ << ' ' << nominal_state.position_(2);
  error.d_depth = measurement.depth_ - nominal_state.position_(2); // difference between the measured depth and the z coordinate, assuming the z axis pointing down

  MatrixWrapper::ColumnVector e(1); // fbf 23/07/2012 store the error in a vector of 1 element
  error.toVector(e);
  return e;
}


/**
 * @brief Compute expected visual measurement error.
 * @return  expected measurement error conditioned to nominal and error state.
 *
 * The first conditional argument (index 0) is the error state.
 * The second conditional argument (index 1) must be the nominal state.
 */
MatrixWrapper::ColumnVector BFL::AnalyticConditionalGaussianDepthMeasurement::
ExpectedValueGet() const
{
	MatrixWrapper::ColumnVector e(1); // fbf 26-07-2012 vector containing the error in the depth measurement
 e(1) = ConditionalArgumentGet(0)(3);
 /* ConditionalArgumentGet(0)(ErrorStateVector::D_POSITION_Z) --> ConditionalArgumentGet(0): complete error state 
  vector. From the state vector, only the error in the z position must be returned. That would be the D_POSITION_Z index
  One could also writte: ConditionalArgumentGet(0)(3). 
  return ConditionalArgumentGet(0); */
  return e;
}


/**
 * @brief Compute derivative of the conditional measurement. Definitio and computation of the H matrix for the 
 * estimation of the measurement errors from the H and the error estate values. 
 * @param i index of conditional variable to use for partial derivation.
 * @return partial derivative with respect to conditional variable i.
 *
 * This function should be called only for the first conditional argument
 * (index 0) which is the current error state.
 * The second conditional argument (index 1) must be the nominal state.
 */
MatrixWrapper::Matrix
BFL::AnalyticConditionalGaussianDepthMeasurement::dfGet(unsigned int i) const
{
  switch(i)
  {
    case 0:
    {
      // const MatrixWrapper x = ConditionalArgumentGet(1); // K= z-Hx; H: matrix of 1x15, 1 because the measurement 
    	// corresponds to the depth error value, 15 because it is the dimension of the error state vector.
    //	Since H*x must give a number to be substracted to the z(depth) measurement, 
    	// H would be a 1 in the index corresponding to the  D_POSITION_Z and 0 in the rest. 
      MatrixWrapper::Matrix H(pose_twist_meskf::DepthMeasurementErrorVector::DIMENSION,
                              pose_twist_meskf::ErrorStateVector::DIMENSION); // 
      H = 0.0; // initializa all values of the matrix to 0. 
      // Z - Position:
      //  H(pose_twist_meskf::DepthMeasurementErrorVector::D_DEPTH_Z,
      H(pose_twist_meskf::DepthMeasurementErrorVector::D_DEPTH_Z, 
    		  pose_twist_meskf::ErrorStateVector::D_POSITION_Z) = 1.0;  // row 1, column 3 (Z pose coordinate = 1)
      return H;
    }
    default:
      return MatrixWrapper::Matrix();
  }
}


/**
 * @brief Destructor (doing nothing).
 * @return
 */
BFL::AnalyticConditionalGaussianDepthMeasurement::
~AnalyticConditionalGaussianDepthMeasurement()
{}


/**
 * @brief Clone function.
 * @return pointer to a new object that is an exact copy of this.
 */
BFL::AnalyticConditionalGaussianDepthMeasurement*
BFL::AnalyticConditionalGaussianDepthMeasurement::Clone() const
{
  return new AnalyticConditionalGaussianDepthMeasurement(*this);
}

