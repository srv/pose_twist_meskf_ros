/**
 * @file
 * @author Joan Pau Beltran
 * @brief Conditional distribution for measurement from visual sensors (implementation).
 */

#include "analyticconditionalgaussian_visualmeasurement.h"
#include "nominal_state_vector.h"
#include "error_state_vector.h"
#include "visual_measurement_vector.h"
#include "visual_measurement_error_vector.h"


/**
 * @brief Constructor not setting measurement noise.
 * @param additive_noise pdf representing the additive Gaussian uncertainty.
 * @return
 * Additive noise parameters must be set later with AdditiveNoiseSigmaSet() and
 * AdditiveNoiseMuSet().
 */
BFL::AnalyticConditionalGaussianVisualMeasurement::
AnalyticConditionalGaussianVisualMeasurement()
: AnalyticConditionalGaussianErrorMeasurement(pose_twist_meskf::VisualMeasurementErrorVector::DIMENSION)
{}


/**
 * @brief Compute error measurements between visual measurements and nominal state.
 * @param z visual measurement vector.
 * @param x nominal state vector.
 * @return measurement error vector.
 */
MatrixWrapper::ColumnVector
BFL::AnalyticConditionalGaussianVisualMeasurement::ErrorMeasurement(
  const MatrixWrapper::ColumnVector& z,
  const MatrixWrapper::ColumnVector& x) const
{
  pose_twist_meskf::VisualMeasurementVector measurement;
  measurement.fromVector(z);
  pose_twist_meskf::NominalStateVector nominal_state;
  nominal_state.fromVector(x);
  pose_twist_meskf::VisualMeasurementErrorVector error;
  error.d_position_ = measurement.position_ - nominal_state.position_;
  Eigen::AngleAxisd aa(nominal_state.orientation_.inverse() * measurement.orientation_);
  error.d_orientation_ = aa.angle() * aa.axis();
  error.d_lin_vel_ = measurement.lin_vel_ - nominal_state.lin_vel_;
  error.d_acc_bias_ = measurement.lin_acc_ - nominal_state.lin_acc_;
//Gyrometer drift and accelerometer bias are tricky because they do not appear explicitly in the nominal state,  
// update rules result from the following equations: accelerometer bias--> a_t=R'g-(a_s-b_t)=R'g-(a_s-(b+db))=R'g-(a_s-b)+db
// gyros drift --> w_t=w_s-d_t = w_s-(d+dd)=w_s-d-dd .
 // error.d_gyro_drift_ = measurement.ang_vel_ - nominal_state.ang_vel_;
  error.d_gyro_drift_ = nominal_state.ang_vel_ - measurement.ang_vel_ ;
  /*MatrixWrapper::ColumnVector e(pose_twist_meskf::ErrorStateVector::DIMENSION);
  error.toVector(e);
  return e;*/ return error.toVector();
}


/**
 * @brief Compute expected visual measurement error.
 * @return  expected measurement error conditioned to nominal and error state.
 *
 * The first conditional argument (index 0) is the error state.
 * The second conditional argument (index 1) must be the nominal state.
 */
MatrixWrapper::ColumnVector BFL::AnalyticConditionalGaussianVisualMeasurement::
ExpectedValueGet() const
{
  /*pose_twist_meskf::ErrorStateVector error_state;
  return ConditionalArgumentGet(0);*/
	pose_twist_meskf::ErrorStateVector error_state;
	  error_state.fromVector(ConditionalArgumentGet(0));
	  pose_twist_meskf::VisualMeasurementErrorVector measurement_error;
	  measurement_error.d_lin_vel_ = error_state.d_lin_vel_;
	  measurement_error.d_gyro_drift_ = error_state.d_gyro_drift_;
	//  MatrixWrapper::ColumnVector e(pose_twist_meskf::ErrorStateVector::DIMENSION);
	  return measurement_error.toVector();
	 // return e;
}


/**
 * @brief Compute derivative of the conditional measurement.
 * @param i index of conditional variable to use for partial derivation.
 * @return partial derivative with respect to conditional variable i.
 *
 * This function should be called only for the first conditional argument
 * (index 0) which is the current error state.
 * The second conditional argument (index 1) must be the nominal state.
 */
MatrixWrapper::Matrix
BFL::AnalyticConditionalGaussianVisualMeasurement::dfGet(unsigned int i) const
{
  switch(i)
  {
    case 0:
    {
      // const MatrixWrapper x = ConditionalArgumentGet(1);
      MatrixWrapper::Matrix H(pose_twist_meskf::VisualMeasurementErrorVector::DIMENSION,
                              pose_twist_meskf::ErrorStateVector::DIMENSION);
      H = 0.0;
      for (int i=0; i<3; i++)
      {
        // Position:
        H(pose_twist_meskf::VisualMeasurementErrorVector::D_POSITION_X + i,
          pose_twist_meskf::ErrorStateVector::D_POSITION_X + i) = 1.0;
        // Velocity:
        H(pose_twist_meskf::VisualMeasurementErrorVector::D_LIN_VEL_X + i,
          pose_twist_meskf::ErrorStateVector::D_LIN_VEL_X + i) = 1.0;
        // Accelerometers' bias:
        H(pose_twist_meskf::VisualMeasurementErrorVector::D_ACC_BIAS_X + i,
          pose_twist_meskf::ErrorStateVector::D_ACC_BIAS_X + i) = 1.0; // meas_acc - input_acc = db - R*skew(G_VEC)*dq.
        // Orientation:
        H(pose_twist_meskf::VisualMeasurementErrorVector::D_ORIENTATION_X + i,
          pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X + i) = 1.0;
        // Gyroscopes' bias:
        H(pose_twist_meskf::VisualMeasurementErrorVector::D_GYRO_DRIFT_X + i,
          pose_twist_meskf::ErrorStateVector::D_GYRO_DRIFT_X + i) = 1.0;
      }
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
BFL::AnalyticConditionalGaussianVisualMeasurement::
~AnalyticConditionalGaussianVisualMeasurement()
{}


/**
 * @brief Clone function.
 * @return pointer to a new object that is an exact copy of this.
 */
BFL::AnalyticConditionalGaussianVisualMeasurement*
BFL::AnalyticConditionalGaussianVisualMeasurement::Clone() const
{
  return new AnalyticConditionalGaussianVisualMeasurement(*this);
}
