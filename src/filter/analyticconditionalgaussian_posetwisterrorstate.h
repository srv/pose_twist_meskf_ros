/**
 * @file
 * @author Joan Pau Beltran
 * @brief Conditional distribution for pose-twist error state (presentation).
 */

#ifndef ANALYTICCONDITIONALGAUSSIAN_POSETWISTERRORSTATE_H
#define ANALYTICCONDITIONALGAUSSIAN_POSETWISTERRORSTATE_H

#include <pdf/analyticconditionalgaussian.h>
#include <Eigen/Dense>
#include "nominal_state_vector.h"

namespace BFL
{

/**
 * @brief Pose-twist error state conditional distribution with additive noise.
 *
 * Actually it is a hack because BFL does not implement error state filters
 * directly, and an ordinary ExtendeKalmanFilter is used to deal
 * only with the error state. The problem is how to implement the conditional
 * since the state transition function and its derivative depend on the values
 * of the nominal state.
 *
 * The adopted solution is this:
 *
 * - The filter only takes care of the error-state update and correct.
 *
 * - The error state conditional pdf keeps the nominal state internally.
 *
 * - On an error state system update, the state transition Jacobian matrix
 *   and the nominal state must be updated.
 *
 * - Reset is performed manually (correcting the nominal state
 *   with the filter's post mean and resetting it to zero).
 *
 * The variable time step problem is solved passing the time step as an element
 * of the input.
 *
 * The problem of the nominal state update effects on the Jacobian matrix is
 * solved doing the updates of all pdf entities in a single function
 * and caching the results. This works as long as the filter calls the functions
 * in the proper order during system update (actually calling ExpectedValueGet()
 * first is sufficient).
 */
class AnalyticConditionalGaussianPoseTwistErrorState
: public BFL::AnalyticConditionalGaussian
{
public:
  // Default constructor specifying system noise.
  AnalyticConditionalGaussianPoseTwistErrorState(const double& acc_var,
                                                 const double& gyro_var,
                                                 const double& acc_bias_var,
                                                 const double& gyro_drift_var,
                                                 const Eigen::Vector3d& gravity);

  // Error state filtering specific functions.
  void NominalStateSet(const MatrixWrapper::ColumnVector& x);
  MatrixWrapper::ColumnVector NominalStateGet();
  void CorrectNominalState(const MatrixWrapper::ColumnVector& e);

  // Redefine virtual functions.
  virtual MatrixWrapper::ColumnVector ExpectedValueGet() const;
  virtual MatrixWrapper::Matrix dfGet(unsigned int i) const;
  virtual MatrixWrapper::SymmetricMatrix CovarianceGet() const;

  virtual ~AnalyticConditionalGaussianPoseTwistErrorState();

private:

  // Error state not needed because it is stored in the filter pdf.
  // In addition, the error state is always zero since it is only modified
  // by a measurement update, and after that the reset step transfers
  // its values to the nominal state and sets it to zero again.

  // Nominal state.
  // It is needed here because the error state transition depends on values
  // of its entities.
  // It is updated with every call to the prediction function.
  // It should be mutable because base prediction function is const.
  mutable pose_twist_meskf::NominalStateVector nominal_state_; //!< Nominal state.

  // Derivative of the system function w.r.t. the error state.
  // It is needed because it should be computed from the values of the nominal
  // state, and hence it can not be computed in a separated function
  // after the nominal state update. It is updated in parallel with the nominal
  // state and retrieved when the derivative is requested.
  // It should be mutable because base prediction function is const.
  mutable MatrixWrapper::Matrix error_state_derivative_; //!< Error state derivative.

  // Error state additive noise covariance.
  // Probably it is not needed, since it may be computed in a separate function.
  // However it is useful to compute it with the rest of elements of the pdf
  // and cache it, too, for consistency and to reuse partial computations.
  // It should be mutable because base prediction function is const.
  mutable MatrixWrapper::SymmetricMatrix noise_covariance_; //!< Noise covariance.

  // Sensor variances.
  const double ACC_VAR_;        //!< Accelerometer variance.
  const double GYRO_VAR_;       //!< Gyroscope variance.
  const double ACC_BIAS_VAR_;   //!< Accelerometer variance.
  const double GYRO_DRIFT_VAR_; //!< Gyroscope variance.

  // Gravity constant vector in reference frame.
  const Eigen::Vector3d G_VECT; //!< Standard gravity vector.

};

} // namespace

#endif // ANALYTICCONDITIONALGAUSSIAN_POSETWISTERRORSTATE_H
