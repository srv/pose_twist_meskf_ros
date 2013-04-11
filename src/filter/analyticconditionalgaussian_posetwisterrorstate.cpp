/**
 * @file
 * @author Joan Pau Beltran
 * @brief Conditional distribution for pose-twist error state (presentation).
 *
 * This is the needed implementation of the pose-twist error state conditional
 * distribution with additive noise.
 *
 * Actually it is a hack because BFL does not implement error state filters
 * directly, and an ordinary ExtendeKalmanFilter is used to deal
 * only with the error. The problem is how to implement the conditional pdf
 * since the state transition depends on the values of the nominal state.
 *
 * The adopted solution is this:
 *
 * - The filter only takes care of the error-state update and correct.
 *
 * - The error state conditional pdf keeps the nominal state internally.
 *
 * - Nominal state update is performed during the error state update.
 *   Before an error state system update, this nominal state must be updated
 *   and the new nominal values must be used to update the error state.
 *
 * - Reset is performed manually (correcting the nominal state
 *   with the filter's post mean and resetting it to zero).
 *
 * The variable time step problem is solved
 * passing the time step as an element of the input.
 *
 * The details about the model may be found in:
 * - Joan Sola,
 *   Quaternion kinematics for the error-state KF
 *   http://homepages.laas.fr/jsola/JoanSola/objectes/notes/kinematics.pdf
 *
 * - Nikolas Trawny and Stergios I. Roumeliotis,
 *   Indirect Kalman Filter for 3D Attitude Estimation
 *   http://www.asl.ethz.ch/education/master/aircraft/2010_L9_Quaternions_3D.pdf
 *
 * - Ales Ude,
 *   Filtering in a Unit Quaternion Space for Model-Based Object Tracking
 *   http://www.cns.atr.jp/~aude/publications/ras99.pdf
 *
 * - F. Landis Markley,
 *   Attitude Error Representations for Kalman Filtering
 *   http://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20020060647_2002098710.pdf
 *
 */

#include "analyticconditionalgaussian_posetwisterrorstate.h"
#include "error_state_vector.h"
#include "input_vector.h"

// Auxiliary function prototypes:
void performFullUpdate(const Eigen::Vector3d& g_vect,
                       const double& acc_var,
                       const double& gyro_var,
                       const double& acc_bias_var,
                       const double& gyro_drift_var,
                       const pose_twist_meskf::InputVector& input,
                       pose_twist_meskf::ErrorStateVector& error_state,
                       pose_twist_meskf::NominalStateVector& nominal_state,
                       MatrixWrapper::Matrix& error_state_derivative,
                       MatrixWrapper::SymmetricMatrix& noise_covariance);
Eigen::Matrix3d skew(const Eigen::Vector3d& v);
//! Tolerance for small angle approximations.
const double ANGULAR_RATE_EPSILON = 1E-3;



/**
 * @brief Default constructor.
 * @param noise additive system noise pdf.
 * @return
 */
BFL::AnalyticConditionalGaussianPoseTwistErrorState::
AnalyticConditionalGaussianPoseTwistErrorState(const double& acc_var,
                                               const double& gyro_var,
                                               const double& acc_bias_var,
                                               const double& gyro_drift_var,
                                               const Eigen::Vector3d& gravity)
: AnalyticConditionalGaussian(pose_twist_meskf::ErrorStateVector::DIMENSION,2) // 2 conditional arguments
, error_state_derivative_(pose_twist_meskf::ErrorStateVector::DIMENSION,
                          pose_twist_meskf::ErrorStateVector::DIMENSION)
, noise_covariance_(pose_twist_meskf::ErrorStateVector::DIMENSION,
                    pose_twist_meskf::ErrorStateVector::DIMENSION)
, ACC_VAR_(acc_var)
, GYRO_VAR_(gyro_var)
, ACC_BIAS_VAR_(acc_bias_var)
, GYRO_DRIFT_VAR_(gyro_drift_var)
, G_VECT(gravity)
{}

/**
 * @brief Destructor (doing nothing).
 * @return
 */
BFL::AnalyticConditionalGaussianPoseTwistErrorState::
~AnalyticConditionalGaussianPoseTwistErrorState()
{}


/**
 * @brief Correct nominal state with given error.
 *
 * The correction is performed according to these rules:
 * - position:
 *      p_t = p + dp
 * - orientation:
 *      q_t = q * exp(1/2*dq)  with exp(1/2*dq) = [ cos(1/2*|dq|) , sin(1/2*|dq|) * dq/|dq| ]
 * - linear velocity:
 *      v_t = v + dv
 * - accelerometer bias:
 *      b_t = b + db
 * - gyroscope drift:
 *      d_t = d + dd
 * - linear acceleration:
 *      a_t = R'g - a_s + b + db
 * - angular velocity:
 *      w_t = w_s - d_t = w_s - (d + dd) = w_s - d - dd
 *
 */
void BFL::AnalyticConditionalGaussianPoseTwistErrorState::
CorrectNominalState(const MatrixWrapper::ColumnVector& e)
{
  pose_twist_meskf::ErrorStateVector error;
  error.fromVector(e);

  // Correct the nominal state.
  nominal_state_.position_ += error.d_position_;
  nominal_state_.lin_vel_ += error.d_lin_vel_;
 /* cout << "nominal_state_.orientation_ avans acumular error " << ' ' <<nominal_state_.orientation_ .x() ;
  cout << "nominal_state_.orientation_ avans acumular error " << ' ' <<nominal_state_.orientation_ .y() ;
  cout << "nominal_state_.orientation_ avans acumular error " << ' ' <<nominal_state_.orientation_ .z() ;
  cout << "lineal velocity & error lin. vel" << ' ' << error.d_lin_vel_ ;*/


  Eigen::Quaterniond q;

  if (error.d_orientation_(0)== 0.0 && error.d_orientation_(1)== 0.0 && error.d_orientation_(2)== 0.0 )
  {
  q.x()=0;
  q.y()= 0;
  q.z() = 0;
  q.w() = 1;
  nominal_state_.orientation_ *= q;
  }else{
     nominal_state_.orientation_ *= Eigen::Quaterniond(Eigen::AngleAxisd(error.d_orientation_.norm(), error.d_orientation_.normalized()));
  }

//  cout << " error orientation " << ' ' << error.d_orientation_ ;
//  cout << "nominal_state_.orientation_  " << ' ' <<nominal_state_.orientation_ .x() ;
  // orientation_.normalize();
  nominal_state_.acc_bias_ += error.d_acc_bias_;
  nominal_state_.gyro_drift_ += error.d_gyro_drift_; // --> d_t = d + dd

//  cout << "error gyro_drift" << error.d_gyro_drift_ << "\n";
//  cout << "error lineal acc" << error.d_acc_bias_ << "\n";
//  cout << "error pose" << error.d_position_ << "\n";

  nominal_state_.lin_acc_ += error.d_acc_bias_;
  nominal_state_.ang_vel_ -= error.d_gyro_drift_; // w_t = w_s - d_t = w_s - (d + dd) = w_s - d - dd // fbf 26-06-2012 changed + by -
}


/**
 * @brief Set the current nominal state.
 * @param x nominal state vector.
 */
void
BFL::AnalyticConditionalGaussianPoseTwistErrorState::
NominalStateSet(const MatrixWrapper::ColumnVector& x)
{
  nominal_state_.fromVector(x);
}


/**
 * @brief Get the current nominal state.
 * @return current nominal state vector.
 */
MatrixWrapper::ColumnVector
BFL::AnalyticConditionalGaussianPoseTwistErrorState::NominalStateGet()
{
	return nominal_state_.toVector();
}

/**
 * @brief Update the error state and implicitly all the elements of the pdf.
 * @return next predicted error state.
 *
 * It calls performFullUpdate() to update the nominal state, the error state,
 * the Jacobian of the state transition function and the noise covariance
 * matrix from current values of nominal state and given input.
 */
MatrixWrapper::ColumnVector
BFL::AnalyticConditionalGaussianPoseTwistErrorState::ExpectedValueGet() const
{
  pose_twist_meskf::ErrorStateVector error_state;
  error_state.fromVector(ConditionalArgumentGet(0));
  pose_twist_meskf::InputVector input;
  input.fromVector(ConditionalArgumentGet(1));
  performFullUpdate(G_VECT, ACC_VAR_, GYRO_VAR_, ACC_BIAS_VAR_, GYRO_DRIFT_VAR_,
                    input, error_state, nominal_state_,
                    error_state_derivative_, noise_covariance_);
  return error_state.toVector();
}

/**
 * @brief Get the Jacobian of the state transition function.
 * @param i conditional argument index.
 * @return the state transition matrix.
 *
 * It simply returns the cached value of the state transition Jacobian matrix
 * computed by performFullUpdate() invoked in last ExpectedValueGet() call.
 */
MatrixWrapper::Matrix
BFL::AnalyticConditionalGaussianPoseTwistErrorState::dfGet(unsigned int i) const
{
  /*
  if(i!=0)
  {
    cerr << "The pdf is not implemented for the" << i << "th conditional argument\n";
    exit(-BFL_ERRMISUSE);
  }
  */
  assert(i==0);
  return error_state_derivative_;
}

/**
 * @brief Get the noise covariance.
 * @return noise covariance.
 *
 * It simply returns the cached value of the noise covariance matrix computed
 * by performFullUpdate() invoked in last ExpectedValueGet() call.
 */
MatrixWrapper::SymmetricMatrix
BFL::AnalyticConditionalGaussianPoseTwistErrorState::CovarianceGet() const
{
  return noise_covariance_;
}


/**
 * @brief Update all entities in the pdf.
 * @param g_vect standard gravity vector in reference frame.
 * @param acc_var accelerometers' variance.
 * @param gyro_var gyroscopes' variance.
 * @param acc_bias_var accelerometer bias' variance
 * @param gyro_drift_var gyroscope drifts' variance
 * @param error_state error state struct to be updated.
 * @param nominal_state nominal state struct to be updated.
 * @param error_state_derivative error error state derivative of the transition function to be computed.
 * @param noise_covariance noise covariance matrix to be computed.
 *
 * This function gathers in a single place the update of all elements in the pdf.
 * Is needed because the computation of the derivative depends on values of the
 * nominal state, and the shipped filter implementation calls first
 * ExpectedValueGet() and after dFGet() and CovarianceGet().
 * Hence, if the nominal state is updated in the first function call, the other
 * elements can not be computed properly.
 * The solution is to compute all the elements in the same place and cache them,
 * letting the mentioned functions just return the cached values.
 *
 * The nominal state update is performed according to these rules:
 * - linear acceleration:
 *      a = R'g - a_s + b
 * - angular velocity:
 *      w = w_s - d   (angle turned = dt*|w| , axis turned = w / |w|)
 * - position:
 *      p_next = p + dt*R*v + 0.5*dt^2*[g - R*(a_s-b)] = p + dt*R*v + 0.5*dt^2*R*a
 * - orientation:
 *      q_next = q * q(angle turned, axis turned)
 * - linear velocity:
 *      v_next = v + dt*(R'*g-a_s+b) = v + dt*a
 * - accelerometer bias:
 *      b_next = b
 * - gyroscope drift:
 *      d_next = d
 *
 * The error state update may be omitted since it will lead to the same zero
 * error state (provided that the previous error is zero, which is always the
 * case because of the filter reset). Anyway, these are the rules:
 * - position:
 *      dp_next = dp + dt*R*dv + (-dt*R*[v]_x + 0.5*dt^2*R*[a_s - b]_x)*dq + 0.5*dt^2*R*db
 * - orientation:
 *      dq_next = R(angle turned, axis turned)'*dq
 *                + { -dt*I + (1-cos(dt*|w|))/|w|^2*[w]_x - (dt*|w|-sin(dt*|w|))/|w|^3*[w]_x^2 }*db
 * - linear velocity:
 *      dv_next = dv + dt*[R'*g]_x*dq + dt*db
 * - accelerometer bias:
 *      db_next = db
 * - gyroscope drift:
 *      dd_next = dd
 *
 * The Jacobian matrix is build according to the above equations
 * Some values are approximated by simpler expressions when the angular rate
 * is smaller than ANGULAR_RATE_EPSILON.
 *
 * The gyroscopes' output noise and drift noise
 * are supposed to be independent, gaussian, and isotropic.
 * Then, the covariance may be computed according to the second citation.
 * Accelerometers' output noise and bias noise
 * are supposed to be independent, gaussian, and isotropic too.
 * But the covariance is only an approximation due to the system model
 * complexity.
 */
void performFullUpdate(const Eigen::Vector3d& g_vect,
                       const double& acc_var,
                       const double& gyro_var,
                       const double& acc_bias_var,
                       const double& gyro_drift_var,
                       const pose_twist_meskf::InputVector& input,
                       pose_twist_meskf::ErrorStateVector& error_state,
                       pose_twist_meskf::NominalStateVector& nominal_state,
                       MatrixWrapper::Matrix& error_state_derivative,
                       MatrixWrapper::SymmetricMatrix& noise_covariance)
{
  Eigen::Matrix3d R = nominal_state.orientation_.toRotationMatrix();

  nominal_state.lin_acc_ = R.transpose()*g_vect - input.lin_acc_ + nominal_state.acc_bias_; // fugu-c
  nominal_state.ang_vel_ = input.ang_vel_ - nominal_state.gyro_drift_;

  const double dt = input.time_incr_;
  const double rate = nominal_state.ang_vel_.norm();
  const Eigen::Vector3d axis = nominal_state.ang_vel_.normalized();
  const double angle = dt*rate;
  Eigen::AngleAxisd angle_axis(angle,axis); // axis is the angular velocity direction & angle
  // is the the norm of the angular velocity*dt

  const double dt2 = pow(dt,2);
  const double dt3 = pow(dt,3);
  const double rate2 = pow(rate,2);
  const double rate3 = pow(rate,3);

  const double cos_angle = cos(angle);
  const double sin_angle = sin(angle);

  const double dt4 = pow(dt,4);
  const double dt5 = pow(dt,5);
  const double rate4 = pow(rate,4);
  const double rate5 = pow(rate,5);
  const double angle2 = pow(angle,2);
  const double angle3 = pow(angle,3);

  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d S = skew(nominal_state.ang_vel_);
  const Eigen::Matrix3d S2 = S*S;
  const Eigen::Matrix3d TH = angle_axis.toRotationMatrix().transpose();
                        // = I - sin_angle/rate*S + (1-cos_angle)/rate2*S2
                        // ~ I - dt*S + 0.5*dt2*S2
  const Eigen::Matrix3d PS =
      ( rate < ANGULAR_RATE_EPSILON )
      ? ( - dt*I + dt2/2.0*S - dt3/6.0*S2 )
      : ( - dt*I + ((1-cos_angle)/rate2)*S - ((angle-sin_angle)/rate3)*S2 );

  // Update the noise covariance matrix.
  const Eigen::Matrix3d Qpp = 0.5*dt2*acc_var*I;
  const Eigen::Matrix3d Qvv = dt*acc_var*I;
  const Eigen::Matrix3d Qqq = dt*gyro_var*I
                            + gyro_drift_var*
                            (
                              ( rate < ANGULAR_RATE_EPSILON )
                              ? ( dt3/3.0*I + dt5/60.0*S2 )
                              : ( dt3/3.0*I + (angle3/3.0+2.0*(sin_angle-angle))/rate5*S2 )
                            );
  const Eigen::Matrix3d Qqd = -gyro_drift_var*
                            (
                              ( rate < ANGULAR_RATE_EPSILON )
                              ? ( dt2/2.0*I - dt3/6.0*S + dt4/24.0*S2)
                              : ( dt2/2.0*I - (angle-sin_angle)/rate3*S + (angle2/2.0+cos_angle-1.0)/rate4*S2 )
                            );
  const Eigen::Matrix3d Qbb = dt*acc_bias_var*I;
  const Eigen::Matrix3d Qdd = dt*gyro_drift_var*I;

  noise_covariance = 0.0;
  for (int i=0; i<3; i++)
  {
    for (int j=0; j<3; j++)
    {
      // Position:
      noise_covariance(pose_twist_meskf::ErrorStateVector::D_POSITION_X + i,
                       pose_twist_meskf::ErrorStateVector::D_POSITION_X + j) = Qpp(i,j);
      // Velocity:
      noise_covariance(pose_twist_meskf::ErrorStateVector::D_LIN_VEL_X + i,
                       pose_twist_meskf::ErrorStateVector::D_LIN_VEL_X + j) = Qvv(i,j);
      // Accelerometers' bias:
      noise_covariance(pose_twist_meskf::ErrorStateVector::D_ACC_BIAS_X + i,
                       pose_twist_meskf::ErrorStateVector::D_ACC_BIAS_X + j) = Qbb(i,j);
      // Orientation:
      noise_covariance(pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X + i,
                       pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X + j) = Qqq(i,j);
      noise_covariance(pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X + i,
                       pose_twist_meskf::ErrorStateVector::D_GYRO_DRIFT_X + j) = Qqd(i,j);
      // Gyroscopes' drift:
      noise_covariance(pose_twist_meskf::ErrorStateVector::D_GYRO_DRIFT_X + i,
                       pose_twist_meskf::ErrorStateVector::D_GYRO_DRIFT_X + j) = Qdd(i,j);
      noise_covariance(pose_twist_meskf::ErrorStateVector::D_GYRO_DRIFT_X + i,
                       pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X + j) = Qqd(j,i);
    }
  }

  // Update the error state derivative matrix.
  const Eigen::Matrix3d dp_dp = I;
  const Eigen::Matrix3d dp_dq = - dt*R*skew(nominal_state.lin_vel_)
                                + 0.5*dt2*R*skew(input.lin_acc_ - nominal_state.acc_bias_);
  const Eigen::Matrix3d dp_dv = dt*R;
  const Eigen::Matrix3d dp_db = 0.5*dt2*R;
  const Eigen::Matrix3d dv_dv = I;
  const Eigen::Matrix3d dv_dq = dt*skew(R.transpose()*g_vect);
  const Eigen::Matrix3d dv_db = dt*I;
  const Eigen::Matrix3d db_db = I;
  const Eigen::Matrix3d dq_dq = TH;
  const Eigen::Matrix3d dq_dd = PS;
  const Eigen::Matrix3d dd_dd = I;

  error_state_derivative = 0.0;
  for (int i=0; i<3; i++)
  {
    for (int j=0; j<3; j++)
    {
      // Position:
      error_state_derivative(pose_twist_meskf::ErrorStateVector::D_POSITION_X + i,
                             pose_twist_meskf::ErrorStateVector::D_POSITION_X + j) = dp_dp(i,j);
      error_state_derivative(pose_twist_meskf::ErrorStateVector::D_POSITION_X + i,
                             pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X + j) = dp_dq(i,j);
      error_state_derivative(pose_twist_meskf::ErrorStateVector::D_POSITION_X + i,
                             pose_twist_meskf::ErrorStateVector::D_LIN_VEL_X + j) = dp_dv(i,j);
      error_state_derivative(pose_twist_meskf::ErrorStateVector::D_POSITION_X + i,
                             pose_twist_meskf::ErrorStateVector::D_ACC_BIAS_X + j) = dp_db(i,j);
      // Velocity:
      error_state_derivative(pose_twist_meskf::ErrorStateVector::D_LIN_VEL_X + i,
                             pose_twist_meskf::ErrorStateVector::D_LIN_VEL_X + j) = dv_dv(i,j);
      error_state_derivative(pose_twist_meskf::ErrorStateVector::D_LIN_VEL_X + i,
                             pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X + j) = dv_dq(i,j);
      error_state_derivative(pose_twist_meskf::ErrorStateVector::D_LIN_VEL_X + i,
                             pose_twist_meskf::ErrorStateVector::D_ACC_BIAS_X + j) = dv_db(i,j);
      // Accelerometers' bias:
      error_state_derivative(pose_twist_meskf::ErrorStateVector::D_ACC_BIAS_X + i,
                             pose_twist_meskf::ErrorStateVector::D_ACC_BIAS_X + j) = db_db(i,j);
     // Orientation:
      error_state_derivative(pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X + i,
                             pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X + j) = dq_dq(i,j);
      error_state_derivative(pose_twist_meskf::ErrorStateVector::D_ORIENTATION_X + i,
                             pose_twist_meskf::ErrorStateVector::D_GYRO_DRIFT_X + j) = dq_dd(i,j);
      // Gyroscopes' drift:
      error_state_derivative(pose_twist_meskf::ErrorStateVector::D_GYRO_DRIFT_X + i,
                             pose_twist_meskf::ErrorStateVector::D_GYRO_DRIFT_X + j) = dd_dd(i,j);
    }
  }

  // Update the error state.
  // If the error state is zero (it should be because of the reset)
  // the update will not change it.
  // However these are the equations:
  error_state.d_position_ += dt*R*error_state.d_lin_vel_
                          - dt*R*nominal_state.lin_vel_.cross(error_state.d_orientation_)
                          + 0.5*dt2*R*(input.lin_acc_ - nominal_state.acc_bias_).cross(error_state.d_orientation_)
                          + 0.5*dt2*R*error_state.d_acc_bias_;
  error_state.d_lin_vel_ += dt*(R.transpose()*g_vect).cross(error_state.d_orientation_)
                         + dt*error_state.d_acc_bias_;
  error_state.d_orientation_ = TH*error_state.d_orientation_ + PS*error_state.d_gyro_drift_;
  // Gyroscopes' drift and accelerometers' bias are constant,
  // so their errors keep their value.

  // Update the nominal state following the above rules.
  nominal_state.position_ += R*(dt*nominal_state.lin_vel_ + 0.5*dt2*nominal_state.lin_acc_);
  nominal_state.lin_vel_ += dt*nominal_state.lin_acc_;
  nominal_state.orientation_*= Eigen::Quaterniond(angle_axis);
  // nominal_state_.orientation_.normalize();
  // Gyroscopes' drift and accelerometers' bias are constant,
  // so they keep their values.
}


/**
 * @brief Build the skew-symmetric matrix that computes the cross product.
 * @param v left vector in cross product.
 * @return matrix $M$ such that $M w = v \cross w$ for all $w$.
 */
Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d m;
  m <<  0.0 , -v(2),  v(1),
        v(2),  0.0 , -v(0),
       -v(1),  v(0),  0.0 ;
  return m;
}
