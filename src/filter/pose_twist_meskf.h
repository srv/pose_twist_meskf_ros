/**
 * @file
 * @brief Pose twist error state extended Kalman filter
 * with multiplicative orientation error.
 *
 * The filter is implemented using the Bayesian Filter Library.
 */

#ifndef POSE_TWIST_MESKF_H
#define POSE_TWIST_MESKF_H

#include <queue>
#include <wrappers/matrix/matrix_wrapper.h>
#include <wrappers/matrix/vector_wrapper.h>
#include <model/analyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include "analyticconditionalgaussian_posetwisterrorstate.h"
#include "analyticconditionalgaussian_errormeasurement.h"
#include "extendedkalmanfilter_resetcapable.h"

namespace pose_twist_meskf
{

/**
 * @brief Multiplicative error-state Kalman filter class.
 *
 * The estimator has one internal queue for the inputs,
 * and one internal queue for each measurement type.
 * Every input and measurement is added to the respective queue
 * with the functions addInput() and addMeasurement().
 * The input and measurement processing is delayed
 * until a call to the function update(), when all inputs and measurements
 * are processed sequentially according to its time stamp.
 * To use the class first set up the system model with setUpSystem() and
 * then set the the initial state with initialize().
 *
 * Details about the dynamic model and the measurement models may be found
 * in the documentation of the respective conditional pdf implementation.
 */
class PoseTwistMESKF
{
public:
  typedef MatrixWrapper::ColumnVector Vector;
  typedef MatrixWrapper::SymmetricMatrix SymmetricMatrix;
  typedef double TimeStamp;
  typedef int MeasurementIndex;

  enum MeasurementType
  {
    VISUAL, DEPTH
  };

  static const int NUM_MEASUREMENT_TYPES = 2; // VISUAL AND PRESSURE SENSOR

  PoseTwistMESKF();
  virtual ~PoseTwistMESKF();

  TimeStamp getFilterTime() const;
  SymmetricMatrix getCovariance() const;
  Vector getEstimate() const;
  void getEstimate(Vector& x, SymmetricMatrix& P, TimeStamp& t) const;

  void setUpSystem(const double& acc_var,
                   const double& gyro_var,
                   const double& acc_bias_var,
                   const double& gyro_drift_var,
                   const Eigen::Vector3d& gravity);

  void setUpMeasurementModels();

  void initialize(const Vector& x, const SymmetricMatrix& P, const TimeStamp& t);

  bool addInput(const TimeStamp& t, const Vector& u);
  bool addMeasurement(const MeasurementType& m, const Vector& z,
                      const SymmetricMatrix Q, const TimeStamp& t);

  bool update();
  bool updateAll();

private:

  struct Measurement
  {
    TimeStamp t_;
    Vector z_;
    SymmetricMatrix Q_;
    Measurement(const TimeStamp& t, const Vector& z, const SymmetricMatrix& Q)
    :  t_(t), z_(z), Q_(Q)
    {}
    bool operator<(const Measurement& r) const {return t_ > r.t_;}
  };

  struct Input
  {
    TimeStamp t_;
    Vector u_;
    Input(const TimeStamp& t, const Vector& u)
    :  t_(t), u_(u)
    {}
    bool operator<(const Input& r) const {return t_ > r.t_;}
  };

  BFL::ExtendedKalmanFilterResetCapable*               filter_;
  BFL::AnalyticSystemModelGaussianUncertainty*         system_model_;
  BFL::AnalyticConditionalGaussianPoseTwistErrorState* system_pdf_;
  BFL::Gaussian*                                       system_prior_;
  std::priority_queue<Input>                           input_queue_;

  std::vector< BFL::AnalyticConditionalGaussianErrorMeasurement* > measurement_pdfs_;
  std::vector< BFL::AnalyticMeasurementModelGaussianUncertainty* > measurement_models_;
  std::vector< std::priority_queue<Measurement> >                  measurement_queues_;

  Vector filter_input_;
  TimeStamp filter_time_;


  bool updateFilterSys(const Input& u);
  bool updateFilterMeas(const MeasurementIndex& i, const Measurement& m);
};

} // namespace

#endif // POSE_TWIST_MESKF_H
