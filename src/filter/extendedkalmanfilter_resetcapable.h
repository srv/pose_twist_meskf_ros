#ifndef __EXTENDED_KALMAN_FILTER_RESET_CAPABLE__
#define __EXTENDED_KALMAN_FILTER_RESET_CAPABLE__

#include <bfl/filter/extendedkalmanfilter.h>

namespace BFL
{

/** This is a class that add the PostMuSet() functionality
    to the BFL Extended Kalman Filter.

    It is needed because of a bug in the implementation of the base class
    member Filter::Reset(), and because the PostMuSet member is protected.

    @see ExtendedKalmanFilter, KalmanFilter, Filter
*/
class ExtendedKalmanFilterResetCapable : public ExtendedKalmanFilter
{
public:
  /** Constructor
      @pre you created the prior
      @param prior pointer to the Gaussian prior density
  */
  ExtendedKalmanFilterResetCapable(Gaussian* prior);

  /// Destructor
  virtual ~ExtendedKalmanFilterResetCapable();

  /// Set covariance of posterior estimate
  void PostSigmaSet( const MatrixWrapper::SymmetricMatrix& s);

  /// Set expected value of posterior estimate
  void PostMuSet( const MatrixWrapper::ColumnVector& c);

};  // class

} // End namespace BFL

#endif // __EXTENDED_KALMAN_FILTER_RESET_CAPABLE__
