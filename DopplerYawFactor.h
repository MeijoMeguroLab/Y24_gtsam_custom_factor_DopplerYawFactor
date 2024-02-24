#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <Eigen/Geometry>

namespace gtsam {

/**
 * Prior on yaw orientation based on Doppler effect measurements.
 * @ingroup navigation
 */
class GTSAM_EXPORT DopplerYawFactor: public NoiseModelFactor1<Pose3> {

private:

  typedef NoiseModelFactor1<Pose3> Base;
  double yawMeasurement_; 
  //Eigen::Quaterniond yawMeasurement_; ///< Yaw measurement in radians

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<DopplerYawFactor> shared_ptr;

  /// Typedef to this class
  typedef DopplerYawFactor This;

  /** default constructor - only use for serialization */
  //DopplerYawFactor(): yawMeasurement_(Eigen::Quaterniond::Identity()) {}
  DopplerYawFactor(): yawMeasurement_(0.0) {}


  ~DopplerYawFactor() override {}

  /**
   * @brief Constructor from a yaw measurement based on Doppler effect.
   * @param key of the Pose3 variable that will be constrained
   * @param yawMeasurement measurement in radians
   * @param model Gaussian noise model
   */
  // DopplerYawFactor(Key key, const Eigen::Quaterniond& yawMeasurement, const SharedNoiseModel& model) :
  //     Base(model, key), yawMeasurement_(yawMeasurement) {
  // }

  DopplerYawFactor(Key key, const double yawMeasurement, const SharedNoiseModel& model) :
      Base(model, key), yawMeasurement_(yawMeasurement) {
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  /// equals
  bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override;

  /// vector of errors
  Vector evaluateError(const Pose3& p,
      boost::optional<Matrix&> H = boost::none) const override;
      
  inline double measurementIn() const {
    return yawMeasurement_;
  }
  // inline Eigen::Quaterniond measurementIn() const {
  //   return yawMeasurement_;
  // }

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar
        & boost::serialization::make_nvp("NoiseModelFactor1",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(yawMeasurement_);
  }
//   template<class ARCHIVE>
//   void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
//     ar & boost::serialization::make_nvp("NoiseModelFactor1",
//         boost::serialization::base_object<Base>(*this));
//     ar & BOOST_SERIALIZATION_NVP(yawMeasurement_);
//   }
};

} /// namespace gtsam
