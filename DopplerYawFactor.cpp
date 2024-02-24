/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   DopplerYawFactor.cpp
 *  @author Frank Dellaert
 *  @brief  Implementation file for Doppler Yaw factor
 *  @date   January 28, 2014
 **/

#include "DopplerYawFactor.h"

using namespace std;

namespace gtsam {

//***************************************************************************
void DopplerYawFactor::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << (s.empty() ? "" : s + " ") << "DopplerYawFactor on " << keyFormatter(key())
       << "\n";
  cout << "  Yaw measurement: " << yawMeasurement_ << "\n";
  noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool DopplerYawFactor::equals(const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) && fabs(yawMeasurement_ - e->yawMeasurement_) <= tol;
}

// bool DopplerYawFactor::equals(const NonlinearFactor& expected, double tol) const {
//   const This* e = dynamic_cast<const This*>(&expected);
//   if (e == nullptr || !Base::equals(*e, tol)) return false;

//   // Calculate the relative rotation quaternion
//   Eigen::Quaterniond diff = yawMeasurement_.inverse() * e->yawMeasurement_;

//   // Convert the relative rotation to an angle
//   double angle = 2 * acos(diff.w()); // diff.w() is the scalar part of the quaternion

//   // Normalize the angle to be within -pi to pi
//   if (angle > M_PI) angle -= 2 * M_PI;
//   if (angle < -M_PI) angle += 2 * M_PI;

//   // Compare the angle with the tolerance
//   return fabs(angle) <= tol;
// }
//***************************************************************************
Vector DopplerYawFactor::evaluateError(const Pose3& p,
    boost::optional<Matrix&> H) const {
  // Assuming you want to compute the error between yaw measurement and the yaw of Pose3
  // double yawError = p.rotation().yaw() - yawMeasurement_;

  // while (yawError > M_PI) yawError -= 2 * M_PI;
  // while (yawError < -M_PI) yawError += 2 * M_PI;
  // //double yawError = p.rotation().pitch() - yawMeasurement_;
  // Vector1 error(yawError); // Assuming error is just a single dimensional vector
  // // If needed, compute the Jacobian with respect to Pose3 'p'
  // if (H) {
  //       // ヤコビアンの計算
  //       *H = gtsam::Matrix::Zero(1, 6); // 1x6のゼロ行列を作成
  //       (*H)(0, 5) = 1.0; // ヨー角に対する偏微分
  // }

  //Compute the Jacobian if required
  if (H) {
      // Placeholder for Jacobian computation, adjust as necessary for your application
      //*H = gtsam::Matrix::Zero(1, 6);
      //(*H)(0, 5) = 1.0;
      *H = (gtsam::Matrix(1, 6) << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0).finished();
      //Fill in Jacobian computation here
  }

  Rot3 poseRot = p.rotation();

  // Get the yaw angle from the Rot3 object
  double poseYaw = poseRot.yaw();

    // Calculate the yaw error
  double yawError = poseYaw - yawMeasurement_;

    // Normalize the yaw error to be within the range [-π, π]
  // while (yawError > M_PI) yawError -= 2 * M_PI;
  // while (yawError < -M_PI) yawError += 2 * M_PI;

  // yawError = std::fmod(yawError + 3 * M_PI / 2, 2 * M_PI) - M_PI;

  // Return the yaw error as a Vector
  Vector1 error(yawError);

  
  return error;
}

//Vector DopplerYawFactor::evaluateError(const Pose3& p, boost::optional<Matrix&> H) const {
  // Convert Pose3's rotation to a quaternion
  // Eigen::Quaterniond poseQuat = p.rotation().toQuaternion();

  // // Compute the relative rotation quaternion
  // Eigen::Quaterniond relativeQuat = poseQuat.inverse() * yawMeasurement_;

  // // Convert to angle-axis representation
  // Eigen::AngleAxisd angleAxis(relativeQuat);
  // double angleError = angleAxis.angle();

  // // Optionally, normalize the angle to [-pi, pi]
  // while (angleError > M_PI) angleError -= 2 * M_PI;
  // while (angleError < -M_PI) angleError += 2 * M_PI;

  // // Assuming error is just a single dimensional vector
  // Vector1 error(angleError);

  // // If needed, compute the Jacobian with respect to Pose3 'p'
  // if (H) {
  //   // Placeholder for Jacobian computation
  //   *H = gtsam::Matrix::Zero(1, 6);
  //   // ... fill in Jacobian computation here
  // }

  //return error;
//}

// Vector DopplerYawFactor::evaluateError(const Pose3& p, boost::optional<Matrix&> H) const {
//     // QuaternionをRot3に変換
//     Rot3 quat_rot3 = Rot3::Quaternion(yawMeasurement_.w(), yawMeasurement_.x(), yawMeasurement_.y(), yawMeasurement_.z());

//     // 現在の姿勢の回転部分を取得
//     Rot3 pose_rot3 = p.rotation();

//     // 二つの姿勢の差異（誤差）を計算
//     Quaternion pose_quat = pose_rot3.toQuaternion();
//     Quaternion quat_diff = yawMeasurement_.conjugate() * pose_quat;

//     // 誤差をQuaternionのベクトル部分として返す
//     Vector3 error(quat_diff.x(), quat_diff.y(), quat_diff.z(), quat_diff.w());
//   if (H) {
//     // Placeholder for Jacobian computation
//     *H = gtsam::Matrix::Identity(3, 6);
//     (*H)(0, 5) = 1.0;
//     // ... fill in Jacobian computation here
//   }

//   return error;
// }

//***************************************************************************

}/// namespace gtsam
