#pragma once

#include "Eigen/Core"

namespace frc1678 {

namespace muan {

namespace util {

namespace frames {

// Z is up, X is forward, Y is left
class ReferenceFrame {
 public:
  ReferenceFrame(ReferenceFrame const* const parent, Eigen::Vector3d const& translation,
                 Eigen::Vector3d const& rotation);

  Eigen::Vector3d ToInertialCoordinates(Eigen::Vector3d const& in_this_frame) const;
  Eigen::Vector3d FromInertialCoordinates(Eigen::Vector3d const& in_inertial_frame) const;

  static ReferenceFrame InertialFrame();

  void Set(Eigen::Vector3d const& translation, Eigen::Vector3d const& rotation);

  bool is_inertial() const;

  void SetVelocity(Eigen::Vector3d const& velocity, Eigen::Vector3d const& angular_velocity);

  Eigen::Vector3d ToInertialTranslationalVelocity(Eigen::Vector3d const& in_this_frame);
  Eigen::Vector3d ToInertialAngularVelocity();

 private:
  explicit ReferenceFrame(ReferenceFrame const* const parent, Eigen::Matrix4d const& transform_matrix);

  Eigen::Matrix4d GetTransformFromInertial() const;

  inline static Eigen::Vector4d ToHomogenousVector(Eigen::Vector3d vec);
  inline static Eigen::Vector3d FromHomogenousVector(Eigen::Vector4d vec);
  inline static Eigen::Vector4d ToHomogenousVelocityVector(Eigen::Vector3d vec);
  inline static Eigen::Vector3d FromHomogenousVelocityVector(Eigen::Vector4d vec);

  Eigen::Matrix4d homogenous_transform_;
  // nullptr if it is an inertial frame
  ReferenceFrame const* const parent_;

  // Relative to parent, expressed in parent coordinates
  Eigen::Vector3d frame_velocity_;
  Eigen::Vector3d frame_angular_velocity_;
};

} /* frames */

} /* util */

} /* muan */

} /* frc1678 */
