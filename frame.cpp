#include "frame.h"
#include <cassert>
#include "Eigen/Geometry"

namespace frc1678 {

namespace muan {

namespace util {

namespace frames {

ReferenceFrame::ReferenceFrame(ReferenceFrame const* const parent,
                               Eigen::Vector3d const& translation,
                               Eigen::Vector3d const& rotation)
    : parent_(parent) {
  Set(translation, rotation);
  SetVelocity(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
}

ReferenceFrame::ReferenceFrame(ReferenceFrame const* const parent_ptr,
                               Eigen::Matrix4d const& transform_matrix)
    : homogenous_transform_(transform_matrix), parent_(parent_ptr) {
  SetVelocity(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
}

ReferenceFrame ReferenceFrame::InertialFrame() {
  auto ret = ReferenceFrame(nullptr, Eigen::Matrix4d::Identity());
  ret.cache_valid_ = true;
  return ret;
}

Eigen::Vector3d ReferenceFrame::ToInertialCoordinates(
    Eigen::Vector3d const& in_this_frame) const {
  return FromHomogenousVector(GetTransformFromInertial() *
                              ToHomogenousVector(in_this_frame));
}

Eigen::Vector3d ReferenceFrame::FromInertialCoordinates(
    Eigen::Vector3d const& in_inertial_frame) const {
  return FromHomogenousVector(GetTransformFromInertial().inverse() *
                              ToHomogenousVector(in_inertial_frame));
}

void ReferenceFrame::Set(Eigen::Vector3d const& translation,
                         Eigen::Vector3d const& rotation) {
  cache_valid_ = false;
  Eigen::Matrix4d translation_matrix = Eigen::Matrix4d::Identity(),
                  rotation_matrix = Eigen::Matrix4d::Zero();

  rotation_matrix.block<3, 3>(0, 0) =
      (Eigen::AngleAxisd(rotation(2), Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(rotation(1), Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(rotation(0), Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  rotation_matrix(3, 3) = 1;

  translation_matrix.block<3, 1>(0, 3) = translation;

  // Rotate first, then translate
  homogenous_transform_ = translation_matrix * rotation_matrix;
}

void ReferenceFrame::SetVelocity(Eigen::Vector3d const& velocity,
                                 Eigen::Vector3d const& angular_velocity) {
  frame_velocity_ = velocity;
  frame_angular_velocity_ = angular_velocity;
}

bool ReferenceFrame::is_inertial() const { return parent_ == nullptr; }

bool ReferenceFrame::is_cache_valid() const {
  return is_inertial() ||
         (cache_valid_ &&
          !(parent_->should_recalculate_child(parent_rev_cached_)));
}

// TODO(Kyle) Cache this when it doesn't need to be recalculated
Eigen::Matrix4d ReferenceFrame::GetTransformFromInertial() const {
  if (is_inertial()) {
    cache_valid_ = true;
    cached_inertial_transform_ = homogenous_transform_;
  } else if (!is_cache_valid()) {
    cached_inertial_transform_ =
        parent_->GetTransformFromInertial() * homogenous_transform_;
    parent_rev_cached_ = parent_->current_rev_;
    current_rev_++;
    cache_valid_ = true;
  }

  return cached_inertial_transform_;
}

Eigen::Vector4d ReferenceFrame::ToHomogenousVector(Eigen::Vector3d vec) {
  Eigen::Vector4d ret;
  ret << vec, 1.0;
  return ret;
}

Eigen::Vector3d ReferenceFrame::FromHomogenousVector(Eigen::Vector4d vec) {
  assert(std::abs(vec(3) - 1.0) < 1e-9);
  return vec.block<3, 1>(0, 0);
}

Eigen::Vector4d ReferenceFrame::ToHomogenousVelocityVector(
    Eigen::Vector3d vec) {
  return (Eigen::Vector4d() << vec, 0.0).finished();
}

Eigen::Vector3d ReferenceFrame::FromHomogenousVelocityVector(
    Eigen::Vector4d vec) {
  assert(std::abs(vec(3)) < 1e-9);
  return vec.block<3, 1>(0, 0);
}

Eigen::Vector3d ReferenceFrame::ToInertialTranslationalVelocity(
    Eigen::Vector3d const& in_this_frame) {
  ReferenceFrame const* frame = this;
  Eigen::Vector4d velocity_sum =
                      ToHomogenousVelocityVector(Eigen::Vector3d::Zero()),
                  position_in_frame = ToHomogenousVector(in_this_frame);
  while (frame != nullptr) {
    // When converting translational velocity of a point in a moving frame, add
    // the cross product of the point vector
    // and the frame's angular velocity vector
    velocity_sum = frame->homogenous_transform_ * velocity_sum +
                   ToHomogenousVelocityVector(
                       frame->frame_velocity_ +
                       frame->frame_angular_velocity_.cross(
                           FromHomogenousVector(position_in_frame)));
    position_in_frame = frame->homogenous_transform_ * position_in_frame;

    frame = frame->parent_;
  }
  return FromHomogenousVelocityVector(velocity_sum);
}

Eigen::Vector3d ReferenceFrame::ToInertialAngularVelocity() {
  ReferenceFrame const* frame = this;
  Eigen::Vector4d angular_velocity_sum = Eigen::Vector4d::Zero();
  while (frame != nullptr) {
    // TODO(Kyle) Add in rotational velocity -> linear velocity
    angular_velocity_sum =
        frame->homogenous_transform_ * angular_velocity_sum +
        ToHomogenousVelocityVector(frame->frame_angular_velocity_);
    frame = frame->parent_;
  }
  return FromHomogenousVelocityVector(angular_velocity_sum);
}

bool ReferenceFrame::should_recalculate_child(FrameVersion cached_rev) const {
  return !is_cache_valid() || cached_rev < current_rev_;
}

} /* frames */

} /* util */

} /* muan */

} /* frc1678 */
