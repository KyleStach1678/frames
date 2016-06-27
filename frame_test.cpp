#include "frame.h"
#include <cmath>
#include "gtest/gtest.h"

using frc1678::muan::util::frames::ReferenceFrame;

TEST(ReferenceFrame, Translation) {
  auto inertial = ReferenceFrame::InertialFrame();
  ReferenceFrame translation(&inertial,
                             (Eigen::Vector3d() << 1.0, 2.0, 3.0).finished(),
                             Eigen::Vector3d::Zero());

  auto point = translation.ToInertialCoordinates(Eigen::Vector3d::Zero());

  EXPECT_NEAR(point(0), 1.0, 1e-9);
  EXPECT_NEAR(point(1), 2.0, 1e-9);
  EXPECT_NEAR(point(2), 3.0, 1e-9);

  auto point_inv = translation.FromInertialCoordinates(Eigen::Vector3d::Zero());

  EXPECT_NEAR(point_inv(0), -1.0, 1e-9);
  EXPECT_NEAR(point_inv(1), -2.0, 1e-9);
  EXPECT_NEAR(point_inv(2), -3.0, 1e-9);
}

TEST(ReferenceFrame, CompositeTranslations) {
  auto inertial = ReferenceFrame::InertialFrame();
  ReferenceFrame translation_a(&inertial,
                               (Eigen::Vector3d() << 1.0, 2.0, 3.0).finished(),
                               Eigen::Vector3d::Zero());
  ReferenceFrame translation_b(&translation_a,
                               (Eigen::Vector3d() << -1.0, 0.0, 9.0).finished(),
                               Eigen::Vector3d::Zero());

  auto point = translation_b.ToInertialCoordinates(Eigen::Vector3d::Zero());

  EXPECT_NEAR(point(0), 0.0, 1e-9);
  EXPECT_NEAR(point(1), 2.0, 1e-9);
  EXPECT_NEAR(point(2), 12.0, 1e-9);

  auto point_inv =
      translation_b.FromInertialCoordinates(Eigen::Vector3d::Zero());

  EXPECT_NEAR(point_inv(0), 0.0, 1e-9);
  EXPECT_NEAR(point_inv(1), -2.0, 1e-9);
  EXPECT_NEAR(point_inv(2), -12.0, 1e-9);
}

TEST(ReferenceFrame, Rotations) {
  auto inertial = ReferenceFrame::InertialFrame();
  const double pi = 3.1415926535897;
  // pi/2 radian rotations around the x-, y-, and z-axes
  ReferenceFrame rotation_x(&inertial, Eigen::Vector3d::Zero(),
                            (Eigen::Vector3d() << pi / 2, 0.0, 0.0).finished());
  ReferenceFrame rotation_y(&inertial, Eigen::Vector3d::Zero(),
                            (Eigen::Vector3d() << 0.0, pi / 2, 0.0).finished());
  ReferenceFrame rotation_z(&inertial, Eigen::Vector3d::Zero(),
                            (Eigen::Vector3d() << 0.0, 0.0, pi / 2).finished());

  auto point_a = rotation_x.ToInertialCoordinates(
      (Eigen::Vector3d() << 1.0, 0.0, 0.0).finished());

  EXPECT_NEAR(point_a(0), 1.0, 1e-9);
  EXPECT_NEAR(point_a(1), 0.0, 1e-9);
  EXPECT_NEAR(point_a(2), 0.0, 1e-9);

  auto point_b = rotation_y.ToInertialCoordinates(
      (Eigen::Vector3d() << 1.0, 0.0, 0.0).finished());

  EXPECT_NEAR(point_b(0), 0.0, 1e-9);
  EXPECT_NEAR(point_b(1), 0.0, 1e-9);
  EXPECT_NEAR(point_b(2), -1.0, 1e-9);

  auto point_c = rotation_z.ToInertialCoordinates(
      (Eigen::Vector3d() << 1.0, 0.0, 0.0).finished());

  EXPECT_NEAR(point_c(0), 0.0, 1e-9);
  EXPECT_NEAR(point_c(1), 1.0, 1e-9);
  EXPECT_NEAR(point_c(2), 0.0, 1e-9);
}

TEST(ReferenceFrame, RotationOrder) {
  auto inertial = ReferenceFrame::InertialFrame();
  const double pi = 3.1415926535897;
  // Should rotate pi/6 radians pitching down and pi/2 radians yawing
  // counterclockwise
  ReferenceFrame rotation(&inertial, Eigen::Vector3d::Zero(),
                          (Eigen::Vector3d() << 0, pi / 6, pi / 3).finished());

  auto point = rotation.ToInertialCoordinates(
      (Eigen::Vector3d() << 1.0, 0.0, 0.0).finished());

  EXPECT_NEAR(point(0), cos(pi / 3) * cos(pi / 6), 1e-9);
  EXPECT_NEAR(point(1), sin(pi / 3) * cos(pi / 6), 1e-9);
  EXPECT_NEAR(point(2), -sin(pi / 6), 1e-9);
}

TEST(ReferenceFrame, CompositeRotations) {
  auto inertial = ReferenceFrame::InertialFrame();
  const double pi = 3.1415926535897;
  // Should rotate pi/6 radians pitching down and pi/2 radians yawing
  // counterclockwise
  ReferenceFrame rotation_a(&inertial, Eigen::Vector3d::Zero(),
                            (Eigen::Vector3d() << 0, pi / 6, 0).finished());
  ReferenceFrame rotation_b(&rotation_a, Eigen::Vector3d::Zero(),
                            (Eigen::Vector3d() << 0, 0, pi / 3).finished());

  auto point = rotation_b.ToInertialCoordinates(
      (Eigen::Vector3d() << 1.0, 0.0, 0.0).finished());

  EXPECT_NEAR(point(0), cos(pi / 3) * cos(pi / 6), 1e-9);
  EXPECT_NEAR(point(1), sin(pi / 3), 1e-9);
  EXPECT_NEAR(point(2), -cos(pi / 3) * sin(pi / 6), 1e-9);
}

TEST(ReferenceFrame, TranslationalVelocity) {
  auto inertial = ReferenceFrame::InertialFrame();
  ReferenceFrame moving_frame(&inertial, Eigen::Vector3d::Zero(),
                              Eigen::Vector3d::Zero());
  moving_frame.SetVelocity((Eigen::Vector3d() << 1.0, 2.0, 3.0).finished(),
                           Eigen::Vector3d::Zero());

  auto vel =
      moving_frame.ToInertialTranslationalVelocity(Eigen::Vector3d::Zero());

  EXPECT_NEAR(vel(0), 1.0, 1e-9);
  EXPECT_NEAR(vel(1), 2.0, 1e-9);
  EXPECT_NEAR(vel(2), 3.0, 1e-9);

  ReferenceFrame moving_frame_b(&moving_frame, Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero());
  moving_frame_b.SetVelocity((Eigen::Vector3d() << 2.0, 4.0, 7.5).finished(),
                             Eigen::Vector3d::Zero());

  auto vel_b =
      moving_frame_b.ToInertialTranslationalVelocity(Eigen::Vector3d::Zero());

  EXPECT_NEAR(vel_b(0), 3.0, 1e-9);
  EXPECT_NEAR(vel_b(1), 6.0, 1e-9);
  EXPECT_NEAR(vel_b(2), 10.5, 1e-9);
}

TEST(ReferenceFrame, RotationToTranslation) {
  auto inertial = ReferenceFrame::InertialFrame();
  ReferenceFrame moving_frame(&inertial, Eigen::Vector3d::Zero(),
                              Eigen::Vector3d::Zero());
  // Make a frame with a 1rad/s rotational velocity around the z axis
  moving_frame.SetVelocity(Eigen::Vector3d::Zero(),
                           (Eigen::Vector3d() << 0.0, 0.0, 1.0).finished());

  // A point at (1, 0, 0) should be travelling at (0, 1, 0)
  auto vel = moving_frame.ToInertialTranslationalVelocity(
      (Eigen::Vector3d() << 1.0, 0.0, 0.0).finished());

  EXPECT_NEAR(vel(0), 0.0, 1e-9);
  EXPECT_NEAR(vel(1), 1.0, 1e-9);
  EXPECT_NEAR(vel(2), 0.0, 1e-9);
}

TEST(ReferenceFrame, CacheIsCorrect) {
  auto inertial = ReferenceFrame::InertialFrame();
  ReferenceFrame parent(&inertial,
                        (Eigen::Vector3d() << 1.0, 2.0, 3.0).finished(),
                        Eigen::Vector3d::Zero());
  ReferenceFrame child(&parent, (Eigen::Vector3d() << 1.0, 2.0, 3.0).finished(),
                       Eigen::Vector3d::Zero());

  {
    auto point = child.ToInertialCoordinates(Eigen::Vector3d::Zero());

    EXPECT_NEAR(point(0), 2.0, 1e-9);
    EXPECT_NEAR(point(1), 4.0, 1e-9);
    EXPECT_NEAR(point(2), 6.0, 1e-9);
  }

  child.Set((Eigen::Vector3d() << -1.0, -2.0, -3.0).finished(),
            Eigen::Vector3d::Zero());
  {
    auto point = child.ToInertialCoordinates(Eigen::Vector3d::Zero());

    EXPECT_NEAR(point(0), 0.0, 1e-9);
    EXPECT_NEAR(point(1), 0.0, 1e-9);
    EXPECT_NEAR(point(2), 0.0, 1e-9);
  }

  parent.Set((Eigen::Vector3d() << -1.0, -2.0, -3.0).finished(),
             Eigen::Vector3d::Zero());
  {
    auto point = child.ToInertialCoordinates(Eigen::Vector3d::Zero());

    EXPECT_NEAR(point(0), -2.0, 1e-9);
    EXPECT_NEAR(point(1), -4.0, 1e-9);
    EXPECT_NEAR(point(2), -6.0, 1e-9);
  }
}

TEST(ReferenceFrame, CachePerfTest) {
  auto inertial = ReferenceFrame::InertialFrame();
  ReferenceFrame parent(&inertial,
                        (Eigen::Vector3d() << 1.0, 2.0, 3.0).finished(),
                        Eigen::Vector3d::Zero());
  ReferenceFrame child(&parent, (Eigen::Vector3d() << 1.0, 2.0, 3.0).finished(),
                       Eigen::Vector3d::Zero());

  for (uint32_t i = 0; i < 100000; i++) {
    auto point = child.ToInertialCoordinates(Eigen::Vector3d::Zero());

    EXPECT_NEAR(point(0), 2.0, 1e-9);
    EXPECT_NEAR(point(1), 4.0, 1e-9);
    EXPECT_NEAR(point(2), 6.0, 1e-9);
  }
}
