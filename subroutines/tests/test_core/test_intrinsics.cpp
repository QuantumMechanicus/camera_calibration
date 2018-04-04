//
// Created by danielbord on 9/20/17.
//

#include <gtest/gtest.h>
#include <test_utils.h>

TEST(simple_tests, distortion_undistortion) {
    const int NUMBER_OF_KEY_POINTS = 50;
    auto left_intrinsics = test_utils::generateIntrinsics<1>();
    auto right_intrinsics = test_utils::generateIntrinsics<1>();
    Eigen::Matrix3d left_calibration = left_intrinsics.getCalibrationMatrix();
    Eigen::Matrix3d right_calibration = right_intrinsics.getCalibrationMatrix();
    scene::ImagePoints u1, u2;
    scene::WorldPoints w1, w2;
    scene::FundamentalMatrix fundamental_matrix;
    Sophus::SE3<double> leftToRight;

    test_utils::generateTwoViewScene(left_calibration, right_calibration, NUMBER_OF_KEY_POINTS, u1, u2, w1, w2,
                                     fundamental_matrix,
                                     leftToRight);

    for (size_t j = 0; j < NUMBER_OF_KEY_POINTS; ++j) {
        EXPECT_NEAR((left_intrinsics.undistort(left_intrinsics.distort(u1.col(j))) - u1.col(j)).norm(),
                    0, 1e-8);
        EXPECT_NEAR(
                (right_intrinsics.undistort(right_intrinsics.distort(u2.col(j))) - u2.col(j)).norm(), 0,
                1e-8);
    }
}

TEST(simple_tests, project_backproject) {
    const int NUMBER_OF_KEY_POINTS = 50;
    auto left_intrinsics = test_utils::generateIntrinsics<1>();
    auto right_intrinsics = test_utils::generateIntrinsics<1>();
    scene::ImagePoints u1, u2;
    scene::WorldPoints w1, w2;
    scene::FundamentalMatrix fundamental_matrix;
    Sophus::SE3<double> leftToRight;
    Eigen::Matrix3d left_calibration = left_intrinsics.getCalibrationMatrix();
    Eigen::Matrix3d right_calibration = right_intrinsics.getCalibrationMatrix();
    test_utils::generateTwoViewScene(left_calibration, right_calibration, NUMBER_OF_KEY_POINTS, u1, u2, w1, w2,
                                     fundamental_matrix,
                                     leftToRight);

    for (size_t j = 0; j < NUMBER_OF_KEY_POINTS; ++j) {
        EXPECT_NEAR((left_intrinsics.project(w1.col(j)) - u1.col(j)).norm(), 0, 1e-8);
        EXPECT_NEAR((right_intrinsics.project(w2.col(j)) - u2.col(j)).norm(), 0, 1e-8);

        Eigen::Vector3d left_backprojected = left_intrinsics.backproject(u1.col(j));
        Eigen::Vector3d right_backprojected = right_intrinsics.backproject(u2.col(j));

        EXPECT_NEAR((left_backprojected.hnormalized() - w1.col(j).hnormalized()).norm(), 0, 1e-8);
        EXPECT_NEAR((right_backprojected.hnormalized() - w2.col(j).hnormalized()).norm(), 0, 1e-8);

    }
}