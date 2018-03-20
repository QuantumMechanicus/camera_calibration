//
// Created by danielbord on 9/20/17.
//

#include <gtest/gtest.h>
#include <test_utils.h>

TEST(simple_tests, undistortion)
{
   auto intrinsics = test_utils::generateIntrinsics<1>();
    double lambda = intrinsics.getDistortionCoefficients()(0);
    scene::ImagePoints u1, u2;
    scene::WorldPoints w1, w2;
    scene::FundamentalMatrix fundamental_matrix;
    Sophus::SE3<double> leftToRight;
    std::cout << std::is_standard_layout<Sophus::SE3d>::value << std::endl;
    Eigen::Matrix3d calibration = intrinsics.getCalibrationMatrix();
    test_utils::generateTwoViewScene(calibration, calibration, 5, u1, u2, w1, w2, fundamental_matrix,
    leftToRight);

    CHECK_NEAR((intrinsics.undistort<double>(intrinsics.distort<double>(u1.col(0))) - u1.col(0)).norm(), 0, 1e-8);

}

TEST(simple_tests, distortion)
{


}