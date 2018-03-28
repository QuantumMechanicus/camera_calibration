//
// Created by danielbord on 3/20/18.
//

#ifndef CAMERA_CALIBRATION_TEST_UTILS_H
#define CAMERA_CALIBRATION_TEST_UTILS_H

#include <Core.h>

namespace test_utils {
    template<int N>
    intrinsics::DivisionModel<N> generateIntrinsics(unsigned int w = 2, unsigned int h = 2) {
        std::random_device rd;
        std::mt19937 gen(rd());
        double r = std::sqrt(w * w + h * h) / 2;
        double ppx, ppy;
        std::uniform_real_distribution<> distribution(0, 1);
        double fov = distribution(gen);
        double f = r / (std::tan(fov / 2));
        CHECK_NEAR(2 * std::atan(r / f), fov, 1e-4);
        ppx = distribution(gen) / 10.0 - 0.05;
        ppy = distribution(gen) / 10.0 - 0.05;
        Eigen::Matrix<double, N, 1> lambdas;
        lambdas.setRandom();
        Eigen::Matrix<double, N, 1> ones;
        ones.setOnes();
        lambdas = (lambdas - ones) / 4;
        return intrinsics::DivisionModel<N>(lambdas, w, h, f, ppx, ppy);
    }

    Sophus::SE3d generateRandomMotion();

    void
    generateTwoViewScene(
            const Eigen::Matrix3d &left_calibration,
            const Eigen::Matrix3d &right_calibration, unsigned int number_of_keypoints, scene::ImagePoints &u1,
            scene::ImagePoints &u2,
            scene::WorldPoints &w1, scene::WorldPoints &w2, scene::FundamentalMatrix &fundamental_matrix,
            Sophus::SE3d &leftToRight);

    Eigen::Vector3d generatePointOnSphere(const Eigen::Vector3d &shift = Eigen::Vector3d::Zero(), double raidus = 1);

}
#endif //CAMERA_CALIBRATION_TEST_UTILS_H
