//
// Created by danielbord on 3/20/18.
//

#include <random>
#include <Scene.h>
#include "test_utils.h"

namespace test_utils{

    Sophus::SE3d generateRandomMotion()
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        return Sophus::SE3d::sampleUniform(gen);
    }

    void
    generateTwoViewScene(
            const Eigen::Matrix3d &left_calibration,
            const Eigen::Matrix3d &right_calibration, unsigned int number_of_keypoints, scene::ImagePoints &u1,
            scene::ImagePoints &u2,
            scene::WorldPoints &w1, scene::WorldPoints &w2, scene::FundamentalMatrix &fundamental_matrix,
            Sophus::SE3d &leftToRight) {

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> distribution(0, 1);
        leftToRight = generateRandomMotion();

        w1.resize(Eigen::NoChange, number_of_keypoints);
        w2.resize(Eigen::NoChange, number_of_keypoints);
        u1.resize(Eigen::NoChange, number_of_keypoints);
        u2.resize(Eigen::NoChange, number_of_keypoints);
        for (size_t k = 0; k < number_of_keypoints; ++k) {
            w1.col(k)(0) = distribution(gen) - 0.5;
            w1.col(k)(1) = distribution(gen) - 0.5;
            w1.col(k)(2) = 5 * distribution(gen) + 1;
            w2.col(k) = leftToRight * w1.col(k);
            u1.col(k) = (left_calibration * w1.col(k)).hnormalized();
            u2.col(k) = (right_calibration * w2.col(k)).hnormalized();
        }
        fundamental_matrix = right_calibration.transpose().inverse() *
                             utils::screw_hat<double>(leftToRight.translation()) * leftToRight.so3().matrix() *
                             left_calibration.inverse();

    }

    Eigen::Vector3d generatePointOnSphere(const Eigen::Vector3d &shift, double raidus)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<double> distribution(0, 1);
        double x, y, z, norm;
        x = distribution(gen);
        y = distribution(gen);
        z = distribution(gen);
        norm = std::sqrt(x * x + y * y + z * z);
        return (Eigen::Vector3d(raidus*x / norm, raidus*y / norm, raidus*z / norm) + shift);
    }

}