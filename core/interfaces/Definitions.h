//
// Created by danielbord on 3/14/18.
//

#ifndef CAMERA_CALIBRATION_DEFINITIONS_H
#define CAMERA_CALIBRATION_DEFINITIONS_H


#include <Eigen/Dense>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>
#include <memory>
#include <map>

namespace scene {
    template<typename T>
    using StdVector = std::vector<T, Eigen::aligned_allocator<T>>;

    template<typename T>
    using TImagePoint = Eigen::Matrix<T, 2, 1>;

    template<typename T>
    using TImagePoints = Eigen::Matrix<T, 2, Eigen::Dynamic>;

    template<typename T>
    using THomogenousImagePoint = Eigen::Matrix<T, 3, 1>;

    template<typename T>
    using THomogenousImagePoints = Eigen::Matrix<T, 3, Eigen::Dynamic>;

    template<typename T>
    using TFundamentalMatrix = Eigen::Matrix<T, 3, 3>;

    typedef Eigen::Matrix<double, 2, 1> ImagePoint;

    typedef Eigen::Matrix<double, 3, 1> WorldPoint;

    typedef Eigen::Matrix<double, 4, 1> HomogenousWorldPoint;

    typedef Eigen::Matrix<double, 3, 1> HomogenousImagePoint;

    typedef Eigen::Matrix<double, 2, Eigen::Dynamic> ImagePoints;

    typedef Eigen::Matrix<double, 3, Eigen::Dynamic> HomogenousImagePoints;

    typedef Eigen::Matrix<double, 3, Eigen::Dynamic> WorldPoints;

    typedef Eigen::Matrix<double, 4, Eigen::Dynamic> HomogenousWorldPoints;

    typedef Eigen::Matrix3d FundamentalMatrix;

    typedef StdVector<FundamentalMatrix> FundamentalMatrices;
}
#endif //CAMERA_CALIBRATION_DEFINITIONS_H
