//
// Created by danielbord on 3/14/18.
//

#ifndef CAMERA_CALIBRATION_DEFINITIONS_H
#define CAMERA_CALIBRATION_DEFINITIONS_H


#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>
#include <boost/math/special_functions/erf.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <ceres/ceres.h>
#include <tbb/tbb.h>
#include <glog/logging.h>
#include <fstream>
#include <memory>
#include <map>
#include <utility>
#include <regex>
#include <fstream>
#include <random>

namespace scene {
    template<typename T>
    using StdVector = std::vector<T, Eigen::aligned_allocator<T>>;

    template<typename K, typename V>
    using StdMap = std::map<K, V, std::less<K>, Eigen::aligned_allocator<std::pair<const K, V> > >;

    template<typename T>
    using TImagePoint = Eigen::Matrix<T, 2, 1>;

    template<typename T>
    using TImagePoints = Eigen::Matrix<T, 2, Eigen::Dynamic>;

    template<typename T>
    using TWorldPoint = Eigen::Matrix<T, 3, 1>;

    template<typename T>
    using TWorldPoints = Eigen::Matrix<T, 3, Eigen::Dynamic>;


    template<typename T>
    using THomogeneousWorldPoint = Eigen::Matrix<T, 4, 1>;

    template<typename T>
    using THomogenousImagePoint = Eigen::Matrix<T, 3, 1>;

    template<typename T>
    using THomogeneousWorldPoints = Eigen::Matrix<T, 4, Eigen::Dynamic>;

    template<typename T>
    using THomogenousImagePoints = Eigen::Matrix<T, 3, Eigen::Dynamic>;


    template<typename T>
    using TFundamentalMatrix = Eigen::Matrix<T, 3, 3>;

    typedef Eigen::Matrix<double, 2, 1> ImagePoint;

    typedef Eigen::Matrix<double, 3, 1> WorldPoint;

    typedef Eigen::Matrix<double, 4, 1> HomogeneousWorldPoint;

    typedef Eigen::Matrix<double, 3, 1> HomogenousImagePoint;

    typedef Eigen::Matrix<double, 2, Eigen::Dynamic> ImagePoints;

    typedef Eigen::Matrix<double, 3, Eigen::Dynamic> WorldPoints;

    typedef Eigen::Matrix<double, 3, Eigen::Dynamic> HomogenousImagePoints;

    typedef Eigen::Matrix<double, 4, Eigen::Dynamic> HomogenousWorldPoints;

    typedef Eigen::Matrix3d FundamentalMatrix;

    typedef StdVector<FundamentalMatrix> FundamentalMatrices;
}
#endif //CAMERA_CALIBRATION_DEFINITIONS_H
