//
// Created by danielbord on 1/26/18.
//

#ifndef CAMERA_CALIBRATION_UTILITIES_H
#define CAMERA_CALIBRATION_UTILITIES_H

#include <boost/math/special_functions/erf.hpp>
#include <IScene.h>

namespace utils {
    static const double EPS = 1e-10;

    template<typename TScalar = double, int RowsAtCompileTime = Eigen::Dynamic, int ColsAtCompileTime = Eigen::Dynamic>
    inline bool loadMatrix(const std::string &filename, Eigen::Matrix<TScalar, RowsAtCompileTime, ColsAtCompileTime> &m,
                           bool transposed = false) {
        if (filename.empty())
            return false;
        std::ifstream input(filename.c_str());
        if (input.fail()) {
            LOG(WARNING) << "Cannot find file '" << filename << "'.";
            m.setZero();
            return false;
        }
        std::string line;
        TScalar d;

        std::vector<TScalar> v;
        std::size_t n_rows = 0;
        while (getline(input, line)) {
            ++n_rows;
            std::stringstream input_line(line);
            while (!input_line.eof()) {
                input_line >> d;
                v.push_back(d);
            }
        }
        input.close();

        std::size_t n_cols = v.size() / n_rows;
        if (transposed)
            std::swap(n_cols, n_rows);

        if (RowsAtCompileTime == Eigen::Dynamic)
            m.resize(n_rows, Eigen::NoChange);
        if (ColsAtCompileTime == Eigen::Dynamic)
            m.resize(Eigen::NoChange, n_cols);

        CHECK_EQ(m.cols(), n_cols) << "Invalid column count";
        CHECK_EQ(m.rows(), n_rows) << "Invalid row count";

        if (transposed)
            std::swap(n_cols, n_rows);
        for (int i = 0; i < n_rows; i++)
            for (int j = 0; j < n_cols; j++)
                if (transposed)
                    m(j, i) = v[i * n_cols + j];
                else
                    m(i, j) = v[i * n_cols + j];


        return true;
    }

    template<typename TScalar = double, int RowsAtCompileTime = Eigen::Dynamic, int ColsAtCompileTime = Eigen::Dynamic>
    inline bool saveMatrix(const std::string &directory, std::string filename,
                           Eigen::Matrix<TScalar, RowsAtCompileTime, ColsAtCompileTime> matrix,
                           bool overwrite = false) {

        if (directory.empty() or filename.empty())
            return false;


        if (!boost::filesystem::exists(directory)) {
            // Directory doesn't exist. Try to create it.
            if (!boost::filesystem::create_directories(directory)) {
                LOG(WARNING) << "Couldn't make directory '" << directory << "'. Not saving data.";
                return false;
            }
        }

        filename = directory + "/" + filename;
        saveMatrix(filename, matrix, overwrite);

        return true;
    }

    template<typename TScalar = double, int RowsAtCompileTime = Eigen::Dynamic, int ColsAtCompileTime = Eigen::Dynamic>
    inline bool
    saveMatrix(const std::string &filename, Eigen::Matrix<TScalar, RowsAtCompileTime, ColsAtCompileTime> matrix,
               bool overwrite = false) {
        if (filename.empty())
            return false;
        if (boost::filesystem::exists(filename)) {
            if (!overwrite) {
                // File exists, but overwriting is not allowed. Abort.
                LOG(WARNING) << "File '" << filename << "' already exists. Not saving data.";
                return false;
            }
        }


        std::ofstream file;
        file.open(filename.c_str());
        if (!file.is_open()) {
            LOG(WARNING) << "Couldn't open file '" << filename << "' for writing.";
            return false;
        }

        file << std::fixed;
        file << matrix;
        file.close();

        return true;

    }

    template<typename TScalar>
    auto solvePoly(Eigen::Matrix<TScalar, Eigen::Dynamic, 1> &coefficients) {
        auto deg = coefficients.size() - 1;
        coefficients /= coefficients[deg];
        Eigen::Matrix<TScalar, Eigen::Dynamic, Eigen::Dynamic> companion(deg, deg);
        companion.setZero();
        companion.col(deg - 1) = -TScalar(1) * coefficients.topRows(deg);
        companion.block(1, 0, deg - 1, deg - 1).setIdentity();

        return companion.eigenvalues();
    }

    template<typename TScalar>
    Eigen::Matrix<TScalar, 3, 3> screw_hat(const Eigen::Matrix<TScalar, 3, 1> &t) {
        Eigen::Matrix<TScalar, 3, 3> t_hat = Eigen::Matrix<TScalar, 3, 3>::Zero(3, 3);
        t_hat << TScalar(0), -t(2), t(1),
                t(2), TScalar(0), -t(0),
                -t(1), t(0), TScalar(0);
        return t_hat;
    }

    template<typename TScalar>
    Eigen::Matrix<TScalar, 3, 1> inverted_screw_hat(const Eigen::Matrix<TScalar, 3, 3> &t_hat) {
        Eigen::Matrix<TScalar, 3, 1> t;
        t << t_hat(2, 1), t_hat(0, 2), t_hat(1, 0);
        return t;
    }

    template<template<typename> class ErrorFunctor, typename TStereoPair, typename T>
    void computeErrors(const TStereoPair &stereo_pair, std::vector<T> &left_residuals,
                       std::vector<T> &right_residuals, T image_r = T(1.0)) {


        auto number_of_points = stereo_pair.getNumberOfPoints();

        left_residuals.resize(number_of_points, T(std::numeric_limits<double>::max()));
        right_residuals.resize(number_of_points, T(std::numeric_limits<double>::max()));
        auto const &u1d = stereo_pair.getLeftKeypoints();
        auto const &u2d = stereo_pair.getRightKeypoints();

        for (size_t k = 0; k < number_of_points; ++k) {

            ErrorFunctor<TStereoPair> cost;
            T left_residual, right_residual;
            bool is_correct = cost(u1d.col(k).eval(), u2d.col(k).eval(), stereo_pair, left_residual, right_residual);
            left_residuals[k] = image_r * left_residual;
            right_residuals[k] = image_r * right_residual;
        }
    }

    double estimateQuantile(std::vector<double> errors,
                            double expected_percent_of_inliers);

    double estimateConfidenceInterval(double quantile, double expected_percent_of_inliers);


    template<template<typename> class ErrorFunctor, typename TStereoPair>
    double findInliers(const TStereoPair &stereo_pair, double expected_percent_of_inliers,
                       std::vector<size_t> &inliers_indices, double image_r) {

        std::vector<double> left_residuals, right_residuals, errors(
                static_cast<unsigned long>(stereo_pair.getNumberOfPoints()));

        utils::computeErrors<ErrorFunctor<TStereoPair>, TStereoPair, double>(stereo_pair,
                                                                             left_residuals,
                                                                             right_residuals,
                                                                             image_r);
        for (std::size_t k = 0; k < errors.size(); ++k) {
            errors[k] = std::abs(left_residuals[k]) + std::abs(right_residuals[k]);
        }
        double quantile = estimateQuantile(errors, expected_percent_of_inliers);
        double interval = estimateConfidenceInterval(quantile, expected_percent_of_inliers);
        inliers_indices.resize(0);

        for (size_t k = 0; k < errors.size(); ++k) {
            if (errors[k] < interval) {
                inliers_indices.push_back(k);
            }
        }
        return interval;

    }


}
#endif //CAMERA_CALIBRATION_UTILITIES_H
