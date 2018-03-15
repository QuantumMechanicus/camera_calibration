//
// Created by danielbord on 3/15/18.
//

#ifndef CAMERA_CALIBRATION_DIVISIONMODELUTILITIES_H
#define CAMERA_CALIBRATION_DIVISIONMODELUTILITIES_H

#include "Utilities.h"

namespace utils {
    namespace division_model {
        template<typename T>
        T undistortionDenominator(const T &r_distorted,
                                  const Eigen::Matrix<T, Eigen::Dynamic, 1> &distortion_coefficients) {
            T denominator(1.0);
            T r_distorted2 = r_distorted * r_distorted;
            T r_distorted2_pow = r_distorted2;
            for (int i = 0; i < distortion_coefficients.rows(); ++i) {
                denominator += distortion_coefficients[i] * r_distorted2_pow;
                r_distorted2_pow *= r_distorted2;
            }
            return denominator;
        }

        template<typename T>
        T undistortionDenominatorDerivative(const T &r_distorted,
                                            const Eigen::Matrix<T, Eigen::Dynamic, 1> &distortion_coefficients) {
            T denominator(0.0);
            T c(2.0);
            T r_distorted2 = r_distorted * r_distorted;
            T r_distorted_pow = r_distorted;
            for (int i = 0; i < distortion_coefficients.rows(); ++i) {
                denominator += c * distortion_coefficients[i] * r_distorted_pow;
                c = c + T(2.0);
                r_distorted_pow *= r_distorted2;
            }

            return denominator;
        }

        template<typename T>
        bool checkUndistortionInvertibility(const Eigen::Matrix<T, Eigen::Dynamic, 1> &lambdas) {
            T k1 = lambdas(0);
            T k2 = T(0);
            if (lambdas.rows() > 1) {
                k2 = lambdas(1);
            }
            return (k1 > T(-2) and ((k1 > T(2) and (T(-1) - k1 < k2 and k2 < -(k1 * k1 / T(12)))) or
                                    (k1 <= T(2) and (T(-1) - k1 < k2 and k2 < (T(1) - k1) / T(3.0)))));

        }

        template<typename T>
        scene::TImagePoint<T> distortion(const scene::TImagePoint<T> &u, T rd,
                                         const Eigen::Matrix<T, Eigen::Dynamic, 1> &distortion_coefficients) {
            T denominator = undistortionDenominator<T>(rd, distortion_coefficients);
            return u * denominator;
        }

        template<typename T>
        T findDistortedRadius(const Eigen::Matrix<T, Eigen::Dynamic, 1> &distortion_coefficients, T r) {
            Eigen::Matrix<T, Eigen::Dynamic, 1> checked_distortion_coefficients;
            int count_until_non_zero = static_cast<int>(distortion_coefficients.size()) - 1;
            while (count_until_non_zero > 0 && ceres::abs(distortion_coefficients[count_until_non_zero]) < T(1e-9)) {
                --count_until_non_zero;
            }


            checked_distortion_coefficients = distortion_coefficients.head(count_until_non_zero + 1);
            T rd = T(std::numeric_limits<double>::max());
            auto deg = 2 * checked_distortion_coefficients.size();
            if (deg > 0) {
                Eigen::Matrix<T, Eigen::Dynamic, 1> coeff = Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(deg + 1);

                for (auto k = static_cast<size_t>(checked_distortion_coefficients.size()); k > 0; --k)
                    coeff(2 * k, 0) = r * checked_distortion_coefficients[k - 1];

                coeff(1, 0) = T(-1);
                coeff(0, 0) = r;
                coeff /= coeff[deg];


                auto eigenvalues = utils::solvePoly<T>(coeff);
                for (size_t j = 0; j < eigenvalues.rows(); ++j) {
                    T real = eigenvalues[j].real();
                    T imag = eigenvalues[j].imag();
                    if (ceres::abs(imag) < 1e-9 && real > T(1e-9) && rd > real) {
                        rd = real;
                    }

                }
            } else
                rd = r;
            return rd;
        }


        template<typename T>
        scene::TImagePoint<T> distortion(const scene::TImagePoint<T> &u,
                                         const Eigen::Matrix<T, Eigen::Dynamic, 1> &distortion_coefficients) {
            return distortion<T>(u, findDistortedRadius(distortion_coefficients, u.norm()), distortion_coefficients);
        }

        template<typename T>
        scene::TImagePoint<T> undistortion(const scene::TImagePoint<T> &ud,
                                           const Eigen::Matrix<T, Eigen::Dynamic, 1> &distortion_coefficients) {
            T rd = ud.norm();
            T denominator = undistortionDenominator<T>(rd, distortion_coefficients);
            return ud / denominator;
        }

    }
}

#endif //CAMERA_CALIBRATION_DIVISIONMODELUTILITIES_H
