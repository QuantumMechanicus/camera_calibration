//
// Created by danielbord on 3/15/18.
//

#ifndef CAMERA_CALIBRATION_DIVISIONMODELUTILITIES_H
#define CAMERA_CALIBRATION_DIVISIONMODELUTILITIES_H

#include "Utilities.h"

namespace utils {

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


                auto eigenvalues = utils::solvePoly(coeff);
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

        template<typename TScalar>
        class DivisionModelWrapperStereoPair
                : public scene::ITwoView<DivisionModelWrapperStereoPair<TScalar>, TScalar> {
            friend class scene::ITwoView<DivisionModelWrapperStereoPair<TScalar>, TScalar>;

            scene::TFundamentalMatrix<TScalar> bifocal_tensor_;
            Eigen::Matrix<TScalar, Eigen::Dynamic, 1> left_distortion_;
            TScalar left_f_, left_ppx_, left_ppy_;
            bool symmetric_;
            Eigen::Matrix<TScalar, Eigen::Dynamic, 1> right_distortion_;
            TScalar right_f_, right_ppx_, right_ppy_;

        protected:

            Eigen::Matrix<TScalar, 3, 3> getLeftCalibration() const {
                Eigen::Matrix<TScalar, 3, 3> calibration;
                calibration.setIdentity();
                calibration(0, 0) = calibration(1, 1) = left_f_;
                calibration(0, 2) = left_ppx_;
                calibration(1, 2) = left_ppy_;
                return calibration;
            }

            Eigen::Matrix<TScalar, 3, 3> getRightCalibration() const {
                if (symmetric_)
                    return getLeftCalibration();

                Eigen::Matrix<TScalar, 3, 3> calibration;
                calibration.setIdentity();
                calibration(0, 0) = calibration(1, 1) = right_f_;
                calibration(0, 2) = right_ppx_;
                calibration(1, 2) = right_ppy_;
                return calibration;
            }

            scene::TImagePoint<TScalar> projectLeftImpl(const scene::TWorldPoint<TScalar> &wp) const {
                auto left_calibration = getLeftCalibration();
                return (left_calibration * wp).hnormalized();
            }


            scene::HomogeneousWorldPoint backprojectLeftImpl(const scene::TImagePoint<TScalar> &p) const {
                auto left_calibration = getLeftCalibration();
                return (left_calibration.inverse() * p.homogeneous()).normalized();
            }


            scene::TImagePoint<TScalar> projectRightImpl(const scene::TWorldPoint<TScalar> &wp) const {
                auto right_calibration = getRightCalibration();
                return (right_calibration * wp).hnormalized();
            }


            scene::THomogeneousWorldPoint<TScalar> backprojectRightImpl(const scene::TImagePoint<TScalar> &p) const {
                auto right_calibration = getRightCalibration();
                return (right_calibration.inverse() * p.homogeneous()).normalized();
            }

            const Eigen::Matrix<TScalar, 3, 1> getRightEpilineImpl(const scene::TImagePoint<TScalar> &u) const {
                return bifocal_tensor_.template cast<TScalar>() * u.homogeneous();
            }


            const Eigen::Matrix<TScalar, 3, 1> getLeftEpilineImpl(const scene::TImagePoint<TScalar> &u) const {
                return bifocal_tensor_.template cast<TScalar>().transpose() * u.homogeneous();
            }


            scene::TImagePoint<TScalar> undistortLeftImpl(const scene::TImagePoint<TScalar> &pd) const {
                return undistortion(pd, left_distortion_);
            }


            scene::TImagePoint<TScalar> undistortRightImpl(const scene::TImagePoint<TScalar> &pd) const {
                if (symmetric_)
                    return undistortLeftImpl(pd);
                return undistortion(pd, right_distortion_);
            }


            scene::TImagePoint<TScalar> distortLeftImpl(const scene::TImagePoint<TScalar> &pd) const {
                return distortion(pd, left_distortion_);
            }


            scene::TImagePoint<TScalar> distortRightImpl(const scene::TImagePoint<TScalar> &pd) const {
                if (symmetric_)
                    return distortLeftImpl(pd);
                return distortion(pd, right_distortion_);
            }

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            DivisionModelWrapperStereoPair(const scene::TFundamentalMatrix<TScalar> &bifocal_tensor_,
                                           const Eigen::Matrix<TScalar, -1, 1> &left_distortion_,
                                           TScalar left_f = TScalar(1), TScalar left_ppx = TScalar(0),
                                           TScalar left_ppy = TScalar(0),
                                           bool symmetric = true,
                                           const Eigen::Matrix<TScalar, -1, 1> &right_distortion_ = Eigen::Matrix<TScalar, -1, 1>(),
                                           TScalar right_f = TScalar(1),
                                           TScalar right_ppx = TScalar(0),
                                           TScalar right_ppy = TScalar(0))
                    : left_distortion_(left_distortion_), right_distortion_(right_distortion_),
                      bifocal_tensor_(bifocal_tensor_), symmetric_(symmetric),
                      left_f_(left_f), left_ppx_(left_ppx), left_ppy_(left_ppy),
                      right_f_(right_f), right_ppx_(right_ppx), right_ppy_(right_ppy) {}

            TScalar &getRawLeftFocalLength() {
                return *left_f_;
            }

            TScalar &getRawLeftPrincipialPointX() {
                return *left_ppx_;
            }


            TScalar &getRawLeftPrincipialPointY() {
                return *left_ppy_;
            }

            TScalar &getRawFundamental() {
                return bifocal_tensor_.data();
            }

            TScalar &getRawRightFocalLength() {
                return (!symmetric_) ? *right_f_ : *left_f_;
            }

            TScalar &getRawRightPrincipialPointX() {
                return (!symmetric_) ? *right_ppx_ : *left_ppx_;
            }


            TScalar &getRawRightPrincipialPointY() {
                return (!symmetric_) ? *right_ppy_ : *left_ppy_;
            }

        };


    }
}

#endif //CAMERA_CALIBRATION_DIVISIONMODELUTILITIES_H
