//
// Created by danielbord on 3/15/18.
//

#ifndef CAMERA_CALIBRATION_DIVISIONMODELFUNCTORS_H
#define CAMERA_CALIBRATION_DIVISIONMODELFUNCTORS_H

#include "DivisionModelUtilities.h"
namespace functors {
    template<typename TStereoPair>
    struct EpipolarCurveDistanceErrorExpanded {

        template<typename T>
        bool operator()(const scene::TImagePoint<T> &u1d, const scene::TImagePoint<T> &u2d,
                        const TStereoPair &stereo_pair,
                        scene::TImagePoint<T> &curve_point1,
                        scene::TImagePoint<T> &curve_point2, T &left_residual,
                        T &right_residual) const {

            bool is_correct = true;
            scene::TImagePoint<T> u1, u2;
            u1 = stereo_pair.undistortLeft(u1d);
            u2 = stereo_pair.undistortRight(u2d);


            Eigen::Matrix<T, 2, 1> line_point1, line_point2;


            Eigen::Matrix<T, 3, 1> l2 = stereo_pair.getRightEpiline(u1);
            Eigen::Matrix<T, 3, 1> l1 = stereo_pair.getLeftEpiline(u2);

            T n1 = l1.template block<2, 1>(0, 0).norm();
            T n2 = l2.template block<2, 1>(0, 0).norm();
            T err = l1.dot(u1.homogeneous());


            left_residual = err / n1;
            right_residual = err / n2;
            line_point1 = u1 - left_residual * l1.template block<2, 1>(0, 0) / n1;
            line_point2 = u2 - right_residual * l2.template block<2, 1>(0, 0) / n2;


            curve_point1 = stereo_pair.distortLeft(line_point1);
            curve_point2 = stereo_pair.distortRight(line_point2);

            left_residual = (u1d - curve_point1).norm();
            right_residual = (u2d - curve_point2).norm();
            //TODO check correctness
            return is_correct;

        }
    };

    template<typename TStereoPair>
    struct EpipolarCurveDistanceError {

        template<typename T>
        bool operator()(const scene::TImagePoint<T> &u1d, const scene::TImagePoint<T> &u2d,
                        const TStereoPair &stereo_pair, T &left_residual,
                        T &right_residual) const {
            EpipolarCurveDistanceErrorExpanded<TStereoPair> cost;
            scene::TImagePoint<T> stub1, stub2;
            return cost(u1d, u2d, stereo_pair, stub1, stub2, left_residual,
                        right_residual);

        }
    };

    template<typename TStereoPair>
    struct ReprojectionError {

        TStereoPair stereo_pair_;
        Eigen::Vector2d u1d_;
        Eigen::Vector2d u2d_;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ReprojectionError(const TStereoPair &stereo_pair, const Eigen::Vector2d &u1d, const Eigen::Vector2d &u2d)
                : stereo_pair_(stereo_pair),
                  u1d_(u1d), u2d_(u2d) {}

        template<typename T>
        bool operator()(const T *world_point_ptr, T *residuals) const {
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> point(world_point_ptr);
            Eigen::Map<Eigen::Matrix<T, 4, 1>> rs(residuals);
            rs.template head<2>() = stereo_pair_.distortLeft(stereo_pair_.projectLeft(point).hnormalized())
                                    -
                                    u1d_.template cast<T>();
            Eigen::Matrix<T, 3, 1> right_coordinates_point = stereo_pair_.template getRelativeMotion<T>() * point;

            rs.template tail<2>() =
                    stereo_pair_.distortRight(stereo_pair_.projectRight(right_coordinates_point).hnormalized) -
                    u2d_.template cast<T>();

            return true;
        }
    };

    struct costEssentialFunctor {
        Eigen::Matrix3d fundamental_matrix;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        explicit costEssentialFunctor(const Eigen::Matrix3d &f) : fundamental_matrix(f) {}

        template<typename T>
        bool operator()(const T *f_ptr, const T *cx_ptr, const T *cy_ptr, T *residuals) const {
            Eigen::Matrix<T, 3, 3> K;
            K.setIdentity();
            K(0, 0) = K(1, 1) = *f_ptr;
            K(0, 2) = *cx_ptr;
            K(1, 2) = *cy_ptr;
            auto essential_matrix = (K.transpose() * fundamental_matrix.template cast<T>() * K).eval();

            Eigen::Map<Eigen::Matrix<T, 3, 3> > e(residuals);

            e = ((essential_matrix * essential_matrix.transpose() * essential_matrix
                  - 0.5 * (essential_matrix * essential_matrix.transpose()).trace() * essential_matrix) /
                 ceres::pow(essential_matrix.norm(), 3));
            return *f_ptr > T(0.0);

        }
    };
}
#endif //CAMERA_CALIBRATION_DIVISIONMODELFUNCTORS_H
