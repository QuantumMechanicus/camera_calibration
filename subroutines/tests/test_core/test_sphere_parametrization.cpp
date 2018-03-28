//
// Created by danielbord on 9/21/17.
//
#include <gtest/gtest.h>
#include <Local_Parametrization_Sphere.h>
#include <test_utils.h>

namespace test_plus {
    struct SpherePlus {
        const double radius_;

        SpherePlus() : radius_(1) {}

        SpherePlus(double norm) : radius_(norm) {}

        template<typename T>
        static void
        calculateBasis(const Eigen::Matrix<T, 3, 1> &p, Eigen::Matrix<T, 3, 1> &du, Eigen::Matrix<T, 3, 1> &dv) {
            Eigen::Matrix<T, 3, 1> x, y, z, c;
            x << (T) 1, (T) 0, (T) 0;
            y << (T) 0, (T) 1, (T) 0;
            z << (T) 0, (T) 0, (T) 1;
            auto maxNonCollinearity = x.cross(p).norm();
            c = x;
            if (y.cross(p).norm() > maxNonCollinearity) {
                c = y;
                maxNonCollinearity = y.cross(p).norm();
            }
            if (z.cross(p).norm() > maxNonCollinearity) {
                c = z;
            }

            du = (p - c).cross(p);
            dv = du.cross(p);
            du.normalize();
            dv.normalize();
        }


        template<typename T>
        bool operator()(const T *x_raw, const T *delta_raw, T *x_plus_delta_raw) const {

            Eigen::Map<Eigen::Matrix<T, 3, 1> const> x(x_raw);
            Eigen::Map<Eigen::Matrix<T, 3, 1> > x_plus_delta(x_plus_delta_raw);

            Eigen::Matrix<T, 3, 1> du, dv;
            calculateBasis<T>(x, du, dv);

            x_plus_delta = x + delta_raw[0] * du + delta_raw[1] * dv;

            x_plus_delta *= (T) radius_ / x_plus_delta.norm();

            return true;
        }
    };
}

TEST(simple_tests, sphere_jacobian) {
    for (size_t j = 0; j < 100; ++j) {
        local_parametrization::LocalParameterizationSphere ref_parameterization(1);
        double p[3];
        Eigen::Map<Eigen::Vector3d> x(p);
        x = test_utils::generatePointOnSphere();
        double jacobian_ref[6];
        double jacobian[6];
        ref_parameterization.ComputeJacobian(p, jacobian_ref);
        ceres::AutoDiffLocalParameterization<test_plus::SpherePlus, 3, 2> parameterization;
        parameterization.ComputeJacobian(p, jacobian);

        for (int i = 0; i < 6; ++i) {
            ASSERT_NEAR(jacobian[i], jacobian_ref[i], 1e-8);
        }
    }
}