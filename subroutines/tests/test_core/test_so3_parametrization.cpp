//
// Created by danielbord on 9/21/17.
//
#include <gtest/gtest.h>
#include <Local_Parametrization_SO3.h>
#include <test_utils.h>

namespace test_plus {
    struct SO3Plus {

        template<typename T>
        bool operator()(T const *T_raw, T const *delta_raw,
                  T *T_plus_delta_raw) const {
            Eigen::Map<Sophus::SO3<T> const> const R(T_raw);
            Eigen::Map<Eigen::Matrix<T, 3, 1> const> const delta(delta_raw);
            Eigen::Map<Sophus::SO3<T>> R_plus_delta(T_plus_delta_raw);
            R_plus_delta = R * Sophus::SO3<T>::exp(delta);
            return true;
        }
    };
}

TEST(simple_tests, so3_jacobian) {
    for (size_t j = 0; j < 100; ++j) {
        local_parametrization::LocalParameterizationSO3 ref_parameterization;
        double p[4];
        Eigen::Map<Sophus::SO3d> x(p);
        x = test_utils::generateRandomMotion().so3();
        double jacobian_ref[12];
        double jacobian[12];
        ref_parameterization.ComputeJacobian(p, jacobian_ref);
        ceres::AutoDiffLocalParameterization<test_plus::SO3Plus, 4, 3> parameterization;
        parameterization.ComputeJacobian(p, jacobian);

        for (int i = 0; i < 12; ++i) {
            ASSERT_NEAR(jacobian[i], jacobian_ref[i], 1e-8);
        }
    }
}