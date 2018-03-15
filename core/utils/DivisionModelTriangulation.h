//
// Created by danielbord on 3/15/18.
//

#ifndef CAMERA_CALIBRATION_DIVISIONMODELTRIANGULATION_H
#define CAMERA_CALIBRATION_DIVISIONMODELTRIANGULATION_H

#include "DivisionModelFunctors.h"

namespace utils {

    scene::WorldPoint baseTriangulation(const Sophus::SE3d &leftToRight,
                                        const Eigen::Vector3d &dirLeft,
                                        const Eigen::Vector3d &dirRight);

    template<typename TStereoPair>
    double pointsOnCurves(const TStereoPair &stereo_pair,
                          const scene::ImagePoint &distorted_left, const scene::ImagePoint &distorted_right,
                          scene::ImagePoint &distorted_curve_left, scene::ImagePoint &distorted_curve_right) {
        double residual_left, residual_right;

        functors::EpipolarCurveDistanceErrorExpanded<TStereoPair> cst;
        cst(distorted_left, distorted_right, stereo_pair, distorted_curve_left, distorted_curve_right, residual_left,
            residual_right);
        return std::pow(residual_left, 2) + std::pow(residual_right, 2);
    }

    template<typename TStereoPair>
    void triangulate(const TStereoPair &stereo_pair,
                     const scene::ImagePoint &left_keypoint,
                     const scene::ImagePoint &right_keypoint,
                     scene::HomogeneousWorldPoint &left_backprojected,
                     scene::HomogeneousWorldPoint &right_backprojected,
                     double *reproj_error) {

        Eigen::Matrix3d left_calibration = stereo_pair.getLeftCalibrationMatrix();
        Eigen::Matrix3d right_calibration = stereo_pair.getRightCalibrationMatrix();
        Eigen::FullPivHouseholderQR<Eigen::Matrix3d> leftCalibrationQR(left_calibration);
        Eigen::FullPivHouseholderQR<Eigen::Matrix3d> rightCalibrationQR(right_calibration);


        Eigen::Vector2d distorted_keypoint_left = stereo_pair.distortLeft(left_keypoint),
                distorted_keypoint_right = stereo_pair.distortRight(right_keypoint);

        Eigen::Vector2d distorted_curve_left, distorted_curve_right;
        double expected_error = pointsOnCurves(stereo_pair, distorted_keypoint_left,
                                               distorted_curve_right, distorted_curve_left, distorted_curve_right);

        // XXX: taking left ray as point on curve corresponding to right's point epipolar line
        Eigen::Vector3d left_ray = leftCalibrationQR.solve(
                (stereo_pair.undistortLeft(distorted_curve_left)).homogeneous().normalized()),
                right_ray = rightCalibrationQR.solve(right_keypoint.homogeneous()).normalized();

        double left_error_initial = (
                stereo_pair.distortLeft(stereo_pair.projectLeft(left_ray)) - distorted_keypoint_left).squaredNorm();
        double right_error_initial = (
                stereo_pair.distortRight(stereo_pair.projectRight(right_ray)) - distorted_keypoint_right).squaredNorm();

        //CHECK_LT(left_error_initial, 2.0*expected_error) << "Left ray error mismatch";
        //CHECK_LT(right_error_initial, 1e-5) << "Right ray error mismatch";




        Sophus::SE3d leftToRight = stereo_pair.getRelativeMotion();

        Eigen::Matrix3d E_expected = stereo_pair.getEssentialMatrix();
        E_expected /= E_expected.norm();
        Eigen::Matrix3d E_observed = utils::screw_hat(leftToRight.translation()) * leftToRight.rotationMatrix();
        E_observed /= E_observed.norm();
        double diff_E = std::min((E_expected - E_observed).norm(), (E_expected + E_observed).norm());
        //CHECK_LT(diff_E, 1e-5) << "Pose does not match fundamental matrix";
        //CHECK_LT(right_ray.transpose() * E_observed * left_ray, 1e-2) << "Rays/translation are not coplanar";


        const Eigen::Vector3d pt_init = baseTriangulation(leftToRight, left_ray, right_ray);
        double left_error_lsq = stereo_pair.distortLeft(
                stereo_pair.projectLeft((leftToRight.inverse() * pt_init)) - distorted_keypoint_left).squaredNorm();
        double right_error_lsq = (stereo_pair.distortRight(stereo_pair.projectRight(pt_init)) -
                                  distorted_keypoint_right).squaredNorm();

        //CHECK_LT(left_error_lsq, 10.0*expected_error) << "Left point LSQ-solution error mismatch";
        //CHECK_LT(right_error_lsq, 10.0*expected_error) << "Right point LSQ-solution error mismatch";


        right_backprojected = pt_init.homogeneous();
        left_backprojected = (leftToRight.inverse() * pt_init).homogeneous();
        using ReprojectionError = functors::ReprojectionError<TStereoPair>;
        ceres::Problem problem;
        problem.AddParameterBlock(left_backprojected.data(), 3);
        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<ReprojectionError, 4, 3>(
                new ReprojectionError(stereo_pair,
                                      distorted_keypoint_left,
                                      distorted_keypoint_right)), nullptr,
                                 left_backprojected.data());
        ceres::Solver::Options options;
        //options.max_trust_region_radius = 0.01;
        options.max_num_iterations = 500;
        options.linear_solver_type = ceres::DENSE_QR;
        options.num_threads = 8;
        options.function_tolerance = 1e-16;
        options.parameter_tolerance = 1e-16;
        options.minimizer_progress_to_stdout = false;
        options.preconditioner_type = ceres::IDENTITY;
        options.jacobi_scaling = false;


        // Solve
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        right_backprojected.template head<3>() = leftToRight * left_backprojected.template head<3>();
        if (reproj_error)
            *reproj_error = std::sqrt(summary.final_cost);

        const Eigen::Vector3d pt_nnls = leftToRight * left_backprojected.head<3>();
        double left_error_nnls = (stereo_pair.distortLeft(stereo_pair.projectLeft(leftToRight.inverse() * pt_nnls)) -
                                  distorted_keypoint_left).squaredNorm();
        double right_error_nnls = (stereo_pair.distortRight(
                stereo_pair.projectRight(pt_nnls)) - distorted_keypoint_right).squaredNorm();
        //CHECK_LT(left_error_nnls, 10.0*expected_error) << "Left point NNLS-solution error mismatch";
        //CHECK_LT(right_error_nnls, 10.0*expected_error) << "Right point NNLS-solution error mismatch";

    }


}


#endif //CAMERA_CALIBRATION_DIVISIONMODELTRIANGULATION_H
