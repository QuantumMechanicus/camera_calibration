//
// Created by danielbord on 3/15/18.
//

#ifndef CAMERA_CALIBRATION_DIVISIONMODELTRIANGULATION_H
#define CAMERA_CALIBRATION_DIVISIONMODELTRIANGULATION_H

#include "Functors.h"

namespace utils {

    inline scene::WorldPoint baseTriangulation(const Sophus::SE3d &leftToRight,
                                               const Eigen::Vector3d &dirLeft,
                                               const Eigen::Vector3d &dirRight) {
        Eigen::Vector3d dir_left = (leftToRight.so3() * dirLeft).normalized(),
                dir_right = dirRight.normalized(),
                t = leftToRight.translation();
        Eigen::Matrix<double, 3, 2> A;
        A.col(0) = dir_left;
        A.col(1) = -dir_right;
        Eigen::Vector3d b = -leftToRight.translation();
        Eigen::Vector2d alphas = A.fullPivHouseholderQr().solve(b);
        return (alphas[0] * dir_left + t + alphas[1] * dir_right) / 2.0;
    }


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
                     double *reproj_error = nullptr) {

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

    template<typename TStereoPair>
    bool chiralityTest(const TStereoPair &stereo_pair,
                       const scene::ImagePoint &left_keypoint, const scene::ImagePoint &right_keypoint,
                       double *reproj_error = nullptr) {

        scene::HomogeneousWorldPoint left_backprojected, right_backprojected;
        triangulate(stereo_pair, left_keypoint, right_keypoint, left_backprojected, right_backprojected, reproj_error);
        bool c1 = left_backprojected[2] * left_backprojected[3] > 0;
        bool c2 = right_backprojected[2] * right_backprojected[3] > 0;
        return (c1 && c2);
    }

    template<typename TStereoPair>
    double findChiralityInliers(const TStereoPair &stereo_pair,
                       double expected_percent_of_inliers,
                       std::vector<size_t> &inliers_indices, double image_r) {

        scene::ImagePoints u1d = stereo_pair->getLeftKeyPoints();
        scene::ImagePoints u2d = stereo_pair->getRightKeyPoints();

        Sophus::SE3d leftToRight = stereo_pair->getRelativeMotion();
        Eigen::Matrix3d calibration = stereo_pair->getCalibrationMatrix();

        Eigen::Matrix3d translation_matrix = screw_hat(
                leftToRight.translation()), rotation_matrix = leftToRight.so3().matrix();
        Eigen::Matrix3d fundamental_matrix =
                calibration.inverse().transpose() * translation_matrix * rotation_matrix * calibration.inverse();


        std::vector<double> errors(u1d.cols());

        tbb::parallel_for(tbb::blocked_range<int>(0, u1d.cols()),
                          [&](auto range) {
                              for (int i = range.begin(); i != range.end(); ++i) {

                                  double err = std::numeric_limits<double>::max();

                                  chiralityTest(stereo_pair, stereo_pair->undistortLeft(u1d.col(i)),
                                                stereo_pair->undistortRight(u2d.col(i)), &err) ? 1 : 0;

                                  double px_err = err * image_r;
                                  errors[i] = px_err;
                              }

                          }
        );

        double quantile = estimateQuantile(errors, expected_percent_of_inliers);
        double interval = estimateConfidenceInterval(quantile, expected_percent_of_inliers);
        inliers_indices.clear();

        std::atomic<int> invalid_threshold(0), invalid_chirality(0), invalid_reprojection_threshold(0);

        std::vector<int> validity(u1d.cols());


        tbb::parallel_for(tbb::blocked_range<int>(0, u1d.cols()),
                          [&](auto range) {
                              for (int i = range.begin(); i != range.end(); ++i) {

                                  if (errors[i] >= interval) {
                                      validity[i] = 0;
                                      ++invalid_threshold;
                                      continue;
                                  }
                                  double err = std::numeric_limits<double>::max();
                                  validity[i] = chiralityTest(stereo_pair, stereo_pair->undistortLeft(u1d.col(i)),
                                                              stereo_pair->undistortRight(u2d.col(i)), &err) ? 1 : 0;

                                  double px_err = err * image_r;
                                  if (!validity[i]) {
                                      ++invalid_chirality;
                                  }

                                  if (px_err > interval * 5.0) {
                                      validity[i] = 0;
                                      ++invalid_reprojection_threshold;
                                      LOG(WARNING) << "Expected error: " << interval << " observed error: " << px_err;
                                      continue;
                                  }
                                  LOG(INFO) << "Reprojection error: " << px_err;
                              }

                          }
        );


        for (size_t k = 0; k < u1d.cols(); ++k) {
            if (validity[k]) {
                inliers_indices.push_back(k);
            }
        }
        LOG(INFO) << "Inliers: " << inliers_indices.size() << " [out of: " << u1d.cols() << "]; " << invalid_threshold
                  << "/" << invalid_chirality << "/" << invalid_reprojection_threshold << " thresh/chir/repr";
        return interval;

    }

}


#endif //CAMERA_CALIBRATION_DIVISIONMODELTRIANGULATION_H
