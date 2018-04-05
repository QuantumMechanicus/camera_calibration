//
// Created by danielbord on 2/14/18.
//

#include "Non_Linear_Estimator.h"


namespace non_linear_optimization {
    NonLinearEstimatorOptions::NonLinearEstimatorOptions(int number_of_non_linear_iters, double quantile_to_minimize,
                                                         double max_interval,
                                                         double image_radius) :
            number_of_non_linear_iters_(number_of_non_linear_iters),
            max_interval_(max_interval),
            quantile_to_minimize_(quantile_to_minimize),
            image_radius_(image_radius) {}

    void NonLinearEstimator::estimateImpl() {

        is_estimated_ = true;
        int residuals = 0;
        long number_of_distortion_coefficients = lambdas_.rows();

        for (size_t iters = 0; iters < options_.number_of_non_linear_iters_; ++iters) {
            ceres::Problem problem;
            double *lambda_ptr = lambdas_.data();
            problem.AddParameterBlock(lambda_ptr, static_cast<int>(number_of_distortion_coefficients));

            for (size_t kth_pair = 0; kth_pair < number_of_pairs_; ++kth_pair) {
                stereo_pairs_[kth_pair].estimateLeftCamera(lambdas_);
                stereo_pairs_[kth_pair].estimateRightCamera(lambdas_);

                auto &kth_fundamental_matrix = fs_[kth_pair];
                utils::forceFundamentalRank(kth_fundamental_matrix);

                double *f_ptr = kth_fundamental_matrix.data();
                problem.AddParameterBlock(f_ptr, 8);

                std::vector<size_t> inliers_ind;

                double interval = utils::findInliers<scene::DynamicDivisionModelStereoPair>(
                        stereo_pairs_[kth_pair],
                        options_.quantile_to_minimize_,
                        inliers_ind, options_.image_radius_);
                std::cout <<"Int: " << interval << std::endl;
                LOG_IF(WARNING, interval > options_.max_interval_)
                << kth_pair + 1 << "-nth stereo pair with high confidence interval --- "
                << interval << std::endl;


                Eigen::Matrix<double, 2, Eigen::Dynamic> i1d, i2d;
                i1d.resize(Eigen::NoChange, inliers_ind.size());
                i2d.resize(Eigen::NoChange, inliers_ind.size());
                const scene::ImagePoints &left_keypoints = stereo_pairs_[kth_pair].getLeftKeypoints();
                const scene::ImagePoints &right_keypoints = stereo_pairs_[kth_pair].getRightKeypoints();

                for (size_t kth_inlier = 0; kth_inlier < inliers_ind.size(); ++kth_inlier) {
                    i1d.col(kth_inlier) = left_keypoints.col(inliers_ind[kth_inlier]);
                    i2d.col(kth_inlier) = right_keypoints.col(inliers_ind[kth_inlier]);
                }

                for (size_t k = 0; k < i1d.cols(); ++k) {
                    Eigen::Vector2d left, right;
                    left = i1d.col(k);
                    right = i2d.col(k);


                    auto fun = new ceres::DynamicAutoDiffCostFunction<DivisionDistortionAndFundamentalMatrixOptimizerFunctor>(
                            new DivisionDistortionAndFundamentalMatrixOptimizerFunctor(left, right,
                                                                                       static_cast<int>(number_of_distortion_coefficients),
                                                                                       options_.image_radius_));
                    fun->AddParameterBlock(static_cast<int>(number_of_distortion_coefficients));
                    fun->AddParameterBlock(8);
                    fun->SetNumResiduals(2);
                    problem.AddResidualBlock(fun, nullptr, lambda_ptr, f_ptr);
                    double res[2];
                    const double *ptrs[] = {lambda_ptr, f_ptr};
                    fun->Evaluate(ptrs,res, nullptr);
                    std::cout << res[0] << " : res " << std::endl;
                    ++residuals;
                }

            }
            std::cout << lambdas_.transpose() << " --- coefficients before estimation" << std::endl;
            ceres::Solver::Options options;
            //options.max_trust_region_radius = 0.01;
            options.max_num_iterations = 500;
            options.linear_solver_type = ceres::DENSE_QR;
            options.num_threads = 8;
            options.function_tolerance = 1e-16;
            options.parameter_tolerance = 1e-16;
            options.minimizer_progress_to_stdout = true;
            options.preconditioner_type = ceres::IDENTITY;
            options.jacobi_scaling = false;

            // Solve
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.BriefReport() << std::endl;
            std::cout << lambdas_ << " --- estimated coefficients" << std::endl;

        }

    }

    void NonLinearEstimator::getEstimationImpl(Eigen::VectorXd &result) {
        result = lambdas_;
    }

    void NonLinearEstimator::getEstimationImpl(scene::FundamentalMatrices &result) {
        for (auto &kth_fundamental_matrix : fs_) {
            utils::forceFundamentalRank(kth_fundamental_matrix);
        }
        result = fs_;
    }

    NonLinearEstimator::NonLinearEstimator(const scene::StdVector<scene::DynamicDivisionModelStereoPair> &stereo_pairs,
                                           NonLinearEstimatorOptions options) :
            stereo_pairs_(stereo_pairs),
            is_estimated_(false),
            options_(options) {
        number_of_pairs_ = stereo_pairs_.size();
        lambdas_ = stereo_pairs_[0].getLeftIntrinsicsPointer()->getDistortionCoefficients();

        fs_.resize(number_of_pairs_);
        fs_[0] = stereo_pairs_[0].getFundamentalMatrix();

        for (size_t k = 0; k < number_of_pairs_; ++k) {
            fs_[k] = stereo_pairs_[k].getFundamentalMatrix();
        }

    }

    bool NonLinearEstimator::isEstimated() const {
        return is_estimated_;
    }
}