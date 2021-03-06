//
// Created by danielbord on 1/23/18.
//

#ifndef CAMERA_CALIBRATION_GROEBNER_ESTIMATOR_H
#define CAMERA_CALIBRATION_GROEBNER_ESTIMATOR_H

#include <Core.h>
#include <random>

namespace estimators {

    struct GroebnerEstimatorOptions {
        int number_of_RANSAC_iterations;
        double quantile_to_minimize;
        double lambda_lower_bound;
        double lambda_upper_bound;
        double image_radius;

        explicit GroebnerEstimatorOptions(int number_of_RANSAC_iterations = 1000000, double quantile_to_minimize = 0.1,
                                 double lambda_lower_bound = -1, double lambda_upper_bound = 0.25, double image_radius = 1);
    };

    class GroebnerDivisionModelEstimator
            : public estimators::AbstractEstimator<Eigen::Matrix<double,1,1>>,
              public estimators::AbstractEstimator<scene::FundamentalMatrix> {

        Eigen::Matrix<double, 1, 1> lambdas_;
        scene::StandartDivisionModelStereoPair stereo_pair_;
        scene::FundamentalMatrix fundamental_matrix_;
        long number_of_points_ ;
        GroebnerEstimatorOptions options_;
        std::vector<double> left_residuals_;
        std::vector<double> right_residuals_;
        bool is_estimated_;
        struct AutomaticSolver {

            typedef Eigen::Matrix<long double, 7, 1> GPolynomial;
            typedef Eigen::Matrix<double, 2, 8> EightPoints;
            typedef Eigen::Matrix<long double, 2, 8> EightPointsHighPrecision;

            typedef std::pair<scene::StdVector<scene::FundamentalMatrix>, std::vector<double> > FundamentalMatricesAndDistortionCoefficients;

            FundamentalMatricesAndDistortionCoefficients runSolver(EightPointsHighPrecision u1d, EightPointsHighPrecision u2d);

            FundamentalMatricesAndDistortionCoefficients solver(const GPolynomial &g1,
                                                                const GPolynomial &g2,
                                                                const GPolynomial &g3,
                                                                const GPolynomial &g4,
                                                                const GPolynomial &g5,
                                                                const GPolynomial &g6,
                                                                const GPolynomial &g7,
                                                                const GPolynomial &g8);
        };

        double computeErrorsAndEstimateQuantile(double distortion_coefficient,
                                                const scene::FundamentalMatrix &fundamental_matrix);

    public:
        explicit GroebnerDivisionModelEstimator(const scene::StandartDivisionModelStereoPair &stereo_pair, GroebnerEstimatorOptions opt = GroebnerEstimatorOptions());

        void setOptions(const GroebnerEstimatorOptions &options_);

        bool isEstimated() const override;

    protected:
        void estimateImpl() override;

        void getEstimationImpl(Eigen::Matrix<double,1,1> &result) override;

        void getEstimationImpl(scene::FundamentalMatrix &result) override;


    };
}

#endif //CAMERA_CALIBRATION_GROEBNER_ESTIMATOR_H
