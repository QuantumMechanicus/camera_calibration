//
// Created by danielbord on 3/19/18.
//
#include "Utilities.h"

namespace utils {
    double estimateQuantile(std::vector<double> errors,
                            double expected_percent_of_inliers) {

        std::nth_element(errors.begin(), errors.begin() + int(errors.size() * expected_percent_of_inliers),
                         errors.end());
        double quantile = errors[int(errors.size() * expected_percent_of_inliers)];
        return quantile;
    }

    double estimateConfidenceInterval(double quantile, double expected_percent_of_inliers) {
        return 10;/*quantile * boost::math::erfc_inv((0.95 + 1.0)) /
               boost::math::erfc_inv((expected_percent_of_inliers + 1.0));*/

    }
}