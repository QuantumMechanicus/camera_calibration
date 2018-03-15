//
// Created by danielbord on 3/15/18.
//

#include "DivisionModelTriangulation.h"
namespace utils
{
    scene::WorldPoint baseTriangulation(const Sophus::SE3d &leftToRight,
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


}