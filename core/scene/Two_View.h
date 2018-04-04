//
// Created by danielbord on 1/22/18.
//

#ifndef CAMERA_CALIBRATION_TWO_VIEW_H
#define CAMERA_CALIBRATION_TWO_VIEW_H


#include "Camera.h"
#include <ceres/ceres.h>
#include <Triangulation.h>

namespace scene {


    template<typename TIntrinsicsModel, typename TLabel = std::string, typename TScalar = typename TIntrinsicsModel::Scalar_t>
    class TwoView
            : public graph::AbstractEdge<scene::Camera<TIntrinsicsModel, TLabel, TScalar>>, public ITwoView<
                    TwoView<TIntrinsicsModel, TLabel, TScalar>, TScalar> {

        friend class ITwoView<TwoView<TIntrinsicsModel, TLabel, TScalar>, TScalar>;
        TImagePoints<TScalar> left_keypoints_{};
        TImagePoints<TScalar> right_keypoints_{};
        TFundamentalMatrix<TScalar> bifocal_tensor_{};
        Sophus::SO3<TScalar> relativeRotation_{};
        Eigen::Matrix<TScalar, 3, 1> relativeTranslation_{};
        //Eigen::Matrix<TScalar, 2, Eigen::Dynamic> i1d{}, i2d{};
        long number_of_points_{};


    protected:

        template<typename TEstimator>
        void estimateLeftCameraImpl(TEstimator &estimator) {
            this->ptr_to_list_of_vertices_->at(this->start_vertex_label_).estimate(estimator);
        }

        template<typename TEstimator>
        void estimateRightCameraImpl(TEstimator &estimator) {
            this->ptr_to_list_of_vertices_->at(this->end_vertex_label_).estimate(estimator);
        }


        void estimateImpl(estimators::AbstractEstimator<FundamentalMatrix> &estimator) {
            bifocal_tensor_ = estimator.getEstimation();
        }

        void estimateImpl(const Eigen::Matrix<TScalar, 3, 3> &simple_estimation) {
            bifocal_tensor_ = simple_estimation;
        }


        const Eigen::Matrix<TScalar, 3, 1> getRightEpilineImpl(const scene::TImagePoint<TScalar> &u) const {
            return bifocal_tensor_.template cast<TScalar>() * u.homogeneous();
        }


        const Eigen::Matrix<TScalar, 3, 1> getLeftEpilineImpl(const scene::TImagePoint<TScalar> &u) const {
            return bifocal_tensor_.template cast<TScalar>().transpose() * u.homogeneous();
        }


        scene::TImagePoint<TScalar> undistortLeftImpl(const scene::TImagePoint<TScalar> &pd) const {
            return getLeftIntrinsicsPointer()->undistort(pd);
        }


        scene::TImagePoint<TScalar> undistortRightImpl(const scene::TImagePoint<TScalar> &pd) const {
            return getRightIntrinsicsPointer()->undistort(pd);
        }


        scene::TImagePoint<TScalar> distortLeftImpl(const scene::TImagePoint<TScalar> &pd) const {
            return getLeftIntrinsicsPointer()->distort(pd);
        }


        scene::TImagePoint<TScalar> distortRightImpl(const scene::TImagePoint<TScalar> &pd) const {
            return getRightIntrinsicsPointer()->distort(pd);
        }

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using VertexMap_t = typename graph::AbstractEdge<scene::Camera<TIntrinsicsModel>>::VertexMap_t;
        using Scalar_t = TScalar;

        TwoView() = default;

        TwoView(std::shared_ptr<VertexMap_t> cameras, TLabel left_camera_label,
                TLabel right_camera_label,
                const TImagePoint<TScalar> &left_keypoints,
                const TImagePoint<TScalar> &right_keypoints,
                const TFundamentalMatrix<TScalar> &bifocal_tensor = TFundamentalMatrix<TScalar>::Zero(),
                const Sophus::SO3<TScalar> &relativeRotation = Sophus::SO3<TScalar>(),
                const Eigen::Matrix<TScalar, 3, 1> &relativeTranslation = Eigen::Matrix<TScalar, 3, 1>::Zero())
                : graph::AbstractEdge<scene::Camera<TIntrinsicsModel>>(
                std::move(left_camera_label),
                std::move(right_camera_label), cameras),
                  left_keypoints_(left_keypoints),
                  right_keypoints_(right_keypoints),
                  bifocal_tensor_(bifocal_tensor),
                  relativeRotation_(relativeRotation),
                  relativeTranslation_(relativeTranslation) {
            number_of_points_ = TwoView::left_keypoints_.cols();

        }


        bool normalizeLeftKeypoints() {

            auto &left_camera_ = this->getStartVertex();
            double w = left_camera_.getWidth();
            double h = left_camera_.getHeight();
            if (w > 0 && h > 0) {
                TScalar r = TScalar(std::sqrt(w * w + h * h) / 2.0);
                left_keypoints_.row(0) =
                        (left_keypoints_.row(0) - (w / TScalar(2.0)) * Eigen::VectorXd::Ones(
                                number_of_points_).template cast<TScalar>().transpose()) / r;
                left_keypoints_.row(1) =
                        (left_keypoints_.row(1) - (h / TScalar(2.0)) * Eigen::VectorXd::Ones(
                                number_of_points_).template cast<TScalar>().transpose()) / r;
                return true;
            }
            return false;

        }

        bool normalizeRightKeypoints() {

            auto &right_camera_ = this->getFinishVertex();
            double w = right_camera_.getWidth();
            double h = right_camera_.getHeight();
            if (w > 0 && h > 0) {
                TScalar r = TScalar(std::sqrt(w * w + h * h) / 2.0);
                right_keypoints_.row(0) =
                        (right_keypoints_.row(0) - (w / TScalar(2.0)) * Eigen::VectorXd::Ones(
                                number_of_points_).template cast<TScalar>().transpose()) /
                        r;
                right_keypoints_.row(1) =
                        (right_keypoints_.row(1) - (h / TScalar(2.0)) * Eigen::VectorXd::Ones(
                                number_of_points_).template cast<TScalar>().transpose()) /
                        r;
                return true;
            }
            return false;

        }

        bool denormalizeLeftKeypoints() {

            auto &left_camera_ = this->getStartVertex();
            double w = left_camera_.getWidth();
            double h = left_camera_.getHeight();
            if (w > 0 && h > 0) {
                TScalar r = TScalar(std::sqrt(w * w + h * h) / 2.0);
                left_keypoints_.row(0) =
                        r * left_keypoints_.row(0) + (w / TScalar(2.0)) * Eigen::VectorXd::Ones(
                                number_of_points_).template cast<TScalar>().transpose();
                left_keypoints_.row(1) =
                        r * left_keypoints_.row(1) + (h / TScalar(2.0)) * Eigen::VectorXd::Ones(
                                number_of_points_).template cast<TScalar>().transpose();
                return true;
            }
            return false;
        }

        bool denormalizeRightKeypoints() {

            auto &right_camera_ = this->getFinishVertex();
            double w = right_camera_.getWidth();
            double h = right_camera_.getHeight();
            if (w > 0 && h > 0) {
                TScalar r = TScalar(std::sqrt(w * w + h * h) / 2.0);
                right_keypoints_.row(0) =
                        r * right_keypoints_.row(0) + (w / TScalar(2.0)) * Eigen::VectorXd::Ones(
                                number_of_points_).template cast<TScalar>().transpose();
                right_keypoints_.row(1) =
                        r * right_keypoints_.row(1) + (h / TScalar(2.0)) * Eigen::VectorXd::Ones(
                                number_of_points_).template cast<TScalar>().transpose();
                return true;
            }
            return false;
        }


        template<typename SceneArchiver>
        void saveScene(const SceneArchiver &serializator) const {
            serializator.serialize(*this);
        }

        template<typename SceneArchiver>
        void loadScene(const SceneArchiver &serializator,
                       std::shared_ptr<VertexMap_t> ptr_to_list_of_vertices) {
            serializator.deserialize(*this, ptr_to_list_of_vertices);
        }

        template<typename SceneArchiver>
        void loadScene(const SceneArchiver &serializator) {
            serializator.deserialize(*this, this->ptr_to_list_of_vertices_);
        }


        void recoverRelativeMotion() {


            Eigen::Matrix<TScalar, 3, 3> matrix_D;

            matrix_D.setZero();

            Eigen::Matrix<TScalar, 3, 3> essential_matrix(this->getEssentialMatrix());


            matrix_D(0, 1) = TScalar(1);
            matrix_D(1, 0) = TScalar(-1);
            matrix_D(2, 2) = TScalar(1);


            Eigen::JacobiSVD<Eigen::Matrix<TScalar, 3, 3>> svd(essential_matrix,
                                                               Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix<TScalar, 3, 3> matrixU = svd.matrixU();
            Eigen::Matrix<TScalar, 3, 3> matrixV = svd.matrixV();
            Eigen::Matrix<TScalar, 3, 3> singVal = svd.singularValues().asDiagonal();

            singVal(0, 0) = singVal(1, 1) = TScalar(1);
            singVal(2, 2) = TScalar(0);
            Eigen::Matrix<TScalar, 3, 3> original_E = essential_matrix;
            essential_matrix = matrixU * singVal * matrixV.transpose();

            if (matrixU.determinant() < 0)
                matrixU = -matrixU;
            if (matrixV.determinant() < 0)
                matrixV = -matrixV;

            Eigen::Matrix<TScalar, 3, 3> rotation_matrix = Eigen::Matrix<TScalar, 3, 3>::Zero();
            Eigen::Matrix<TScalar, 3, 3> translation_matrix = Eigen::Matrix<TScalar, 3, 3>::Zero();
            Eigen::Matrix<TScalar, 3, 1> translation_vector = matrixU.col(2);
            translation_vector.normalize();
            translation_matrix = utils::screw_hat(translation_vector);

            Eigen::Matrix<TScalar, 3, 3> current_rotation = Eigen::Matrix<TScalar, 3, 3>::Identity();
            Eigen::Matrix<TScalar, 3, 3> current_translation = translation_matrix;

            int max_counter = 0;
            for (std::size_t k = 0; k < 4; ++k) {
                int counter = 0;
                switch (k) {
                    case 0:
                        current_rotation = matrixU * matrix_D * matrixV.transpose();
                        break;
                    case 1:
                        current_rotation = matrixU * matrix_D * matrixV.transpose();
                        current_translation.transposeInPlace();
                        break;
                    case 2:
                        current_rotation = matrixU * matrix_D.transpose() * matrixV.transpose();
                        current_translation.transposeInPlace();
                        break;
                    case 3:
                        current_rotation = matrixU * matrix_D.transpose() * matrixV.transpose();
                        current_translation.transposeInPlace();
                        break;
                    default:
                        break;
                }

                std::vector<size_t> inliers_ind;
                Sophus::SE3<TScalar> leftToRight(current_rotation, utils::inverted_screw_hat(current_translation));

                double interval = utils::findChiralityInliers<TwoView<TIntrinsicsModel, TLabel>>(*this,
                                                                                                 0.1,
                                                                                                 inliers_ind,
                                                                                                 Eigen::Matrix<TScalar, 2, 1>(
                                                                                                         getLeftIntrinsicsPointer()->getWidth(),
                                                                                                         getLeftIntrinsicsPointer()->getHeight()).norm() /
                                                                                                 2.0);


                counter = inliers_ind.size();
                LOG(INFO) << "Number of points in front of camera " << counter << " Max: " << max_counter;

                if (counter > max_counter) {
                    rotation_matrix = current_rotation;
                    translation_matrix = current_translation;
                    max_counter = counter;
                }
            }
            relativeRotation_ = Sophus::SO3<TScalar>(rotation_matrix);
            relativeTranslation_ = utils::inverted_screw_hat(translation_matrix).normalized();

        }

        const std::shared_ptr<TIntrinsicsModel> getLeftIntrinsicsPointer() const {
            return this->getStartVertex().getIntrinsicsPointer();
        }

        const std::shared_ptr<TIntrinsicsModel> getRightIntrinsicsPointer() const {
            return this->getFinishVertex().getIntrinsicsPointer();
        }

        const TIntrinsicsModel &getLeftIntrinsics() const {
            return this->getStartVertex().getIntrinsics();
        }

        const TIntrinsicsModel &getRightIntrinsics() const {
            return this->getFinishVertex().getIntrinsics();
        }

        const Sophus::SO3<TScalar> &getRelativeRotation() const {
            return relativeRotation_;
        }


        const Eigen::Matrix<TScalar, 3, 1> &getRelativeTranslation() const {
            return relativeTranslation_;
        }

        const Sophus::SO3<TScalar> &getLeftAbsoluteRotation() const {
            return this->getStartVertex().getRotation();
        }

        const Sophus::SO3<TScalar> &getRightAbsoluteRotation() const {
            return this->getFinishVertex().getRotation();
        }

        const Eigen::Matrix<TScalar, 3, 1> &getLeftAbsoluteTranslation() const {
            return this->getStartVertex().getTranslation();
        }

        const Eigen::Matrix<TScalar, 3, 1> &getRightAbsoluteTranslation() const {
            return this->getFinishVertex().getTranslation();
        }

        long getNumberOfPoints() const {
            return number_of_points_;
        }

        const TImagePoints<TScalar> &getLeftKeypoints() const {
            return left_keypoints_;
        }

        const TImagePoints<TScalar> &getRightKeypoints() const {
            return right_keypoints_;
        }

        const TFundamentalMatrix<TScalar> &getFundamentalMatrix() const {
            return bifocal_tensor_;
        }

        const TFundamentalMatrix<TScalar> getEssentialMatrix() const {
            auto left_K = getLeftIntrinsicsPointer()->getCalibrationMatrix();
            auto right_K = getRightIntrinsicsPointer()->getCalibrationMatrix();


            return right_K.transpose() * bifocal_tensor_ * left_K;
        }


    };

    using StandartDivisionModelStereoPair = TwoView<intrinsics::StandardDivisionModel, std::string, double>;
    using DynamicDivisionModelStereoPair = TwoView<intrinsics::DynamicDivisionModel, std::string, double>;
}
#endif //CAMERA_CALIBRATION_TWO_VIEW_H
