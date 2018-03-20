//
// Created by danielbord on 1/22/18.
//

#ifndef CAMERA_CALIBRATION_TWO_VIEW_H
#define CAMERA_CALIBRATION_TWO_VIEW_H


#include "Camera.h"
#include <ceres/ceres.h>
#include <Triangulation.h>

namespace scene {


    template<typename TIntrinsicsModel, typename TLabel = std::string>
    class TwoView
            : public graph::AbstractEdge<scene::Camera<TIntrinsicsModel, TLabel>>, public ITwoView<
                    TwoView<TIntrinsicsModel, TLabel>> {

        friend class ITwoView<TwoView<TIntrinsicsModel, TLabel>>;

        Eigen::Vector3d relativeTranslation_{};
        Sophus::SO3d relativeRotation_{};
        ImagePoints left_keypoints_{};
        ImagePoints right_keypoints_{};
        FundamentalMatrix bifocal_tensor_{};
        Eigen::Matrix<double, 2, Eigen::Dynamic> i1d{}, i2d{};
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


        void estimateFundamentalMatrixImpl(estimators::AbstractEstimator<FundamentalMatrix> &estimator) {
            bifocal_tensor_ = estimator.getEstimation();
        }

        void estimateFundamentalMatrixImpl(const Eigen::Matrix3d &simple_estimation) {
            bifocal_tensor_ = simple_estimation;
        }

        long getNumberOfPointsImpl() const {
            return number_of_points_;
        }

        const ImagePoints &getLeftKeypointsImpl() const {
            return left_keypoints_;
        }

        const ImagePoints &getRightKeypointsImpl() const {
            return right_keypoints_;
        }

        const FundamentalMatrix &getFundamentalMatrixImpl() const {
            return bifocal_tensor_;
        }

        const FundamentalMatrix getEssentialMatrixImpl() const {

            auto left_K = getLeftIntrinsicsPointer()->getCalibrationMatrix();
            auto right_K = getRightIntrinsicsPointer()->getCalibrationMatrix();


            return right_K.transpose() * bifocal_tensor_ * left_K;
        }

        template <typename T>
        const Eigen::Vector3d getLeftEpilineImpl(const scene::TImagePoint<T> &u) const {
            return bifocal_tensor_.template cast<T>() * u.homogeneous();
        }

        template <typename T>
        const Eigen::Vector3d getRightEpilineImpl(const scene::TImagePoint<T> &u) const {
            return bifocal_tensor_.template cast<T>().transpose() * u.homogeneous();
        }

        template<typename T>
        scene::TImagePoint<T> undistortLeftImpl(const scene::TImagePoint<T> &pd) const {
            return getLeftIntrinsicsPointer()->undistort(pd);
        }

        template<typename T>
        scene::TImagePoint<T>undistortRightImpl(const scene::TImagePoint<T> &pd) const {
            return getRightIntrinsicsPointer()->undistort(pd);
        }

        template<typename T>
        scene::TImagePoint<T> distortLeftImpl(const scene::TImagePoint<T> &pd) const {
            return getLeftIntrinsicsPointer()->distort(pd);
        }

        template<typename T>
        scene::TImagePoint<T>distortRightImpl(const scene::TImagePoint<T> &pd) const {
            return getRightIntrinsicsPointer()->distort(pd);
        }

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using VertexMap_t = typename graph::AbstractEdge<scene::Camera<TIntrinsicsModel>>::VertexMap_t;

        TwoView() = default;

        TwoView(std::shared_ptr<VertexMap_t> cameras, TLabel left_camera_label,
                TLabel right_camera_label,
                const ImagePoints &left_keypoints,
                const ImagePoints &right_keypoints,
                const FundamentalMatrix &bifocal_tensor = FundamentalMatrix::Zero(),
                const Sophus::SO3d &relativeRotation = Sophus::SO3d(),
                const Eigen::Vector3d &relativeTranslation = Eigen::Vector3d::Zero())
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
                double r = std::sqrt(w * w + h * h) / 2.0;
                left_keypoints_.row(0) =
                        (left_keypoints_.row(0) - (w / 2.0) * Eigen::VectorXd::Ones(number_of_points_).transpose()) / r;
                left_keypoints_.row(1) =
                        (left_keypoints_.row(1) - (h / 2.0) * Eigen::VectorXd::Ones(number_of_points_).transpose()) / r;
                return true;
            }
            return false;

        }

        bool normalizeRightKeypoints() {

            auto &right_camera_ = this->getFinishVertex();
            double w = right_camera_.getWidth();
            double h = right_camera_.getHeight();
            if (w > 0 && h > 0) {
                double r = std::sqrt(w * w + h * h) / 2.0;
                right_keypoints_.row(0) =
                        (right_keypoints_.row(0) - (w / 2.0) * Eigen::VectorXd::Ones(number_of_points_).transpose()) /
                        r;
                right_keypoints_.row(1) =
                        (right_keypoints_.row(1) - (h / 2.0) * Eigen::VectorXd::Ones(number_of_points_).transpose()) /
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
                double r = std::sqrt(w * w + h * h) / 2.0;
                left_keypoints_.row(0) =
                        r * left_keypoints_.row(0) + (w / 2.0) * Eigen::VectorXd::Ones(number_of_points_).transpose();
                left_keypoints_.row(1) =
                        r * left_keypoints_.row(1) + (h / 2.0) * Eigen::VectorXd::Ones(number_of_points_).transpose();
                return true;
            }
            return false;
        }

        bool denormalizeRightKeypoints() {

            auto &right_camera_ = this->getFinishVertex();
            double w = right_camera_.getWidth();
            double h = right_camera_.getHeight();
            if (w > 0 && h > 0) {
                double r = std::sqrt(w * w + h * h) / 2.0;
                right_keypoints_.row(0) =
                        r * right_keypoints_.row(0) + (w / 2.0) * Eigen::VectorXd::Ones(number_of_points_).transpose();
                right_keypoints_.row(1) =
                        r * right_keypoints_.row(1) + (h / 2.0) * Eigen::VectorXd::Ones(number_of_points_).transpose();
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


            Eigen::Matrix3d matrix_D;

            matrix_D.setZero();

            Eigen::Matrix3d essential_matrix(this->getEssentialMatrix());


            matrix_D(0, 1) = 1;
            matrix_D(1, 0) = -1;
            matrix_D(2, 2) = 1;


            Eigen::JacobiSVD<Eigen::Matrix3d> svd(essential_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix3d matrixU = svd.matrixU();
            Eigen::Matrix3d matrixV = svd.matrixV();
            Eigen::Matrix3d singVal = svd.singularValues().asDiagonal();

            singVal(0, 0) = singVal(1, 1) = 1;
            singVal(2, 2) = 0;
            Eigen::Matrix3d original_E = essential_matrix;
            essential_matrix = matrixU * singVal * matrixV.transpose();

            if (matrixU.determinant() < 0)
                matrixU = -matrixU;
            if (matrixV.determinant() < 0)
                matrixV = -matrixV;

            Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Zero();
            Eigen::Matrix3d translation_matrix = Eigen::Matrix3d::Zero();
            Eigen::Vector3d translation_vector = matrixU.col(2);
            translation_vector.normalize();
            translation_matrix = utils::screw_hat(translation_vector);

            Eigen::Matrix3d current_rotation = Eigen::Matrix3d::Identity();
            Eigen::Matrix3d current_translation = translation_matrix;

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
                Sophus::SE3d leftToRight(current_rotation, utils::inverted_screw_hat(current_translation));

                double interval = utils::findChiralityInliers<TwoView<TIntrinsicsModel, TLabel>>(*this,
                                                                                                 0.1,
                                                                                                 inliers_ind,
                                                                                                 Eigen::Vector2d(
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
            relativeRotation_ = Sophus::SO3d(rotation_matrix);
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

        const Sophus::SO3d &getRelativeRotation() const {
            return relativeRotation_;
        }


        const Eigen::Vector3d &getRelativeTranslation() const {
            return relativeTranslation_;
        }

        const Sophus::SO3d &getLeftAbsoluteRotation() const {
            return this->getStartVertex().getRotation();
        }

        const Sophus::SO3d &getRightAbsoluteRotation() const {
            return this->getFinishVertex().getRotation();
        }

        const Eigen::Vector3d &getLeftAbsoluteTranslation() const {
            return this->getStartVertex().getTranslation();
        }

        const Eigen::Vector3d &getRightAbsoluteTranslation() const {
            return this->getFinishVertex().getTranslation();
        }


    };

    using StandartDivisionModelStereoPair = TwoView<intrinsics::StandardDivisionModel>;
    using DynamicDivisionModelStereoPair = TwoView<intrinsics::DynamicDivisionModel>;
}
#endif //CAMERA_CALIBRATION_TWO_VIEW_H