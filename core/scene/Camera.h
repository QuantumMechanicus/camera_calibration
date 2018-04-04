//
// Created by danielbord on 1/22/18.
//

#ifndef CAMERA_CALIBRATION_CAMERA_H
#define CAMERA_CALIBRATION_CAMERA_H

#include <ICamera.h>
#include <INode.h>
#include "Intrinsics.h"

namespace scene {
    /**
     * @brief Class describing position (L = RW + t, where L denotes local coordinates system and W --- world coordinates system) and camera inner parameters
     * @tparam TIntrinsicsModel Parametrization of camera intrinsics model (pinhole, fisheye, etc.)
     */
    template<typename TIntrinsicsModel, typename TLabel = std::string, typename TScalar = double>
    class Camera
            : public ICamera<Camera<TIntrinsicsModel, TLabel, TScalar>, TScalar>,
              public graph::INode<Camera<TIntrinsicsModel, TLabel, TScalar>, TLabel> {

        friend class ICamera<Camera<TIntrinsicsModel, TLabel, TScalar>, TScalar>;

        friend graph::INode<Camera<TIntrinsicsModel, TLabel, TScalar>, TLabel>;

        std::shared_ptr<TIntrinsicsModel> intrinsics_;
        Sophus::SO3<TScalar> world_rotation_;
        Eigen::Matrix<TScalar, 3, 1> world_translation_;
        TLabel label_;

    protected:
        TLabel getLabelImpl() const {
            return label_;
        }

        template<typename TEstimator>
        void estimateImpl(TEstimator &estimator) {
            intrinsics_->estimateParameter(estimator);
        }

        void estimateImpl(estimators::AbstractEstimator<Sophus::SO3d> &estimator) {
            world_rotation_ = estimator.getEstimation();
        }

        void estimateImpl(std::shared_ptr<TIntrinsicsModel> simple_estimator) {
            intrinsics_ = simple_estimator;
        }

        //TODO more simple estimators == setters

        scene::TImagePoint<TScalar> undistortImpl(const scene::TImagePoint<TScalar> &pd) const {
            return intrinsics_->undistort(pd);
        }

       
        scene::TImagePoint<TScalar> distortImpl(const scene::TImagePoint<TScalar> &p) const {
            return intrinsics_->distort(p);
        }

        
        scene::TImagePoint<TScalar> projectImpl(const scene::WorldPoint &wp) const {
            return intrinsics_->project(wp);
        }

        
        scene::THomogeneousWorldPoint<TScalar> backprojectImpl(const scene::TImagePoint<TScalar> &p) const {
            return intrinsics_->backproject(p);
        }
        
    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using Model_t = TIntrinsicsModel;
        using Scalar_t = TScalar;

        Camera() : world_rotation_{}, world_translation_{}, label_{} {
            intrinsics_ = std::allocate_shared<TIntrinsicsModel>(Eigen::aligned_allocator<TIntrinsicsModel>());
        }

        Camera(Camera &&rhs) noexcept = default;

        Camera(const Camera &rhs) = default;

        Camera &operator=(const Camera &rhs) = default;

        Camera &operator=(Camera &&rhs) noexcept = default;

        /**
         * @brief Constructor
         * @param intrinsics Pointer to intrinsic parameters
         * @param rotation  Rotation element R of transform from world coordinates to local camera coordinates
         * @param translation Translation element t of transform from world coordinates to local camera coordinates
         */
        Camera(TLabel label, std::shared_ptr<TIntrinsicsModel> intrinsics, const Sophus::SO3<TScalar> &rotation,
               const Eigen::Matrix<TScalar, 3,1> &translation) : label_(std::move(label)), intrinsics_(std::move(intrinsics)), world_rotation_(rotation),
                                                     world_translation_(translation) {}


        /**
         * @brief Constructor
         * @param intrinsics Intrinsic parameters
         * @param rotation  Rotation element R of transform from world coordinates to local camera coordinates
         * @param translation Translation element t of transform from world coordinates to local camera coordinates
         */
        Camera(TLabel label, TIntrinsicsModel intrinsics, const Sophus::SO3<TScalar> &rotation,
               const Eigen::Matrix<TScalar, 3, 1> &translation) : label_(std::move(label)),
                                                     intrinsics_(std::allocate_shared<TIntrinsicsModel>(
                                                             Eigen::aligned_allocator<TIntrinsicsModel>())),
                                                     world_rotation_(rotation),
                                                     world_translation_(translation) {
            *intrinsics_ = intrinsics;
        }


        /**
         * @brief Another version of constructor
         */
        Camera(TLabel label, std::shared_ptr<TIntrinsicsModel> intrinsics) : label_(std::move(label)),
                                                                             intrinsics_(std::move(intrinsics)),
                                                                             world_rotation_() {
            world_translation_.setZero();
        }

        /**
         * @brief Another version of constructor
         */
        Camera(TLabel label, std::shared_ptr<const TIntrinsicsModel> intrinsics) : label_(std::move(label)),
                                                                                   intrinsics_(std::move(intrinsics)),
                                                                                   world_rotation_() {
            world_translation_.setZero();
        }


        /**
         * @brief Getter for intrinsic parameters
         * @return Pointer to intrinsics
         */
        const std::shared_ptr<TIntrinsicsModel> getIntrinsicsPointer() const {
            return intrinsics_;
        }

        /**
         * @brief Getter for intrinsic parameters
         * @return Intrinsic object
         */
        const TIntrinsicsModel &getIntrinsics() const {
            return *intrinsics_;
        }

        /**
         * @brief Getter for R
         * @return Sophus representation of SO3 (3D-rotation) group
         */
        const Sophus::SO3<TScalar> &getRotation() const {
            return world_rotation_;
        }

        /**
         * @brief Getter for t
         * @return 3D vector
         */
        const Eigen::Matrix<TScalar, 3, 1> &getTranslation() const {
            return world_translation_;
        }

        /**
         * @brief Getter for camera local coordinates
         * @return element of SE3d
         */
        Sophus::SE3<TScalar> getMotion() const {

            return Sophus::SE3<TScalar>(world_rotation_, world_translation_);
        }

        /**
         * @brief getter for height of image produced by camera
         * @return Height
         */
        unsigned int getHeight() const {
            return (*intrinsics_).getHeight();
        }

        /**
         * @brief getter for width of image produced by camera
         * @return Width
         */
        unsigned int getWidth() const {
            return (*intrinsics_).getWidth();
        }

    };

    using DynamicDivisionModelCamera = Camera<intrinsics::DynamicDivisionModel, std::string, double>;
    using StandartDivisionModelCamera = Camera<intrinsics::StandardDivisionModel, std::string, double>;
    using PinholeCamera = Camera<intrinsics::PinholeModel, std::string, double>;

}
#endif //CAMERA_CALIBRATION_CAMERA_H
