//
// Created by danielbord on 2/19/18.
//

#ifndef CAMERA_CALIBRATION_ITWO_VIEW_H
#define CAMERA_CALIBRATION_ITWO_VIEW_H


#include <ICamera.h>

namespace scene {
    /**
     * @brief Base class for stereo pair
     * @tparam TDerived --- CRTP
     */
    template<typename TDerived>
    struct ITwoView {
    public:

        //TODO forward
        /**
         * @brief Estimate fundamental matrix via estimator
         */
        template<typename TEstimator>
        void estimateFundamentalMatrix(TEstimator &estimator) {
            static_cast<TDerived *>(this)->estimateFundamentalMatrixImpl(estimator);
        }

        /**
         * @brief Estimate any parameter of left camera (e.g. intrinsics, extrinsic), see implementation for details
         */
        template<typename TEstimator>
        void estimateLeftCamera(TEstimator &estimator) {
            static_cast<TDerived *>(this)->estimateLeftCameraImpl(estimator);
        }

        /**
         * @brief Estimate any parameter of right camera (e.g. intrinsics, extrinsic), see implementation for details
         */
        template<typename TEstimator>
        void estimateRightCamera(TEstimator &estimator) {
            static_cast<TDerived *>(this)->estimateRightCameraImpl(estimator);
        }

        template<typename T>
        scene::TImagePoint<T> undistortLeft(const scene::TImagePoint<T> &pd) const {
            return static_cast<const TDerived *>(this)->template undistortLeftImpl<T>(pd);
        }

        template<typename T>
        scene::TImagePoint<T> undistortRight(const scene::TImagePoint<T> &pd) const {
            return static_cast<const TDerived *>(this)->template undistortRightImpl<T>(pd);
        }

        template<typename T>
        scene::TImagePoint<T> distortLeft(const scene::TImagePoint<T> &pd) const {
            return static_cast<const TDerived *>(this)->template distortLeftImpl<T>(pd);
        }

        template<typename T>
        scene::TImagePoint<T> distortRight(const scene::TImagePoint<T> &pd) const {
            return static_cast<const TDerived *>(this)->template distortRightImpl<T>(pd);
        }

        template<typename T>
        scene::TImagePoint<T> projectLeft(const scene::TWorldPoint<T> &wp) const {
            return static_cast<const TDerived *>(this)->template projectLeftImpl<T>(wp);
        }

        template<typename T>
        scene::THomogeneousWorldPoint<T> backprojectLeft(const scene::TImagePoint<T> &p) const {
            return static_cast<const TDerived *>(this)->template backprojectLeftImpl<T>(p);
        }

        template<typename T>
        scene::TImagePoint<T> projectRight(const scene::TWorldPoint<T> &wp) const {
            return static_cast<const TDerived *>(this)->template projectRightImpl<T>(wp);
        }

        template<typename T>
        scene::THomogeneousWorldPoint<T> backprojectRight(const scene::TImagePoint<T> &p) const {
            return static_cast<const TDerived *>(this)->template backprojectRightImpl<T>(p);
        }

        template<typename T>
        Sophus::SE3<T> getRelativeMotion() const {
            return static_cast<const TDerived *>(this)->template getRelativeMotionImpl();
        }

        long getNumberOfPoints() const {
            return static_cast<const TDerived *>(this)->getNumberOfPointsImpl();
        }

        const ImagePoints &getLeftKeypoints() const {
            return static_cast<const TDerived *>(this)->getLeftKeypointsImpl();
        }

        const ImagePoints &getRightKeypoints() const {
            return static_cast<const TDerived *>(this)->getRightKeypointsImpl();
        }

        const FundamentalMatrix &getFundamentalMatrix() const {
            return static_cast<const TDerived *>(this)->getFundamentalMatrixImpl();
        }

        const FundamentalMatrix getEssentialMatrix() const {
            return static_cast<const TDerived *>(this)->getEssentialMatrixImpl();
        }

        template <typename T>
        const Eigen::Vector3d getLeftEpiline(const scene::TImagePoint<T> &u) const {
            return static_cast<const TDerived *>(this)->template getLeftEpilineImpl<T>(u);
        }

        template <typename T>
        const Eigen::Vector3d getRightEpiline(const scene::TImagePoint<T> &u) const {
            return static_cast<const TDerived *>(this)->template getRightEpilineImpl<T>(u);
        }


    };

}


#endif //CAMERA_CALIBRATION_ITWO_VIEW_H
