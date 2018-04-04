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
    template<typename TDerived, typename TScalar = typename TDerived::Scalar_t>
    struct ITwoView {
    public:

        //TODO forward
        /**
         * @brief Estimate two view parameters
         */
        template<typename TEstimator>
        void estimate(TEstimator &estimator) {
            static_cast<TDerived *>(this)->estimateImpl(estimator);
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


        scene::TImagePoint<TScalar> undistortLeft(const scene::TImagePoint<TScalar> &pd) const {
            return static_cast<const TDerived *>(this)->template undistortLeftImpl(pd);
        }


        scene::TImagePoint<TScalar> undistortRight(const scene::TImagePoint<TScalar> &pd) const {
            return static_cast<const TDerived *>(this)->template undistortRightImpl(pd);
        }


        scene::TImagePoint<TScalar> distortLeft(const scene::TImagePoint<TScalar> &pd) const {
            return static_cast<const TDerived *>(this)->template distortLeftImpl(pd);
        }


        scene::TImagePoint<TScalar> distortRight(const scene::TImagePoint<TScalar> &pd) const {
            return static_cast<const TDerived *>(this)->template distortRightImpl(pd);
        }


        scene::TImagePoint<TScalar> projectLeft(const scene::TWorldPoint<TScalar> &wp) const {
            return static_cast<const TDerived *>(this)->template projectLeftImpl(wp);
        }


        scene::HomogeneousWorldPoint backprojectLeft(const scene::TImagePoint<TScalar> &p) const {
            return static_cast<const TDerived *>(this)->template backprojectLeftImpl(p);
        }


        scene::TImagePoint<TScalar> projectRight(const scene::TWorldPoint<TScalar> &wp) const {
            return static_cast<const TDerived *>(this)->template projectRightImpl(wp);
        }


        scene::THomogeneousWorldPoint<TScalar> backprojectRight(const scene::TImagePoint<TScalar> &p) const {
            return static_cast<const TDerived *>(this)->template backprojectRightImpl(p);
        }


        const Eigen::Matrix<TScalar, 3, 1> getLeftEpiline(const scene::TImagePoint<TScalar> &u) const {
            return static_cast<const TDerived *>(this)->template getLeftEpilineImpl(u);
        }


        const Eigen::Matrix<TScalar, 3, 1> getRightEpiline(const scene::TImagePoint<TScalar> &u) const {
            return static_cast<const TDerived *>(this)->template getRightEpilineImpl(u);
        }

        template<typename T>
        Sophus::SE3<T> getRelativeMotion() const {
            return static_cast<const TDerived *>(this)->template getRelativeMotionImpl();
        }


    };

}


#endif //CAMERA_CALIBRATION_ITWO_VIEW_H
