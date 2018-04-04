//
// Created by danielbord on 2/19/18.
//

#ifndef CAMERA_CALIBRATION_ICAMERA_H
#define CAMERA_CALIBRATION_ICAMERA_H


#include <IIntrinsics.h>

namespace scene {

    /**
     * @brief Base class for camera
     * @tparam TDerived --- CRTP
     */
    template<typename TDerived, typename TScalar = typename TDerived::Scalar_t>
    struct ICamera {

        /**
         * @brief Interface to estimates camera parameters depends on type of TEstimator (intrinsics or its part, world coordinates, etc), for implemented types (see Camera.h)
         */
        template<typename TEstimator>
        void estimate(TEstimator &estimator) {
            static_cast<TDerived *>(this)->estimateImpl(estimator);
        }

        /***
         * @brief Expected to call suitable method of intrinsics model
         */
        scene::TImagePoint<TScalar> undistort(const scene::TImagePoint<TScalar> &pd) const {
            return static_cast<const TDerived *>(this)->undistortImpl(pd);
        }

        /***
         * @brief Expected to call suitable method of intrinsics model
         */
        scene::TImagePoint<TScalar> distort(const scene::TImagePoint<TScalar> &p) const {
            return static_cast<const TDerived *>(this)->distortImpl(p);
        }

        /***
         * @brief Expected to call suitable method of intrinsics model
         */
        scene::TImagePoint<TScalar> project(const scene::TWorldPoint<TScalar> &wp) const {
            return static_cast<const TDerived *>(this)->projectImpl(wp);
        }

        /***
         * @brief Expected to call suitable method of intrinsics model
         */
        scene::THomogeneousWorldPoint<TScalar> backproject(const scene::TImagePoint<TScalar> &p) const {
            return static_cast<const TDerived *>(this)->backprojectImpl(p);
        }
    };
}


#endif //CAMERA_CALIBRATION_ICAMERA_H
