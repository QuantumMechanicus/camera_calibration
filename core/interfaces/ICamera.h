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
     * @tparam TIntrinsics --- Intrinsics model
     */
    template<typename TDerived, typename TIntrinsics>
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
        scene::ImagePoint undistort(const scene::ImagePoint &pd) {
            return static_cast<TDerived *>(this)->undistortImpl(pd);
        }

        /***
         * @brief Expected to call suitable method of intrinsics model
         */
        scene::ImagePoint distort(const scene::ImagePoint &p) {
            return static_cast<TDerived *>(this)->distortImpl(p);
        }

        /***
         * @brief Expected to call suitable method of intrinsics model
         */
        scene::ImagePoint project(const scene::WorldPoint &wp) {
            return static_cast<TDerived *>(this)->projectImpl(wp);
        }

        /***
         * @brief Expected to call suitable method of intrinsics model
         */
        scene::HomogenousWorldPoint backproject(const scene::ImagePoint &p) {
            return static_cast<TDerived *>(this)->backprojectImpl(p);
        }
    };
}


#endif //CAMERA_CALIBRATION_ICAMERA_H
