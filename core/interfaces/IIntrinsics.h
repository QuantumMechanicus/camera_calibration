//
// Created by danielbord on 2/28/18.
//

#ifndef CAMERA_CALIBRATION_IINTRINSICS_H
#define CAMERA_CALIBRATION_IINTRINSICS_H

#include <Abstract_Estimator.h>

namespace intrinsics {

    /***
     * DFOV --- diagonal field of view
     * HFOV --- horizontal field of view
     * VFOV --- vertical field of view
     */
    enum FieldOfViewType {
        DFOV,
        HFOV,
        VFOV,
    };


    /**
     * @brief Base class to store intrinsic parameters of camera (e. g. width, height, focal length)
     * @tparam TDerived --- CRTP
     */
    template<typename TDerived, typename TScalar = typename TDerived::Scalar_t>
    class AbstractIntrinsics {

    protected:
        unsigned int w_;
        unsigned int h_;
        double r_;

    public:
        /**
         * @brief Constructor
         * @param w Width of the image
         * @param h Height of the image
         */
        explicit AbstractIntrinsics(unsigned int w = 0, unsigned int h = 0) : w_(w), h_(h),
                                                                              r_(std::sqrt(w * w + h * h) / 2.0) {};

        /**
        * @brief Method for identifying unknown parameters of model
        * @param estimator Class with 'estimate' method or concrete estimation (see different implementations for details)
        */
        template<typename TEstimator>
        void estimateParameter(TEstimator &estimator) {
            static_cast<TDerived *>(this)->estimateParameterImpl(estimator);
        }

        /***
         * @brief Applies undistortion transform
         * @param ud Image point with distortion
         * @return undistorted image point
         */

        scene::TImagePoint<TScalar> undistort(const scene::TImagePoint<TScalar> &pd) const {
            return static_cast<const TDerived *>(this)->template undistortImpl(pd);
        }

        /***
         * @brief Applies distortion transform
         * @param ud Image point without distortion
         * @return distorted image point
         */

        scene::TImagePoint<TScalar> distort(const scene::TImagePoint<TScalar> &p) const {
            return static_cast<const TDerived *>(this)->template distortImpl(p);
        }

        /***
         * @brief Project world point to undistorted image space
         * @param wp Point in 3D world space
         * @return appropriate image point
         */

        scene::TImagePoint<TScalar> project(const scene::WorldPoint &wp) const {
            return static_cast<const TDerived *>(this)->template projectImpl(wp);
        }

        /***
         * @brief Backproject image point to correspondent ray in 3D space
         * @param p Point in undistorted image space
         * @return appropriate ray
         */

        scene::THomogeneousImagePoint<TScalar> backproject(const scene::TImagePoint<TScalar> &p) const {
            return static_cast<const TDerived *>(this)->template backprojectImpl(p);
        }

        /***
         * @brief Computes filed (angle) of view
         * @param t See FieldOfViewType
         * @return angle in degrees
         */
        double getFieldOfView(FieldOfViewType t) {
            return static_cast<const TDerived *>(this)->getFieldOfViewImpl(t);
        }


        /**
         * @brief Getter for width of the image
         * @return width of the image
         */
        unsigned int getWidth() const {
            return w_;
        }

        /**
         * @brief Getter for height of the image
         * @return height of the image
         */
        unsigned int getHeight() const {
            return h_;
        }

        /***
         * @brief Getter for radius of the image
         * @return radius of the image
         */
        double getImageRadius() const {
            return r_;
        }

        /**
        * @brief Equality operator
        * @param other Instance of intrinsics
        * @return Expected to be true if type of other and this the same and their fields are equal, otherwise false
        */
        bool operator==(const AbstractIntrinsics<TDerived, TScalar> &other) const {

            return static_cast<TDerived *>(this)->isEqualImpl(other);
        }

    };
}
#endif //CAMERA_CALIBRATION_IINTRINSICS_H
