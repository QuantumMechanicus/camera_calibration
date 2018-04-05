//
// Created by danielbord on 2/21/18.
//

#ifndef CAMERA_CALIBRATION_SCENE_H
#define CAMERA_CALIBRATION_SCENE_H

#include <IScene.h>
#include "Two_View.h"


namespace scene {

    template<typename TCamera, typename TTwoView = TwoView<typename TCamera::Model_t,
            typename TCamera::Label_t, typename TCamera::Scalar_t>>
    class Scene : public IScene<Scene<TCamera, TTwoView>> {
        friend class IScene<Scene<TCamera, TTwoView>>;

        std::shared_ptr<typename TTwoView::VertexMap_t> ptr_to_map_;
        scene::StdVector<TTwoView> list_of_stereo_pairs_;
        //TODO approve TwoViews ptr_to_map and scene ptr_to_map

    protected:

        template <typename TEstimator>
        void estimateCameraImpl(const typename TCamera::Label_t &label,
                                TEstimator &estimator) {
            (*ptr_to_map_)[label].estimate(estimator);
        }

        template <typename TEstimator>
        void estimateStereoPairImpl(size_t k, TEstimator &estimator) {
            list_of_stereo_pairs_[k].estimate(estimator);
        }

        template <typename TEstimator>
        void
        estimateStereoPairsImpl(
                estimators::AbstractEstimator<scene::StdVector<TEstimator>> &estimator) {
            auto result = estimator.getEstimation();
            assert(result.size() >= list_of_stereo_pairs_.size() &&
                   "Number of estimators should me no less than number of stereo pairs");
            for (size_t k = 0; k < list_of_stereo_pairs_.size(); ++k)
                list_of_stereo_pairs_[k].estimate(result[k]);
        }

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using Camera_t = TCamera;

        Scene(std::shared_ptr<typename TTwoView::VertexMap_t> ptr_to_map, scene::StdVector <TTwoView> list_of_stereo_pairs)
                : ptr_to_map_(std::move(ptr_to_map)), list_of_stereo_pairs_(std::move(list_of_stereo_pairs)) {}

        template<typename SceneArchiver>
        void saveScene(const std::vector<SceneArchiver> &serializators) const {
            assert(serializators.size() >= list_of_stereo_pairs_.size() &&
                   "Number of serializators should me no less than number of stereo pairs");
            for (size_t k = 0; k < list_of_stereo_pairs_.size(); ++k)
                serializators[k].serialize(list_of_stereo_pairs_[k]);
        }

        template<typename SceneArchiver>
        void loadScene(const std::vector<SceneArchiver> &serializators,
                       std::shared_ptr<typename TTwoView::VertexMap_t> ptr_to_map) {
            ptr_to_map_ = ptr_to_map;
            list_of_stereo_pairs_.resize(serializators.size());
            for (size_t k = 0; k < list_of_stereo_pairs_.size(); ++k)
                serializators[k].deserialize(list_of_stereo_pairs_[k], ptr_to_map);
        }

        template<typename SceneArchiver>
        void loadScene(const std::vector<SceneArchiver> &serializators) {
            list_of_stereo_pairs_.resize(serializators.size());
            for (size_t k = 0; k < list_of_stereo_pairs_.size(); ++k)
                serializators[k].deserialize(list_of_stereo_pairs_[k], ptr_to_map_);
        }


    };

    using StandartDivisionModelScene = Scene<StandartDivisionModelCamera, StandartDivisionModelStereoPair>;
    using DynamicDivisionModelScene = Scene<DynamicDivisionModelCamera, DynamicDivisionModelStereoPair>;
}

#endif //CAMERA_CALIBRATION_SCENE_H
