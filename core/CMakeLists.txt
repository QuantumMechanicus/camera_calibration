FIND_PACKAGE(Ceres REQUIRED)
FIND_PACKAGE(Boost COMPONENTS program_options filesystem REQUIRED)
FIND_PACKAGE(Glog REQUIRED)

set(CORE_HEADERS
        interfaces/Abstract_Estimator.h
        interfaces/ICamera.h
        interfaces/IEdge.h
        interfaces/IGraph.h
        interfaces/IIntrinsics.h
        interfaces/INode.h
        interfaces/IScene.h
        interfaces/ITwo_View.h
        interfaces/Definitions.h
        utils/Utilities.h
        utils/Utilities.cpp
        utils/DivisionModelUtilities.h
        utils/Functors.h
        utils/Triangulation.h
        utils/Local_Parametrization_SO3.h
        utils/Local_Parametrization_Sphere.h
        utils/Scene_IO.h
        utils/Scene_IO.cpp
        scene/Camera.h
        scene/Intrinsics.h
        scene/Two_View.h
        scene/Scene.h)


add_library(core STATIC ${CORE_HEADERS})

target_include_directories(core PUBLIC ${CERES_LIBRARIES} ${Boost_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/interfaces
        ${CMAKE_CURRENT_SOURCE_DIR}/utils
        ${CMAKE_CURRENT_SOURCE_DIR}/submodules/Sophus)

target_link_libraries(core PUBLIC ${Boost_LIBRARIES} ${CERES_LIBRARIES} ${GLOG_LIBRARIES})



