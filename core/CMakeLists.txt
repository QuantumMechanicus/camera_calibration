FIND_PACKAGE(Ceres REQUIRED)
FIND_PACKAGE(Boost COMPONENTS program_options filesystem REQUIRED)
FIND_PACKAGE(Glog REQUIRED)

IF (0)
    add_library(core INTERFACE)
    target_sources(core INTERFACE
            utils/Utilities.h
            interfaces/Abstract_Estimator.h
            interfaces/ICamera.h
            interfaces/IEdge.h
            interfaces/IGraph.h
            interfaces/IIntrinsics.h
            interfaces/INode.h
            interfaces/IScene.h
            interfaces/ITwo_View.h
            interfaces/Definitions.h
            utils/DivisionModelUtilities.h
            utils/DivisionModelFunctors.h
            #utils/DivisionModelTriangulation.cpp
            utils/DivisionModelTriangulation.h
            scene/Camera.h
            scene/Intrinsics.h
            #scene/Scene.h
            #scene/Two_View.h
            )

    target_include_directories(core INTERFACE ${CERES_LIBRARIES} ${Boost_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/interfaces
            ${CMAKE_CURRENT_SOURCE_DIR}/utils
            ${CMAKE_CURRENT_SOURCE_DIR}/submodules/Sophus)
    target_link_libraries(core INTERFACE ${Boost_LIBRARIES} ${CERES_LIBRARIES} ${GLOG_LIBRARIES})
ENDIF ()

#TEMPORAL WORKAROUND (I DONT KNOW HOW USE CMAKE AND CLION PROPERLY!
IF (1)
    add_library(core STATIC
            utils/Utilities.h
            interfaces/Abstract_Estimator.h
            interfaces/ICamera.h
            interfaces/IEdge.h
            interfaces/IGraph.h
            interfaces/IIntrinsics.h
            interfaces/INode.h
            interfaces/IScene.h
            interfaces/ITwo_View.h
            interfaces/Definitions.h
            utils/DivisionModelUtilities.h
            utils/DivisionModelFunctors.h
            utils/DivisionModelTriangulation.cpp
            utils/DivisionModelTriangulation.h
            scene/Camera.h
            scene/Intrinsics.h
            #scene/Scene.h
            #scene/Two_View.h
            )

    target_include_directories(core PUBLIC ${CERES_LIBRARIES} ${Boost_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/interfaces
            ${CMAKE_CURRENT_SOURCE_DIR}/utils
            ${CMAKE_CURRENT_SOURCE_DIR}/submodules/Sophus)
    target_link_libraries(core PUBLIC ${Boost_LIBRARIES} ${CERES_LIBRARIES} ${GLOG_LIBRARIES})
ENDIF ()


