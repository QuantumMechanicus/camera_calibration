add_library(test_utils test_utils.h test_utils.cpp)

target_include_directories(test_utils PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test_utils core)