add_executable(test_core main.cpp test_intrinsics.cpp)


target_link_libraries(test_core PUBLIC test_utils PUBLIC libgtest PUBLIC libgmock)
target_include_directories(test_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/)

enable_testing()
add_test(name test_core command testCore "Test library core")
