cmake_minimum_required(VERSION 3.15)
project(HGS_Test_cpp)
set(CMAKE_CXX_STANDARD 17)

add_executable(lib_test_cpp test.cpp)

IF (NOT WIN32)
  target_link_libraries(lib_test_cpp m)
ENDIF()
target_link_libraries(lib_test_cpp lib_static)
