cmake_minimum_required(VERSION 3.6)

project(Triangles3DTests)

find_package(GTest REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED on)

include_directories(
    ../Triangles/include
    ${GTEST_INCLUDE_DIRS}
)

add_executable(
  TestSerialization ./tests.cpp)

target_link_libraries(
	TestSerialization PUBLIC ${GTEST_LIBRARIES})

enable_testing()
add_test(TestSerialization "./TestSerialization")
