cmake_minimum_required(VERSION 3.6)

project(Triangles3DTests)

find_package(GTest REQUIRED) # Not reliable, i'm going to replace it.

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED on)

set(SRCS
	${CMAKE_CURRENT_SOURCE_DIR}/test.cpp
)

include_directories(
	${CMAKE_SOURCE_DIR}/Triangles/include
	${GTEST_INCLUDE_DIRS}
)

add_executable(
	TestSerialization ${CMAKE_CURRENT_SOURCE_DIR}/tests.cpp ${CMAKE_SOURCE_DIR}/Triangles/src/Triangles.cpp)

target_link_libraries(
	TestSerialization PUBLIC ${GTEST_LIBRARIES} gtest gtest_main)

enable_testing()
add_test(TestSerialization "./TestSerialization")
