cmake_minimum_required(VERSION 3.11)

project(Triangles3DTests)

find_package(GTest REQUIRED)
include(GoogleTest)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED on)

set(SRCS
	${CMAKE_CURRENT_SOURCE_DIR}/test.cpp
)

add_executable(
	TestSerialization ${CMAKE_CURRENT_SOURCE_DIR}/tests.cpp ${CMAKE_SOURCE_DIR}/Triangles/src/Triangles.cpp ${CMAKE_SOURCE_DIR}/Triangles/src/objects3d.cpp)

target_include_directories(TestSerialization PUBLIC
	${CMAKE_SOURCE_DIR}/Triangles/include
	${CMAKE_SOURCE_DIR}/Matrix/Matrix/include
	${GTEST_INCLUDE_DIRS}
)

target_link_libraries(
	TestSerialization PUBLIC ${GTEST_LIBRARIES} GTest::gtest GTest::gtest_main)

#add_test(NAME TestSerialization
#         COMMAND "./TestSerialization")

gtest_discover_tests(TestSerialization)
