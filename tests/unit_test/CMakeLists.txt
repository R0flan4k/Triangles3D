cmake_minimum_required(VERSION 3.11)

project(Triangles3DTests)

find_package(GTest REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED on)

set(SRCS
	${CMAKE_CURRENT_SOURCE_DIR}/test.cpp
)

include_directories(
	${CMAKE_SOURCE_DIR}/Triangles/include
	${CMAKE_SOURCE_DIR}/Matrix/Matrix/include
	${GTEST_INCLUDE_DIRS}
)

add_executable(
	TestSerialization ${CMAKE_CURRENT_SOURCE_DIR}/tests.cpp ${CMAKE_SOURCE_DIR}/Triangles/src/Triangles.cpp)

target_link_libraries(
	TestSerialization PUBLIC ${GTEST_LIBRARIES} GTest::gtest GTest::gtest_main)

enable_testing()
add_test(NAME TestSerialization
	 CONFIGURATIONS unit
         COMMAND "./TestSerialization")
