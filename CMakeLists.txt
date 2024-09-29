# rm -rf build
# cmake -S . -B build -DCMAKE_CXX_COMPILER=g++-10
# cmake --build ./build

# To run LFU cache tests:
# cmake --build ./build --target test ARGS="-C LFUTest"
# To run perfect cache tests:
# cmake --build ./build --target test ARGS="-C PerfectTest"

cmake_minimum_required(VERSION 3.11)
project(Triangles3D)
set(CMAKE_CXX_FLAGS "-O2 -std=c++20")


set(SRCS
	${CMAKE_SOURCE_DIR}/Triangles/src/Triangles.cpp
	${CMAKE_SOURCE_DIR}/Triangles/src/main.cpp
)
add_executable(Triangles.x)

target_sources(Triangles.x PRIVATE ${SRCS})
target_include_directories(Triangles.x PUBLIC "${CMAKE_SOURCE_DIR}/Triangles/include")

add_subdirectory(${CMAKE_SOURCE_DIR}/tests)

# set(TESTS "${CMAKE_SOURCE_DIR}/Tests")
# 
# file(GLOB testfiles "${TESTS}/*.dat")
# foreach(file ${testfiles})
#   add_test(
#     NAME ${file}
#     COMMAND bash -c "${CMAKE_SOURCE_DIR}/runtest.sh ${file} ./Triangles.x"
#     WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
# set_tests_properties(${file} PROPERTIES DEPENDS Triangles.x)
# endforeach()
