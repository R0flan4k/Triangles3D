# rm -rf build
# cmake -S . -B build -DCMAKE_CXX_COMPILER=g++-10 -DCMAKE_BUILD_TYPE=Release
# cmake --build ./build

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

