cmake_minimum_required(VERSION 3.6)

project(Triangles3DBenchmark)

find_package(benchmark REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED on)

include_directories(
	${CMAKE_SOURCE_DIR}/Triangles/include
	${benchmark_INCLUDE_DIRS}
)

add_executable(
	BenchmarkSerialization ${CMAKE_CURRENT_SOURCE_DIR}/benchmark.cpp ${CMAKE_SOURCE_DIR}/Triangles/src/Triangles.cpp)

target_link_libraries(
	BenchmarkSerialization PUBLIC ${benchmark_LIBRARIES} benchmark benchmark_main)

SET(GCC_LINK_FLAGS "-isystem /Users/yhr/Programs/benchmark/include -lbenchmark -pthread")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_LINK_FLAGS}")

enable_testing()
add_test(BenchmarkSerialization "./BenchmarkSerialization")
