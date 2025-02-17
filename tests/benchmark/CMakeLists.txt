cmake_minimum_required(VERSION 3.11)

project(Triangles3DBenchmark)

find_package(benchmark REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED on)

add_executable(
	BenchmarkSerialization ${CMAKE_CURRENT_SOURCE_DIR}/benchmark.cpp)

target_include_directories(BenchmarkSerialization PUBLIC
	${CMAKE_SOURCE_DIR}/Triangles/include
	${CMAKE_SOURCE_DIR}/Triangles/src
	${CMAKE_SOURCE_DIR}/Matrix/Matrix/include
	${benchmark_INCLUDE_DIRS}
)

target_link_libraries(
	BenchmarkSerialization PUBLIC ${benchmark_LIBRARIES} benchmark::benchmark_main)

#SET(GCC_LINK_FLAGS "-lbenchmark -lpthread")
#SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_LINK_FLAGS}")
target_link_libraries(BenchmarkSerialization PUBLIC benchmark::benchmark_main)

add_test(NAME BenchmarkSerialization
	COMMAND "$<TARGET_FILE:BenchmarkSerialization>")

