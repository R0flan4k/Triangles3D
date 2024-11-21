cmake_minimum_required(VERSION 3.11)

project(Triangles3D)

enable_testing()
set(TRIANGLES_TESTS "${CMAKE_CURRENT_SOURCE_DIR}/data")

file(GLOB testfiles "${TRIANGLES_TESTS}/*.dat")
message("${CMAKE_BINARY_DIR}")
foreach(file ${testfiles})
  add_test(
    NAME ${file}
    CONFIGURATIONS e2e
    COMMAND bash -c "${CMAKE_CURRENT_SOURCE_DIR}/runtest.sh ${file} ./Triangles.x"
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
set_tests_properties(${file} PROPERTIES DEPENDS Triangles.x)
endforeach()
