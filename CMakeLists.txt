cmake_minimum_required(VERSION 3.10)
project(icts)
set (CMAKE_CXX_STANDARD 17)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
add_subdirectory(library/yaml-cpp)

add_library(icts INTERFACE)
target_include_directories(
  icts
  INTERFACE 
  include 
  library/yaml-cpp/include
)

add_executable(
  icts_mapf
  example/icts_test.cpp
)
target_link_libraries(
  icts_mapf
  icts
  yaml-cpp
  ${Boost_LIBRARIES}
)