cmake_minimum_required(VERSION 3.14)
project(casadi_graph_feasibility_probe LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(PkgConfig REQUIRED)
find_package(TetraPGA CONFIG REQUIRED)
find_package(pinocchio REQUIRED)
find_package(benchmark REQUIRED)
pkg_check_modules(CASADI REQUIRED casadi)

add_executable(casadi_graph_probe casadi_graph_probe.cpp)

target_include_directories(casadi_graph_probe
  PRIVATE
    ${CASADI_INCLUDE_DIRS}
)

target_link_libraries(casadi_graph_probe
  PRIVATE
    TetraPGA::TetraPGA
    pinocchio::pinocchio
    benchmark::benchmark
    ${CASADI_LIBRARIES}
)

target_link_directories(casadi_graph_probe PRIVATE ${CASADI_LIBRARY_DIRS})

target_compile_definitions(casadi_graph_probe
  PRIVATE
    GA4RO_HAS_CASADI_BENCH=1
)
