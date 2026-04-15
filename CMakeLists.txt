cmake_minimum_required(VERSION 3.14)
project(TetraPGA VERSION 0.1.0)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(COAL_DISABLE_HPP_FCL_WARNINGS TRUE)
add_compile_definitions(COAL_DISABLE_HPP_FCL_WARNINGS)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_SKIP_BUILD_RPATH TRUE)

option(TETRAPGA_BUILD_TESTS "Build TetraPGA test executables" ON)
option(TETRAPGA_BUILD_BENCHMARKS "Build TetraPGA benchmark executables" OFF)

find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)

add_library(TetraPGA INTERFACE)
add_library(TetraPGA::TetraPGA ALIAS TetraPGA)

target_include_directories(TetraPGA
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${EIGEN3_INCLUDE_DIR}
    ${urdfdom_INCLUDE_DIRS}
)

target_link_libraries(TetraPGA INTERFACE Eigen3::Eigen)

if(urdfdom_LIBRARIES)
  target_link_libraries(TetraPGA INTERFACE ${urdfdom_LIBRARIES})
endif()

function(tetrapga_add_test target_name source_file)
  add_executable(${target_name} ${source_file})
  target_link_libraries(${target_name} PRIVATE TetraPGA::TetraPGA)
  target_compile_definitions(${target_name}
    PRIVATE TETRAPGA_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  add_test(NAME ${target_name} COMMAND ${target_name})
endfunction()

function(tetrapga_add_benchmark target_name source_file)
  add_executable(${target_name} ${source_file})
  target_link_libraries(${target_name}
    PRIVATE
      TetraPGA::TetraPGA
      benchmark::benchmark
      ${CASADI_LIBRARIES}
  )
  target_compile_definitions(${target_name}
    PRIVATE
      TETRAPGA_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
      GA4RO_HAS_CASADI_BENCH=1
  )
  target_include_directories(${target_name} PRIVATE ${CASADI_INCLUDE_DIRS})
  target_link_directories(${target_name} PRIVATE ${CASADI_LIBRARY_DIRS})
endfunction()

if(TETRAPGA_BUILD_TESTS)
  enable_testing()
  find_package(pinocchio REQUIRED)

  tetrapga_add_test(TetraPGA_motor_test test/TetraPGA_motor_test.cpp)
  tetrapga_add_test(TetraPGA_kinematics_test test/TetraPGA_kinematics_test.cpp)
  tetrapga_add_test(TetraPGA_dynamics_test test/TetraPGA_dynamics_test.cpp)

  tetrapga_add_test(TetraPGA_urdf_test test/TetraPGA_urdf_test.cpp)
  target_link_libraries(TetraPGA_urdf_test PRIVATE pinocchio::pinocchio)

  find_package(PkgConfig REQUIRED)
  pkg_check_modules(COAL REQUIRED IMPORTED_TARGET coal)

  tetrapga_add_test(TetraPGA_collision_test test/TetraPGA_collision_test.cpp)
  target_link_libraries(TetraPGA_collision_test
    PRIVATE
      pinocchio::pinocchio
      PkgConfig::COAL
  )
endif()

if(TETRAPGA_BUILD_BENCHMARKS)
  find_package(benchmark REQUIRED)
  find_package(pinocchio REQUIRED)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(CASADI REQUIRED casadi)

  tetrapga_add_benchmark(TetraPGA_dyn_bench benchmark/TetraPGA_dyn_bench.cpp)
  target_link_libraries(TetraPGA_dyn_bench PRIVATE pinocchio::pinocchio)

  tetrapga_add_benchmark(TetraPGA_dyn_fo_bench benchmark/TetraPGA_dyn_fo_bench.cpp)
  target_link_libraries(TetraPGA_dyn_fo_bench PRIVATE pinocchio::pinocchio)

  tetrapga_add_benchmark(TetraPGA_dyn_so_bench benchmark/TetraPGA_dyn_so_bench.cpp)
  target_link_libraries(TetraPGA_dyn_so_bench PRIVATE pinocchio::pinocchio)

  pkg_check_modules(COAL REQUIRED IMPORTED_TARGET coal)

  tetrapga_add_benchmark(TetraPGA_collision_bench benchmark/TetraPGA_collision_bench.cpp)
  target_link_libraries(TetraPGA_collision_bench
    PRIVATE
      pinocchio::pinocchio
      PkgConfig::COAL
  )
endif()

install(TARGETS TetraPGA EXPORT TetraPGATargets)
install(DIRECTORY include/TetraPGA DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY description DESTINATION ${CMAKE_INSTALL_DATADIR}/TetraPGA)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/TetraPGAConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/TetraPGAConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/TetraPGAConfig.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/TetraPGA"
)

install(EXPORT TetraPGATargets
  FILE TetraPGATargets.cmake
  NAMESPACE TetraPGA::
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/TetraPGA"
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/TetraPGAConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/TetraPGAConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/TetraPGA"
)
