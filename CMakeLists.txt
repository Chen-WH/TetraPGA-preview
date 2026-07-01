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

set(TETRAPGA_ROBOT_ASSETS_DIR_DEFAULT "${CMAKE_CURRENT_SOURCE_DIR}/../robot-assets")
get_filename_component(TETRAPGA_ROBOT_ASSETS_DIR_DEFAULT "${TETRAPGA_ROBOT_ASSETS_DIR_DEFAULT}" ABSOLUTE)
set(TETRAPGA_ROBOT_ASSETS_DIR "${TETRAPGA_ROBOT_ASSETS_DIR_DEFAULT}" CACHE PATH
  "Path to the robot-assets repository used by TetraPGA tests and downstream packages")

if(NOT EXISTS "${TETRAPGA_ROBOT_ASSETS_DIR}/ur10/urdf/ur10.urdf")
  message(FATAL_ERROR
    "TETRAPGA_ROBOT_ASSETS_DIR must point to the robot-assets repository. "
    "Expected to find ur10/urdf/ur10.urdf under '${TETRAPGA_ROBOT_ASSETS_DIR}'.")
endif()

set(TETRAPGA_MOBILE_MANIPULATOR_URDF
  "${TETRAPGA_ROBOT_ASSETS_DIR}/stanford_tidybot/urdf/tidybot_gen3_10dof.urdf")
if(TETRAPGA_BUILD_TESTS AND NOT EXISTS "${TETRAPGA_MOBILE_MANIPULATOR_URDF}")
  message(FATAL_ERROR
    "Expected the Stanford TidyBot mobile manipulator URDF at "
    "'${TETRAPGA_MOBILE_MANIPULATOR_URDF}'.")
endif()

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
    PRIVATE TETRAPGA_ROBOT_ASSETS_DIR="${TETRAPGA_ROBOT_ASSETS_DIR}")
  add_test(NAME ${target_name} COMMAND ${target_name})
endfunction()

function(tetrapga_add_benchmark target_name source_file)
  add_executable(${target_name} ${source_file})
  target_link_libraries(${target_name}
    PRIVATE
      TetraPGA::TetraPGA
      benchmark::benchmark
  )
  target_compile_definitions(${target_name}
    PRIVATE
      TETRAPGA_ROBOT_ASSETS_DIR="${TETRAPGA_ROBOT_ASSETS_DIR}"
  )
  if(CASADI_FOUND)
    target_link_libraries(${target_name} PRIVATE ${CASADI_LIBRARIES})
    target_compile_definitions(${target_name} PRIVATE GA4RO_HAS_CASADI_BENCH=1)
    target_include_directories(${target_name} PRIVATE ${CASADI_INCLUDE_DIRS})
    target_link_directories(${target_name} PRIVATE ${CASADI_LIBRARY_DIRS})
  endif()
endfunction()

if(TETRAPGA_BUILD_TESTS)
  enable_testing()
  find_package(pinocchio REQUIRED)

  tetrapga_add_test(TetraPGA_motor_test test/TetraPGA_motor_test.cpp)
  tetrapga_add_test(TetraPGA_kinematics_test test/TetraPGA_kinematics_test.cpp)
  tetrapga_add_test(TetraPGA_dynamics_test test/TetraPGA_dynamics_test.cpp)

  tetrapga_add_test(TetraPGA_models_test test/TetraPGA_models_test.cpp)
  target_link_libraries(TetraPGA_models_test PRIVATE pinocchio::pinocchio)
  add_test(NAME TetraPGA_mobile_manipulator_models_test
    COMMAND TetraPGA_models_test "${TETRAPGA_MOBILE_MANIPULATOR_URDF}")

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
  pkg_check_modules(CASADI QUIET casadi)

  tetrapga_add_benchmark(TetraPGA_dyn_bench benchmark/TetraPGA_dyn_bench.cpp)
  target_link_libraries(TetraPGA_dyn_bench PRIVATE pinocchio::pinocchio)

  tetrapga_add_benchmark(TetraPGA_dyn_fo_bench benchmark/TetraPGA_dyn_fo_bench.cpp)
  target_link_libraries(TetraPGA_dyn_fo_bench PRIVATE pinocchio::pinocchio)

  tetrapga_add_benchmark(TetraPGA_dyn_so_bench benchmark/TetraPGA_dyn_so_bench.cpp)
  target_link_libraries(TetraPGA_dyn_so_bench PRIVATE pinocchio::pinocchio)

  tetrapga_add_benchmark(TetraPGA_operator_bench benchmark/TetraPGA_operator_bench.cpp)
  target_link_libraries(TetraPGA_operator_bench PRIVATE pinocchio::pinocchio)

  pkg_check_modules(COAL QUIET IMPORTED_TARGET coal)
  if(COAL_FOUND)
    tetrapga_add_benchmark(TetraPGA_collision_bench benchmark/TetraPGA_collision_bench.cpp)
    target_link_libraries(TetraPGA_collision_bench
      PRIVATE
        pinocchio::pinocchio
        PkgConfig::COAL
    )
  endif()
endif()

install(TARGETS TetraPGA EXPORT TetraPGATargets)
install(DIRECTORY include/TetraPGA DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

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
