# TetraPGA-preview

A header-only C++ library for PGA-based robot kinematics, dynamics, and collision-aware computations. [Preview Version]

## Contents

- `include/TetraPGA`: Public headers
- Robot descriptions: external assets from [Chen-WH/robot-assets](https://github.com/Chen-WH/robot-assets)
- `test`: Regression-style executable tests
- `benchmark`: Performance benchmarks against reference implementations

## Build

Robot descriptions come from [Chen-WH/robot-assets](https://github.com/Chen-WH/robot-assets) and are expected at `../robot-assets` by default. Set `TETRAPGA_ROBOT_ASSETS_DIR` for another checkout.

```bash
cmake -S . -B build -DTETRAPGA_BUILD_TESTS=ON
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

To build benchmarks:

```bash
cmake -S . -B build-bench -DTETRAPGA_BUILD_TESTS=OFF -DTETRAPGA_BUILD_BENCHMARKS=ON
cmake --build build-bench -j4
taskset -c 2 ./build-bench/TetraPGA_*_bench # 单线程 benchmark
```

## Dependency Modes

`TetraPGA` can be consumed in two ways:

- as an installed package via `find_package(TetraPGA CONFIG REQUIRED)`
- as source via `add_subdirectory(...)`

The exported CMake package configuration is generated from `cmake/TetraPGAConfig.cmake.in`.
