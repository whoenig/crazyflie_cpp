[![CMake](https://github.com/whoenig/crazyflie_cpp/actions/workflows/cmake.yml/badge.svg?branch=dev-crazyflie-link-cpp)](https://github.com/whoenig/crazyflie_cpp/actions/workflows/cmake.yml) [![CMake (Windows)](https://github.com/whoenig/crazyflie_cpp/actions/workflows/cmake_win.yml/badge.svg)](https://github.com/whoenig/crazyflie_cpp/actions/workflows/cmake_win.yml) [![CMake (Mac)](https://github.com/whoenig/crazyflie_cpp/actions/workflows/cmake_mac.yml/badge.svg)](https://github.com/whoenig/crazyflie_cpp/actions/workflows/cmake_mac.yml)

# Crazyflie_cpp

Standalone C++ library to use the Crazyflie quadrotor.
This is used in crazyflie_ros, and crazyflie_tools, but can also be used for other custom applications.

This repository relies on [crazyflie-link-cpp](https://github.com/bitcraze/crazyflie-link-cpp) for the low level communication link.

## Build

Make sure to clone recursively with submodules.

```
mkdir build
cd build
cmake ..
make
```
