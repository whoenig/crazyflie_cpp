cmake_minimum_required(VERSION 3.5)
project(crazyflie_cpp VERSION 0.0.1)

find_library(USB_LIB usb-1.0)

# Enable C++11 and warnings
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Werror)

add_library(crazyflie_cpp
  src/Crazyflie.cpp
  src/CrazyflieUSB.cpp
  src/Crazyradio.cpp
  src/crtp.cpp
  src/USBDevice.cpp
  src/ITransport.cpp
)

target_include_directories(crazyflie_cpp
  PUBLIC
    $<INSTALL_INTERFACE:include/crazyflie_cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/crazyflie_cpp>
)

target_link_libraries(crazyflie_cpp
  PRIVATE
    ${USB_LIB}
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  crazyflie_cpp-config-version.cmake
  VERSION ${CRAZYFLIE_CPP_VERSION}
  COMPATIBILITY AnyNewerVersion
)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/crazyflie_cpp
  DESTINATION include
)

install(
  TARGETS crazyflie_cpp
  EXPORT crazyflie_cpp-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(EXPORT crazyflie_cpp-targets
  FILE crazyflie_cpp-targets.cmake
  NAMESPACE crazyflie_cpp::
  DESTINATION lib/cmake/crazyflie_cpp
)

install(
  FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/crazyflie_cpp-config.cmake"
  DESTINATION
    lib/cmake/crazyflie_cpp
)

# Install ROS package information
install(
  FILES
    package.xml
  DESTINATION
    share/crazyflie_cpp
)
