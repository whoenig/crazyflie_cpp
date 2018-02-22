cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_cpp)

find_library(USB_LIB usb-1.0)

# Enable C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
  include/crazyflie_cpp
)

add_library(crazyflie_cpp
  src/Crazyflie.cpp
  src/CrazyflieUSB.cpp
  src/Crazyradio.cpp
  src/crtp.cpp
  src/USBDevice.cpp
)

target_link_libraries(crazyflie_cpp
  ${USB_LIB}
)
