cmake_minimum_required(VERSION 3.1)
project(crazyflie_cpp)

find_library(USB_LIB usb-1.0)

# Enable C++11 and warnings
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Werror)

include_directories(
  include/crazyflie_cpp
)

add_library(crazyflie_cpp
  src/Crazyflie.cpp
  src/CrazyflieUSB.cpp
  src/Crazyradio.cpp
  src/crtp.cpp
  src/USBDevice.cpp
  src/ITransport.cpp
)

target_link_libraries(crazyflie_cpp
  ${USB_LIB}
)
