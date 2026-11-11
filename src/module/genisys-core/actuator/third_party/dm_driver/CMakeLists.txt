cmake_minimum_required(VERSION 3.24)
project(dm_driver LANGUAGES C CXX)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(dm_driver STATIC)

target_include_directories(
  dm_driver
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/core ${CMAKE_CURRENT_SOURCE_DIR}/port/linux)

target_sources(
  dm_driver
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/core/dm_motor_driver.cc
          ${CMAKE_CURRENT_SOURCE_DIR}/port/linux/socket_can.c
          ${CMAKE_CURRENT_SOURCE_DIR}/port/linux/canfd_bus.cc)
