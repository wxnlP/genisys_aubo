add_library(genisys_aubo_arm_control STATIC)
add_library(genisys_aubo::arm_control ALIAS genisys_aubo_arm_control)

target_sources(
  genisys_aubo_arm_control
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/gripper_controller.cc
          ${CMAKE_CURRENT_SOURCE_DIR}/src/lspb_time_scaler.cc
          ${CMAKE_CURRENT_SOURCE_DIR}/src/quintic_time_scaler.cc)

target_include_directories(
  genisys_aubo_arm_control
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(genisys_aubo_arm_control PUBLIC fmt::fmt)
