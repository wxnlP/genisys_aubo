add_subdirectory(third_party/dm_driver)

add_library(genisys_aubo_actuator STATIC)
add_library(genisys_aubo::actuator ALIAS genisys_aubo_actuator)

target_sources(
  genisys_aubo_actuator
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/geniarm_gripper.cc
          ${CMAKE_CURRENT_SOURCE_DIR}/src/gripper_manager.cc)

target_include_directories(
  genisys_aubo_actuator
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(genisys_aubo_actuator PUBLIC dm_driver)
target_link_libraries(genisys_aubo_actuator PRIVATE yaml-cpp::yaml-cpp)
