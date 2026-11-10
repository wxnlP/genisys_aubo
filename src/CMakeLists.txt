# Set namespace
set_root_namespace("genisys_aubo")

add_subdirectory(protocols)

# module
add_subdirectory(module/control_module)
add_subdirectory(module/ros2_proxy_module)

# pkg
add_subdirectory(pkg/arm_control_pkg)
add_subdirectory(pkg/ros2_proxy_pkg)

# install
if(CMAKE_SYSTEM_NAME MATCHES "Linux")
  set(CUR_INSTALL_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/install/linux)
elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
  set(CUR_INSTALL_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/install/win)
else()
  message(FATAL_ERROR "Unsupport os")
endif()

install(
  DIRECTORY ${CUR_INSTALL_SOURCE_DIR}/bin
  DESTINATION ./
  USE_SOURCE_PERMISSIONS)

# build all
add_custom_target(
  genisys_aubo_build_all ALL
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CUR_INSTALL_SOURCE_DIR}/bin ${CMAKE_BINARY_DIR}
  DEPENDS aimrt::runtime::main
          genisys_aubo::arm_control_pkg
          genisys_aubo::ros2_proxy_pkg)


