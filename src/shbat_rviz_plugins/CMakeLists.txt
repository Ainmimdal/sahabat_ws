cmake_minimum_required(VERSION 3.8)
project(shbat_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sahabat_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(${PROJECT_NAME} SHARED
  include/shbat_rviz_plugins/localization_recovery_panel.hpp
  include/shbat_rviz_plugins/navigation_status_panel.hpp
  include/shbat_rviz_plugins/waypoint_editor_panel.hpp
  include/shbat_rviz_plugins/waypoint_editor_tool.hpp
  src/localization_recovery_panel.cpp
  src/navigation_status_panel.cpp
  src/waypoint_editor_panel.cpp
  src/waypoint_editor_tool.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  action_msgs
  rclcpp
  rviz_common
  rviz_default_plugins
  interactive_markers
  visualization_msgs
  geometry_msgs
  nav2_msgs
  nav_msgs
  rcl_interfaces
  sahabat_interfaces
  std_msgs
  std_srvs
)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets yaml-cpp)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rviz_common rviz_default_plugins interactive_markers visualization_msgs)
ament_package()
