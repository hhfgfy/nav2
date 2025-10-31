cmake_minimum_required(VERSION 3.8)
project(nav2_mpc_controller)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()

set(library_name nav2_mpc_controller)

add_library(${library_name} SHARED
  src/controller.cpp
)

ament_target_dependencies(${library_name}
  PUBLIC
    angles
    geometry_msgs
    nav2_core
    nav2_costmap_2d
    nav2_ros_common
    nav2_util
    nav_msgs
    pluginlib
    rclcpp
    rclcpp_lifecycle
    tf2
    tf2_ros
)

target_include_directories(${library_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${library_name}
  PUBLIC
    nav2_core::nav2_core
    nav2_costmap_2d::nav2_costmap_2d_core
    nav2_ros_common::nav2_ros_common
    nav2_util::nav2_util_core
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    tf2::tf2
    tf2_ros::tf2_ros
)

install(TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(
  angles
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_ros_common
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
)
ament_export_targets(${library_name})

pluginlib_export_plugin_description_file(nav2_core mpc_controller.xml)

ament_package()
