cmake_minimum_required(VERSION 3.5)
project(road_world_editor)
set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/RoadWorldEditor.cpp
  src/GazeboModel.cpp
  src/ModelPort.cpp
  include/road_world_editor/RoadWorldEditor.hpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  rviz_common::rviz_common
  pluginlib::pluginlib
  interactive_markers::interactive_markers
  visualization_msgs::visualization_msgs__rosidl_typesupport_cpp
  nlohmann_json::nlohmann_json
  ament_index_cpp::ament_index_cpp
  lanelet2_core::lanelet2_core
  lanelet2_io::lanelet2_io
  lanelet2_projection::lanelet2_projection
  Qt5::Widgets
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
    model_meshes
    rviz
  DESTINATION
    DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(RoadWorldEditor)
ament_export_dependencies(rclcpp rviz_common pluginlib interactive_markers visualization_msgs)

ament_package()
