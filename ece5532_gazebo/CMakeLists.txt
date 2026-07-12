cmake_minimum_required(VERSION 3.5)
project(ece5532_gazebo)
set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

add_library(TrafficLightPlugin SHARED
  src/TrafficLightPlugin.cpp
)
target_link_libraries(TrafficLightPlugin
  gz-sim::gz-sim
  gz-plugin::register
)

install(
  DIRECTORY
    launch
    models
    worlds
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS TrafficLightPlugin
  DESTINATION lib
)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")
ament_export_libraries(TrafficLightPlugin)
ament_package()