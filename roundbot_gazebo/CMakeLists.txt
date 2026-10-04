cmake_minimum_required(VERSION 3.20)
project(roundbot_gazebo)
set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

include_directories(include)

add_library(RoundbotInterfacePlugin SHARED
  src/RoundbotInterfacePlugin.cpp
)
target_link_libraries(RoundbotInterfacePlugin
  gz-sim::gz-sim
  gz-plugin::register
)

install(
  DIRECTORY
    config
    launch
    rviz
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(RoundbotInterfacePlugin)
install(
  TARGETS RoundbotInterfacePlugin
  DESTINATION lib
)

ament_package()