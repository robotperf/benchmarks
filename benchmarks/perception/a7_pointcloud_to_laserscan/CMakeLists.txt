cmake_minimum_required(VERSION 3.5)
project(a7_pointcloud_to_laserscan)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


find_package(tracetools_benchmark REQUIRED)
find_package(depthimage_to_laserscan REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# LTTng flame chart and graphs
# see https://archive.eclipse.org/tracecompass/doc/stable/org.eclipse.tracecompass.doc.user/LTTng-UST-Analyses.html#Flame_Chart_View
set(CMAKE_CXX_FLAGS "-g -O2 -finstrument-functions")

# Add the new CPP file to be compiled
ament_auto_add_library(pointcloud_input_component SHARED src/pointcloud_input_component.cpp)
# Link the new component library with other dependencies
ament_target_dependencies(pointcloud_input_component tracetools_benchmark)
target_compile_definitions(pointcloud_input_component PRIVATE "COMPOSITION_BUILDING_DLL")
# Register the node as a component
rclcpp_components_register_nodes(pointcloud_input_component "robotperf::perception::PointCloudInputComponent")
# Add new node component to list of plugins
set(node_plugins "${node_plugins}robotperf::perception::PointCloudInputComponent;$<TARGET_FILE:pointcloud_input_component>\n")


install(
  TARGETS ${install_targets}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch)
