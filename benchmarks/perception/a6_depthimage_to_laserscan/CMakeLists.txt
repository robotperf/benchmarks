cmake_minimum_required(VERSION 3.5)
project(a6_depthimage_to_laserscan)

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
ament_auto_add_library(laserscan_output_component SHARED src/laserscan_output_component.cpp)
# Link the new component library with other dependencies
ament_target_dependencies(laserscan_output_component tracetools_benchmark depthimage_to_laserscan)
target_compile_definitions(laserscan_output_component PRIVATE "COMPOSITION_BUILDING_DLL")
# Register the node as a component
rclcpp_components_register_nodes(laserscan_output_component "robotperf::perception::LaserscanOutputComponent")
# Add new node component to list of plugins
set(node_plugins "${node_plugins}robotperf::perception::LaserscanOutputComponent;$<TARGET_FILE:laserscan_output_component>\n")


# Add the new CPP file (disparity_to_depthimage_component.cpp) to be compiled
ament_auto_add_library(disparity_to_depthimage_component SHARED src/disparity_to_depthimage_component.cpp)
# Link the new component library with other dependencies
ament_target_dependencies(disparity_to_depthimage_component tracetools_benchmark)
# Register the node as a component
rclcpp_components_register_nodes(disparity_to_depthimage_component "robotperf::perception::DisparityToDepthImageComponent")
# Add new node component to the list of plugins
set(node_plugins "${node_plugins}robotperf::perception::DisparityToDepthImageComponent;$<TARGET_FILE:disparity_to_depthimage_component>\n")


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
