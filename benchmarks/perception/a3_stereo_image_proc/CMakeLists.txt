cmake_minimum_required(VERSION 3.5)
project(a3_stereo_image_proc)

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

find_package(stereo_image_proc REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# LTTng flame chart and graphs
# see https://archive.eclipse.org/tracecompass/doc/stable/org.eclipse.tracecompass.doc.user/LTTng-UST-Analyses.html#Flame_Chart_View
set(CMAKE_CXX_FLAGS "-g -O2 -finstrument-functions")

# Nodes built as "components"
ament_auto_add_library(images_input_component SHARED src/images_input_component.cpp)
ament_target_dependencies(images_input_component tracetools_benchmark image_transport stereo_image_proc)
target_compile_definitions(images_input_component PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(images_input_component "robotperf::perception::ImagesInputComponent")
set(node_plugins "${node_plugins}robotperf::perception::ImagesInputComponent;$<TARGET_FILE:images_input_component>\n")

ament_auto_add_library(disparity_output_component SHARED src/disparity_output_component.cpp)
ament_target_dependencies(disparity_output_component tracetools_benchmark image_transport)
target_compile_definitions(disparity_output_component PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(disparity_output_component "robotperf::perception::DisparityOutputComponent")
set(node_plugins "${node_plugins}robotperf::perception::DisparityOutputComponent;$<TARGET_FILE:disparity_output_component>\n")

# install(
#   TARGETS ${install_targets}
#   DESTINATION lib/${PROJECT_NAME}
# )

# # Install launch files.
# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_auto_package(INSTALL_TO_SHARE launch)



# cmake_minimum_required(VERSION 3.8)
# project(a3_stereo_image_proc)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)
# find_package(stereo_image_proc REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()


# install(
#   DIRECTORY config description launch worlds
#   DESTINATION share/${PROJECT_NAME}

# )


# ament_package()
