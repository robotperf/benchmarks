cmake_minimum_required(VERSION 3.5)
project(a4_depth_image_proc)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

install(
  TARGETS ${install_targets}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install description directory.
install(
  DIRECTORY description
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch)
# ament_package()

