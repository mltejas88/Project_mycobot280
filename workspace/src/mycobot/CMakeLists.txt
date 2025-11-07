cmake_minimum_required(VERSION 3.8)
project(mycobot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# If you add C++ nodes, enable these:
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)

# install package.xml so the package is discoverable
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Install urdf (including meshes), launch, config and worlds
install(
  DIRECTORY urdf launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

# ament package macro
ament_package()

