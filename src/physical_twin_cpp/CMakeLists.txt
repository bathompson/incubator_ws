cmake_minimum_required(VERSION 3.8)
project(physical_twin_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(incubator_interfaces REQUIRED)

add_executable(sensor_driver_node src/Sensor_Driver_Node.cpp include/PhysicalTwin_cpp/Device_Layer.hpp)
ament_target_dependencies(sensor_driver_node rclcpp incubator_interfaces)

add_executable(controller_node src/Controller_Node.cpp include/PhysicalTwin_cpp/Controller_Model_SM.hpp)
ament_target_dependencies(controller_node rclcpp incubator_interfaces)

target_include_directories(sensor_driver_node PRIVATE include)
target_include_directories(controller_node PRIVATE include)

target_link_libraries(sensor_driver_node pigpio)

install(TARGETS
  sensor_driver_node
  controller_node
DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
