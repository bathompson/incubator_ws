cmake_minimum_required(VERSION 3.8)
set(CMAKE_TOOLCHAIN_FILE "/home/thompson/vcpkg/scripts/buildsystems/vcpkg.cmake")
project(digital_twin_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(incubator_interfaces REQUIRED)
find_package(influxdb-cxx REQUIRED)

add_executable(influxdb-recorder-node src/influxdb_recorder_node.cpp)
ament_target_dependencies(influxdb-recorder-node rclcpp incubator_interfaces)
target_link_libraries(influxdb-recorder-node PRIVATE InfluxData::InfluxDB)
install(TARGETS 
  influxdb-recorder-node
  DESTINATION /ib/${PROJECT_NAME}
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
