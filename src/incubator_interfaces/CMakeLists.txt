cmake_minimum_required(VERSION 3.8)
project(incubator_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# Also add packages that your interfaces depend 

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DeviceSensorState.msg"
  "msg/DeviceCommand.msg"
  # Add packages that above messages depend on
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_package()
