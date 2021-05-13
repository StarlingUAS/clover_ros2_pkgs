cmake_minimum_required(VERSION 3.5)
project(ws281x)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(ROS_WS2811_MOCK_LIB "Use mock ws281x library instead of the real one" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(led_msgs REQUIRED)

# Build Messages and Services
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetGamma.srv"
)
ament_export_dependencies(rosidl_default_runtime)

# Build rpi_ws281x
add_subdirectory(vendor)

# Build ws281x ros node
add_executable(${PROJECT_NAME}_node src/ws281x_node.cpp)
## Link rpi library
target_link_libraries(${PROJECT_NAME}_node rpi_ws281x)

## Specify libraries to link a library or executable target against
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp led_msgs
)

# Since we're using SUID bits, we need to leave rpath in for stripped binaries
set_target_properties(${PROJECT_NAME}_node PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

# Ensure cmake can find the msg/srv built as part of this project
rosidl_target_interfaces(${PROJECT_NAME}_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

#############
## Install ##
#############

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()