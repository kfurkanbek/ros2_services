cmake_minimum_required(VERSION 3.8)
project(signal_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(customs REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(signal_service src/signal_service.cpp)
ament_target_dependencies(signal_service rclcpp customs example_interfaces)

add_executable(signal_service_test src/signal_service_test.cpp)
ament_target_dependencies(signal_service_test rclcpp customs example_interfaces)

install(TARGETS
  signal_service
  signal_service_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
