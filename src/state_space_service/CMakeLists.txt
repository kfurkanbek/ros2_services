cmake_minimum_required(VERSION 3.8)
project(state_space_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(customs REQUIRED)

add_executable(state_space_service src/state_space_service.cpp)
ament_target_dependencies(state_space_service rclcpp customs)

install(TARGETS
  state_space_service
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
