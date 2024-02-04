cmake_minimum_required(VERSION 3.8)
project(math_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(customs REQUIRED)

add_executable(math_service src/math_service.cpp)
ament_target_dependencies(math_service rclcpp customs)

install(TARGETS
  math_service
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
