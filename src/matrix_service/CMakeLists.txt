cmake_minimum_required(VERSION 3.8)
project(matrix_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(customs REQUIRED)

add_executable(matrix_service src/matrix_service.cpp)
ament_target_dependencies(matrix_service rclcpp customs)

add_executable(matrix_service_test src/matrix_service_test.cpp)
ament_target_dependencies(matrix_service_test rclcpp customs)

install(TARGETS
  matrix_service
  matrix_service_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
