cmake_minimum_required(VERSION 3.5)
project(customs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Factorial.srv"
  "srv/PrintMatrix.srv"
  "srv/MatrixMaxElement.srv"
  "srv/MatrixMinElement.srv"
  "srv/MatrixIdentity.srv"
  "srv/MatrixSquare.srv"
  "srv/MatrixMultiplication.srv"
  "srv/MatrixInversion.srv"
  "srv/MatrixAddition.srv"
  "srv/MatrixSubtraction.srv"
  "srv/MatrixMultiplicationByNumber.srv"
  "srv/MatrixDivisionByNumber.srv"
  "srv/MatrixAdditionByNumber.srv"
  "srv/MatrixSubtractionByNumber.srv"
  "srv/SystemMatrixExponentialDiscretizationByStep.srv"
  "srv/SystemMatrixExponentialDiscretizationByAccuracy.srv"
  "srv/InputMatrixExponentialDiscretizationFromSystemByStep.srv"
  "srv/InputMatrixExponentialDiscretizationFromSystemByAccuracy.srv"
  "srv/MatrixExponentialDiscretizationByStep.srv"
  "srv/MatrixExponentialDiscretizationByAccuracy.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
