# ros2_services
Open source services library for ROS2

### list of services
matrix service

state space service

math service

### matrix service
print_matrix

matrix_max_element

matrix_min_element

matrix_identity

matrix_square

matrix_multiplication

matrix_inversion

matrix_addition

matrix_subtraction

matrix_multiplication_by_number

matrix_division_by_number

matrix_addition_by_number

matrix_subtraction_by_number

### state space service
matrix_exponential_discretization_by_step

matrix_exponential_discretization_by_accuracy

system_matrix_exponential_discretization_by_step

system_matrix_exponential_discretization_by_accuracy

input_matrix_exponential_discretization_from_system_by_step

input_matrix_exponential_discretization_from_system_by_accuracy

### math service
factorial

## list of custom service interfaces
### matrix service interfaces
PrintMatrix.srv

MatrixMaxElement.srv

MatrixMinElement.srv

MatrixIdentity.srv

MatrixSquare.srv

MatrixMultiplication.srv

MatrixInversion.srv

MatrixAddition.srv

MatrixSubtraction.srv

MatrixMultiplicationByNumber.srv

MatrixDivisionByNumber.srv

MatrixAdditionByNumber.srv

MatrixSubtractionByNumber.srv

### state space service interfaces
MatrixExponentialDiscretizationByStep.srv

MatrixExponentialDiscretizationByAccuracy.srv

SystemMatrixExponentialDiscretizationByStep.srv

SystemMatrixExponentialDiscretizationByAccuracy.srv

InputMatrixExponentialDiscretizationFromSystemByStep.srv

InputMatrixExponentialDiscretizationFromSystemByAccuracy.srv

### math service interfaces
Factorial.srv

## example interface for a state space service: MatrixExponentialDiscretizationByAccuracy.srv
```
##### returns the matrix exponential discretization of the system and input matrices, deconstructed as one dimensional array
#
### input 1 : system matrix deconstructed as one dimensional array
#
# [[1, 2],
#  [3, 4],
#  [5, 6]]
#
# [1, 2, 3, 4, 5, 6]
#
### input 2 : height or row count of system matrix
#
# 1) [[*],
# 2)  [*],
# 3)  [*]]
#
# height = 3 rows
#
### input 3 : width or column count of system matrix
#
#   1) 2)
#  [*, *]
#
# width = 2 columns
#
### input 4 : input matrix deconstructed as one dimensional array
#
### input 5 : height or row count of input matrix
#
### input 6 : width or column count of input matrix
#
### input 7 : time step of the discretization
#
### input 8 : relative error limit in taylor series expansion
#
### output 1 : discretized system matrix deconstructed as one dimensional array
#
### output 2 : discretized input matrix deconstructed as one dimensional array
#
#####
float64[] system_matrix
int64 n1
int64 m1
float64[] input_matrix
int64 n2
int64 m2
float64 dt
float64 accuracy
---
float64[] result_system
float64[] result_input
```

## example interface for a matrix service: MatrixIdentity.srv
```
##### returns the inverse of a matrix, deconstructed as one dimensional array
#
### input 1 : matrix deconstructed as one dimensional array
#
# [[1, 2],
#  [3, 4],
#  [5, 6]]
#
# [1, 2, 3, 4, 5, 6]
#
### input 2 : height or row count
#
# 1) [[*],
# 2)  [*],
# 3)  [*]]
#
# height = 3 rows
#
### input 3 : width or column count
#
#   1) 2)
#  [*, *]
#
# width = 2 columns
#
### output 1 : matrix deconstructed as one dimensional array
#
#####
float64[] matrix
int64 n
int64 m
---
float64[] result
```
