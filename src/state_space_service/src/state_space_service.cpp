#include <climits>

#include "rclcpp/rclcpp.hpp"

#include "customs/srv/matrix_exponential_discretization_by_step.hpp"
#include "customs/srv/matrix_exponential_discretization_by_accuracy.hpp"
#include "customs/srv/system_matrix_exponential_discretization_by_step.hpp"
#include "customs/srv/system_matrix_exponential_discretization_by_accuracy.hpp"
#include "customs/srv/input_matrix_exponential_discretization_from_system_by_step.hpp"
#include "customs/srv/input_matrix_exponential_discretization_from_system_by_accuracy.hpp"

class StateSpaceServiceNode : public rclcpp::Node
{
public:
    StateSpaceServiceNode() : Node("state_space_service")
    {
        this->declare_parameter("name", "StateSpaceService");
        _name = this->get_parameter("name").as_string();

        _matrix_exponential_discretization_by_step_service = this->create_service<customs::srv::MatrixExponentialDiscretizationByStep>(
            "matrix_exponential_discretization_by_step", std::bind(&StateSpaceServiceNode::callback_MatrixExponentialDiscretizationByStep, this, std::placeholders::_1, std::placeholders::_2));

        _matrix_exponential_discretization_by_accuracy_service = this->create_service<customs::srv::MatrixExponentialDiscretizationByAccuracy>(
            "matrix_exponential_discretization_by_accuracy", std::bind(&StateSpaceServiceNode::callback_MatrixExponentialDiscretizationByAccuracy, this, std::placeholders::_1, std::placeholders::_2));

        _system_matrix_exponential_discretization_by_step_service = this->create_service<customs::srv::SystemMatrixExponentialDiscretizationByStep>(
            "system_matrix_exponential_discretization_by_step", std::bind(&StateSpaceServiceNode::callback_SystemMatrixExponentialDiscretizationByStep, this, std::placeholders::_1, std::placeholders::_2));

        _system_matrix_exponential_discretization_by_accuracy_service = this->create_service<customs::srv::SystemMatrixExponentialDiscretizationByAccuracy>(
            "system_matrix_exponential_discretization_by_accuracy", std::bind(&StateSpaceServiceNode::callback_SystemMatrixExponentialDiscretizationByAccuracy, this, std::placeholders::_1, std::placeholders::_2));

        _input_matrix_exponential_discretization_from_system_by_step_service = this->create_service<customs::srv::InputMatrixExponentialDiscretizationFromSystemByStep>(
            "_input_matrix_exponential_discretization_from_system_by_step", std::bind(&StateSpaceServiceNode::callback_InputMatrixExponentialDiscretizationFromSystemByStep, this, std::placeholders::_1, std::placeholders::_2));

        _input_matrix_exponential_discretization_from_system_by_accuracy_service = this->create_service<customs::srv::InputMatrixExponentialDiscretizationFromSystemByAccuracy>(
            "_input_matrix_exponential_discretization_from_system_by_accuracy", std::bind(&StateSpaceServiceNode::callback_InputMatrixExponentialDiscretizationFromSystemByAccuracy, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "%s has been started.", _name.c_str());
    }

private:
    std::string _name = "";

    rclcpp::Service<customs::srv::MatrixExponentialDiscretizationByStep>::SharedPtr _matrix_exponential_discretization_by_step_service;
    rclcpp::Service<customs::srv::MatrixExponentialDiscretizationByAccuracy>::SharedPtr _matrix_exponential_discretization_by_accuracy_service;
    rclcpp::Service<customs::srv::SystemMatrixExponentialDiscretizationByStep>::SharedPtr _system_matrix_exponential_discretization_by_step_service;
    rclcpp::Service<customs::srv::SystemMatrixExponentialDiscretizationByAccuracy>::SharedPtr _system_matrix_exponential_discretization_by_accuracy_service;
    rclcpp::Service<customs::srv::InputMatrixExponentialDiscretizationFromSystemByStep>::SharedPtr _input_matrix_exponential_discretization_from_system_by_step_service;
    rclcpp::Service<customs::srv::InputMatrixExponentialDiscretizationFromSystemByAccuracy>::SharedPtr _input_matrix_exponential_discretization_from_system_by_accuracy_service;

    void callback_MatrixExponentialDiscretizationByStep(const customs::srv::MatrixExponentialDiscretizationByStep::Request::SharedPtr request,
                                                        const customs::srv::MatrixExponentialDiscretizationByStep::Response::SharedPtr response)
    {
        std::vector<std::vector<double>> A = _construct_matrix(request->system_matrix,
                                                               request->n1,
                                                               request->m1);

        std::vector<std::vector<double>> F =
            _system_matrix_exponential_discretization_by_step(
                A,
                request->dt,
                request->step);

        std::vector<std::vector<double>> B = _construct_matrix(request->input_matrix,
                                                               request->n2,
                                                               request->m2);

        std::vector<std::vector<double>> G =
            _input_matrix_exponential_discretization_from_system_by_step(
                A,
                B,
                request->dt,
                request->step);

        response->result_system = _deconstruct_matrix(F);
        response->result_input = _deconstruct_matrix(G);
    }

    void callback_MatrixExponentialDiscretizationByAccuracy(const customs::srv::MatrixExponentialDiscretizationByAccuracy::Request::SharedPtr request,
                                                            const customs::srv::MatrixExponentialDiscretizationByAccuracy::Response::SharedPtr response)
    {
        std::vector<std::vector<double>> A = _construct_matrix(request->system_matrix,
                                                               request->n1,
                                                               request->m1);

        std::vector<std::vector<double>> F =
            _system_matrix_exponential_discretization_by_accuracy(
                A,
                request->dt,
                request->accuracy);

        std::vector<std::vector<double>> B = _construct_matrix(request->input_matrix,
                                                               request->n2,
                                                               request->m2);

        std::vector<std::vector<double>> G =
            _input_matrix_exponential_discretization_from_system_by_accuracy(
                A,
                B,
                request->dt,
                request->accuracy);

        response->result_system = _deconstruct_matrix(F);
        response->result_input = _deconstruct_matrix(G);
    }

    void callback_SystemMatrixExponentialDiscretizationByStep(const customs::srv::SystemMatrixExponentialDiscretizationByStep::Request::SharedPtr request,
                                                              const customs::srv::SystemMatrixExponentialDiscretizationByStep::Response::SharedPtr response)
    {
        std::vector<std::vector<double>> A = _construct_matrix(request->system_matrix,
                                                               request->n,
                                                               request->m);

        std::vector<std::vector<double>> F =
            _system_matrix_exponential_discretization_by_step(
                A,
                request->dt,
                request->step);

        response->result = _deconstruct_matrix(F);
    }

    void callback_SystemMatrixExponentialDiscretizationByAccuracy(const customs::srv::SystemMatrixExponentialDiscretizationByAccuracy::Request::SharedPtr request,
                                                                  const customs::srv::SystemMatrixExponentialDiscretizationByAccuracy::Response::SharedPtr response)
    {
        std::vector<std::vector<double>> A = _construct_matrix(request->system_matrix,
                                                               request->n,
                                                               request->m);

        std::vector<std::vector<double>> F =
            _system_matrix_exponential_discretization_by_accuracy(
                A,
                request->dt,
                request->accuracy);

        response->result = _deconstruct_matrix(F);
    }

    void callback_InputMatrixExponentialDiscretizationFromSystemByStep(const customs::srv::InputMatrixExponentialDiscretizationFromSystemByStep::Request::SharedPtr request,
                                                                       const customs::srv::InputMatrixExponentialDiscretizationFromSystemByStep::Response::SharedPtr response)
    {
        std::vector<std::vector<double>> A = _construct_matrix(request->system_matrix,
                                                               request->n1,
                                                               request->m1);

        std::vector<std::vector<double>> B = _construct_matrix(request->input_matrix,
                                                               request->n2,
                                                               request->m2);

        std::vector<std::vector<double>> G =
            _input_matrix_exponential_discretization_from_system_by_step(
                A,
                B,
                request->dt,
                request->step);

        response->result = _deconstruct_matrix(G);
    }

    void callback_InputMatrixExponentialDiscretizationFromSystemByAccuracy(const customs::srv::InputMatrixExponentialDiscretizationFromSystemByAccuracy::Request::SharedPtr request,
                                                                           const customs::srv::InputMatrixExponentialDiscretizationFromSystemByAccuracy::Response::SharedPtr response)
    {
        std::vector<std::vector<double>> A = _construct_matrix(request->system_matrix,
                                                               request->n1,
                                                               request->m1);

        std::vector<std::vector<double>> B = _construct_matrix(request->input_matrix,
                                                               request->n2,
                                                               request->m2);

        std::vector<std::vector<double>> G =
            _input_matrix_exponential_discretization_from_system_by_accuracy(
                A,
                B,
                request->dt,
                request->accuracy);

        response->result = _deconstruct_matrix(G);
    }

    std::vector<std::vector<double>> _construct_matrix(const std::vector<double> a, const size_t n, const size_t m)
    {
        if (a.size() != (n * m))
        {
            return {{0.0}};
        }

        size_t index = 0;
        std::vector<std::vector<double>> A;

        for (size_t i = 1; i <= n; i++)
        {
            std::vector<double> row;

            for (size_t j = 1; j <= m; j++)
            {
                row.push_back(a[index++]);
            }

            A.push_back(row);
        }

        return A;
    }

    std::vector<double> _deconstruct_matrix(const std::vector<std::vector<double>> A)
    {
        int n = A.size();
        int m = A[0].size();

        std::vector<double> a;

        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < m; j++)
            {
                a.push_back(A[i][j]);
            }
        }

        return a;
    }

    int _factorial(const int number)
    {
        int factorial = 1;

        for (int i = 2; i <= number; i++)
        {
            factorial *= i;
        }

        return factorial;
    }

    double _matrix_max_element(const std::vector<std::vector<double>> A)
    {
        size_t n = A.size();
        size_t m = A[0].size();

        double max = INT_MIN;

        for (size_t i = 0; i < n; i++)
        {
            for (size_t j = 0; j < m; j++)
            {
                if (A[i][j] > max)
                {
                    max = A[i][j];
                }
            }
        }

        return max;
    }

    std::vector<std::vector<double>> _matrix_identity(const size_t n)
    {
        std::vector<std::vector<double>> I;

        for (size_t i = 0; i < n; i++)
        {
            std::vector<double> row;

            for (size_t j = 0; j < n; j++)
            {
                if (i == j)
                {
                    row.push_back(1.0);
                    continue;
                }

                row.push_back(0.0);
            }

            I.push_back(row);
        }

        return I;
    }

    std::vector<std::vector<double>> _matrix_multiplication(const std::vector<std::vector<double>> A, const std::vector<std::vector<double>> B)
    {
        size_t n = A.size();
        size_t m_1 = A[0].size();

        size_t m_2 = B.size();
        size_t p = B[0].size();

        if (m_1 != m_2)
        {
            return {{0.0}};
        }

        std::vector<std::vector<double>> AB;

        for (size_t i = 0; i < n; i++)
        {
            std::vector<double> row;

            for (size_t j = 0; j < p; j++)
            {
                row.push_back(0.0);

                for (size_t k = 0; k < m_1; k++)
                {
                    row[j] += A[i][k] * B[k][j];
                }
            }

            AB.push_back(row);
        }

        return AB;
    }

    std::vector<std::vector<double>> _matrix_inversion(const std::vector<std::vector<double>> A)
    {
        size_t n = A.size();
        size_t m = A[0].size();

        if (n != m)
        {
            return {{0.0}};
        }

        std::vector<std::vector<double>> identity = _matrix_identity(n);

        std::vector<std::vector<double>> augmented_matrix = A;

        // Augment the matrix with an identity matrix
        for (size_t i = 0; i < n; i++)
        {
            for (size_t j = 0; j < n; j++)
            {
                augmented_matrix[i].push_back(identity[i][j]);
            }
        }

        // Perform Gaussian elimination to obtain the inverse
        for (size_t i = 0; i < n; i++)
        {
            // Find pivot row
            size_t pivot_row = i;
            for (size_t j = i + 1; j < n; j++)
            {
                if (std::abs(augmented_matrix[j][i]) > std::abs(augmented_matrix[pivot_row][i]))
                {
                    pivot_row = j;
                }
            }

            // Swap current row with pivot row
            std::swap(augmented_matrix[i], augmented_matrix[pivot_row]);

            // Make the diagonal elements 1
            double pivot = augmented_matrix[i][i];
            for (size_t j = 0; j < 2 * n; j++)
            {
                augmented_matrix[i][j] /= pivot;
            }

            // Make the other rows 0
            for (size_t j = 0; j < n; j++)
            {
                if (i != j)
                {
                    double factor = augmented_matrix[j][i];
                    for (size_t k = 0; k < 2 * n; k++)
                    {
                        augmented_matrix[j][k] -= factor * augmented_matrix[i][k];
                    }
                }
            }
        }

        // Extract the inverse matrix
        std::vector<std::vector<double>> inverse;
        for (size_t i = 0; i < n; i++)
        {
            std::vector<double> row;
            for (size_t j = n; j < 2 * n; j++)
            {
                row.push_back(augmented_matrix[i][j]);
            }
            inverse.push_back(row);
        }

        return inverse;
    }

    std::vector<std::vector<double>> _matrix_addition(const std::vector<std::vector<double>> A, const std::vector<std::vector<double>> B)
    {
        size_t n_1 = A.size();
        size_t m_1 = A[0].size();

        size_t n_2 = B.size();
        size_t m_2 = B[0].size();

        if (n_1 != n_2 || m_1 != m_2)
        {
            return {{0.0}};
        }

        std::vector<std::vector<double>> AB;

        for (size_t i = 0; i < n_1; i++)
        {
            std::vector<double> row;

            for (size_t j = 0; j < m_1; j++)
            {
                row.push_back(A[i][j] + B[i][j]);
            }

            AB.push_back(row);
        }

        return AB;
    }

    std::vector<std::vector<double>> _matrix_subtraction(const std::vector<std::vector<double>> A, const std::vector<std::vector<double>> B)
    {
        size_t n_1 = A.size();
        size_t m_1 = A[0].size();

        size_t n_2 = B.size();
        size_t m_2 = B[0].size();

        if (n_1 != n_2 || m_1 != m_2)
        {
            return {{0.0}};
        }

        std::vector<std::vector<double>> AB;

        for (size_t i = 0; i < n_1; i++)
        {
            std::vector<double> row;

            for (size_t j = 0; j < m_1; j++)
            {
                row.push_back(A[i][j] - B[i][j]);
            }

            AB.push_back(row);
        }

        return AB;
    }

    std::vector<std::vector<double>> _matrix_multiplication_by_number(const std::vector<std::vector<double>> A, const double number)
    {
        size_t n = A.size();
        size_t m = A[0].size();

        std::vector<std::vector<double>> AB;

        for (size_t i = 0; i < n; i++)
        {
            std::vector<double> row;

            for (size_t j = 0; j < m; j++)
            {
                row.push_back(A[i][j] * number);
            }

            AB.push_back(row);
        }

        return AB;
    }

    std::vector<std::vector<double>> _matrix_division_by_number(const std::vector<std::vector<double>> A, const double number)
    {
        size_t n = A.size();
        size_t m = A[0].size();

        std::vector<std::vector<double>> AB;

        for (size_t i = 0; i < n; i++)
        {
            std::vector<double> row;

            for (size_t j = 0; j < m; j++)
            {
                row.push_back(A[i][j] / number);
            }

            AB.push_back(row);
        }

        return AB;
    }

    std::vector<std::vector<double>> _system_matrix_exponential_discretization_by_step(const std::vector<std::vector<double>> A, const double dt, const int step)
    {
        int n = A.size();
        int m = A[0].size();

        if (n != m)
        {
            return {{0.0}};
        }

        std::vector<std::vector<double>> F = _matrix_identity(n);
        std::vector<std::vector<double>> Adt_pow = _matrix_identity(n);
        std::vector<std::vector<double>> Adt = _matrix_multiplication_by_number(A, dt);

        for (int i = 1; i <= step; i++)
        {
            int denominator = _factorial(i);

            Adt_pow = _matrix_multiplication(Adt_pow, Adt);

            std::vector<std::vector<double>> Adt_pow_denom = _matrix_division_by_number(Adt_pow, denominator);

            F = _matrix_addition(F, Adt_pow_denom);
        }

        return F;
    }

    std::vector<std::vector<double>> _system_matrix_exponential_discretization_by_accuracy(const std::vector<std::vector<double>> A, const double dt, const double accuracy)
    {
        int n = A.size();
        int m = A[0].size();

        if (n != m)
        {
            return {{0.0}};
        }

        std::vector<std::vector<double>> F = _matrix_identity(n);
        std::vector<std::vector<double>> Adt_pow = _matrix_identity(n);
        std::vector<std::vector<double>> Adt = _matrix_multiplication_by_number(A, dt);

        for (int i = 1;; i++)
        {
            int denominator = _factorial(i);

            Adt_pow = _matrix_multiplication(Adt_pow, Adt);

            std::vector<std::vector<double>> Adt_pow_denom = _matrix_division_by_number(Adt_pow, denominator);
            F = _matrix_addition(F, Adt_pow_denom);

            double significant = _matrix_max_element(Adt_pow_denom);

            if (significant <= accuracy)
            {
                break;
            }
        }

        return F;
    }

    std::vector<std::vector<double>> _input_matrix_exponential_discretization_from_system_by_step(const std::vector<std::vector<double>> A, const std::vector<std::vector<double>> B, const double dt, const int step)
    {
        int n_1 = A.size();
        int m_1 = A[0].size();

        int n_2 = B.size();

        if (m_1 != n_2)
        {
            return {{0.0}};
        }

        std::vector<std::vector<double>> I = _matrix_identity(n_1);
        std::vector<std::vector<double>> F = _system_matrix_exponential_discretization_by_step(A, dt, step);
        std::vector<std::vector<double>> _F = _system_matrix_exponential_discretization_by_step(A, -dt, step);
        std::vector<std::vector<double>> I__F = _matrix_subtraction(I, _F);

        std::vector<std::vector<double>> inv_A = _matrix_inversion(A);
        std::vector<std::vector<double>> inv_AB = _matrix_multiplication(inv_A, B);

        std::vector<std::vector<double>> I__F_inv_AB = _matrix_multiplication(I__F, inv_AB);

        std::vector<std::vector<double>> G = _matrix_multiplication(F, I__F_inv_AB);

        return G;
    }

    std::vector<std::vector<double>> _input_matrix_exponential_discretization_from_system_by_accuracy(const std::vector<std::vector<double>> A, const std::vector<std::vector<double>> B, const double dt, const double accuracy)
    {
        int n_1 = A.size();
        int m_1 = A[0].size();

        int n_2 = B.size();

        if (m_1 != n_2)
        {
            return {{0.0}};
        }

        std::vector<std::vector<double>> I = _matrix_identity(n_1);
        std::vector<std::vector<double>> F = _system_matrix_exponential_discretization_by_accuracy(A, dt, accuracy);
        std::vector<std::vector<double>> _F = _system_matrix_exponential_discretization_by_accuracy(A, -dt, accuracy);
        std::vector<std::vector<double>> I__F = _matrix_subtraction(I, _F);

        std::vector<std::vector<double>> inv_A = _matrix_inversion(A);
        std::vector<std::vector<double>> inv_AB = _matrix_multiplication(inv_A, B);

        std::vector<std::vector<double>> I__F_inv_AB = _matrix_multiplication(I__F, inv_AB);

        std::vector<std::vector<double>> G = _matrix_multiplication(F, I__F_inv_AB);

        return G;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StateSpaceServiceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
