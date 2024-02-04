#include <vector>
#include <cmath>
#include <climits>

#include "rclcpp/rclcpp.hpp"

#include "customs/srv/print_matrix.hpp"
#include "customs/srv/matrix_max_element.hpp"
#include "customs/srv/matrix_min_element.hpp"
#include "customs/srv/matrix_identity.hpp"
#include "customs/srv/matrix_square.hpp"
#include "customs/srv/matrix_multiplication.hpp"
#include "customs/srv/matrix_inversion.hpp"
#include "customs/srv/matrix_addition.hpp"
#include "customs/srv/matrix_subtraction.hpp"
#include "customs/srv/matrix_multiplication_by_number.hpp"
#include "customs/srv/matrix_division_by_number.hpp"
#include "customs/srv/matrix_addition_by_number.hpp"
#include "customs/srv/matrix_subtraction_by_number.hpp"

class MatrixServiceNode : public rclcpp::Node
{
public:
    MatrixServiceNode() : Node("matrix_service")
    {
        this->declare_parameter("name", "MatrixService");
        _name = this->get_parameter("name").as_string();

        _print_matrix_service = this->create_service<customs::srv::PrintMatrix>(
            "print_matrix", std::bind(&MatrixServiceNode::callback_PrintMatrix, this, std::placeholders::_1, std::placeholders::_2));

        _matrix_max_element_service = this->create_service<customs::srv::MatrixMaxElement>(
            "matrix_max_element", std::bind(&MatrixServiceNode::callback_MatrixMaxElement, this, std::placeholders::_1, std::placeholders::_2));

        _matrix_min_element_service = this->create_service<customs::srv::MatrixMinElement>(
            "matrix_min_element", std::bind(&MatrixServiceNode::callback_MatrixMinElement, this, std::placeholders::_1, std::placeholders::_2));

        _matrix_identity_service = this->create_service<customs::srv::MatrixIdentity>(
            "matrix_identity", std::bind(&MatrixServiceNode::callback_MatrixIdentity, this, std::placeholders::_1, std::placeholders::_2));

        _matrix_square_service = this->create_service<customs::srv::MatrixSquare>(
            "matrix_square", std::bind(&MatrixServiceNode::callback_MatrixSquare, this, std::placeholders::_1, std::placeholders::_2));

        _matrix_multiplication_service = this->create_service<customs::srv::MatrixMultiplication>(
            "matrix_multiplication", std::bind(&MatrixServiceNode::callback_MatrixMultiplication, this, std::placeholders::_1, std::placeholders::_2));

        _matrix_inversion_service = this->create_service<customs::srv::MatrixInversion>(
            "matrix_inversion", std::bind(&MatrixServiceNode::callback_MatrixInversion, this, std::placeholders::_1, std::placeholders::_2));

        _matrix_addition_service = this->create_service<customs::srv::MatrixAddition>(
            "matrix_addition", std::bind(&MatrixServiceNode::callback_MatrixAddition, this, std::placeholders::_1, std::placeholders::_2));

        _matrix_subtraction_service = this->create_service<customs::srv::MatrixSubtraction>(
            "matrix_subtraction", std::bind(&MatrixServiceNode::callback_MatrixSubtraction, this, std::placeholders::_1, std::placeholders::_2));

        _matrix_multiplication_by_number_service = this->create_service<customs::srv::MatrixMultiplicationByNumber>(
            "matrix_multiplication_by_number", std::bind(&MatrixServiceNode::callback_MatrixMultiplicationByNumber, this, std::placeholders::_1, std::placeholders::_2));

        _matrix_division_by_number_service = this->create_service<customs::srv::MatrixDivisionByNumber>(
            "matrix_division_by_number", std::bind(&MatrixServiceNode::callback_MatrixDivisionByNumber, this, std::placeholders::_1, std::placeholders::_2));

        _matrix_addition_by_number_service = this->create_service<customs::srv::MatrixAdditionByNumber>(
            "matrix_addition_by_number", std::bind(&MatrixServiceNode::callback_MatrixAdditionByNumber, this, std::placeholders::_1, std::placeholders::_2));

        _matrix_subtraction_by_number_service = this->create_service<customs::srv::MatrixSubtractionByNumber>(
            "matrix_subtraction_by_number", std::bind(&MatrixServiceNode::callback_MatrixSubtractionByNumber, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "%s has been started.", _name.c_str());
    }

private:
    std::string _name = "";

    rclcpp::Service<customs::srv::PrintMatrix>::SharedPtr _print_matrix_service;
    rclcpp::Service<customs::srv::MatrixMaxElement>::SharedPtr _matrix_max_element_service;
    rclcpp::Service<customs::srv::MatrixMinElement>::SharedPtr _matrix_min_element_service;
    rclcpp::Service<customs::srv::MatrixIdentity>::SharedPtr _matrix_identity_service;
    rclcpp::Service<customs::srv::MatrixSquare>::SharedPtr _matrix_square_service;
    rclcpp::Service<customs::srv::MatrixMultiplication>::SharedPtr _matrix_multiplication_service;
    rclcpp::Service<customs::srv::MatrixInversion>::SharedPtr _matrix_inversion_service;
    rclcpp::Service<customs::srv::MatrixAddition>::SharedPtr _matrix_addition_service;
    rclcpp::Service<customs::srv::MatrixSubtraction>::SharedPtr _matrix_subtraction_service;
    rclcpp::Service<customs::srv::MatrixMultiplicationByNumber>::SharedPtr _matrix_multiplication_by_number_service;
    rclcpp::Service<customs::srv::MatrixDivisionByNumber>::SharedPtr _matrix_division_by_number_service;
    rclcpp::Service<customs::srv::MatrixAdditionByNumber>::SharedPtr _matrix_addition_by_number_service;
    rclcpp::Service<customs::srv::MatrixSubtractionByNumber>::SharedPtr _matrix_subtraction_by_number_service;

    void callback_PrintMatrix(const customs::srv::PrintMatrix::Request::SharedPtr request,
                              const customs::srv::PrintMatrix::Response::SharedPtr response)
    {
        std::vector<std::vector<double>> A = _construct_matrix(request->matrix,
                                                               request->n,
                                                               request->m);

        _print_matrix(A);

        response->result = true;
    }

    void callback_MatrixMaxElement(const customs::srv::MatrixMaxElement::Request::SharedPtr request,
                                   const customs::srv::MatrixMaxElement::Response::SharedPtr response)
    {
        response->result = _array_max_element(request->matrix);
    }

    void callback_MatrixMinElement(const customs::srv::MatrixMinElement::Request::SharedPtr request,
                                   const customs::srv::MatrixMinElement::Response::SharedPtr response)
    {
        response->result = _array_min_element(request->matrix);
    }

    void callback_MatrixIdentity(const customs::srv::MatrixIdentity::Request::SharedPtr request,
                                 const customs::srv::MatrixIdentity::Response::SharedPtr response)
    {
        std::vector<std::vector<double>> I = _matrix_identity(request->n);

        response->result = _deconstruct_matrix(I);
    }

    void callback_MatrixSquare(const customs::srv::MatrixSquare::Request::SharedPtr request,
                               const customs::srv::MatrixSquare::Response::SharedPtr response)
    {
        std::vector<std::vector<double>> A = _construct_matrix(request->matrix,
                                                               request->n,
                                                               request->m);

        std::vector<std::vector<double>> A2 = _matrix_square(A);

        response->result = _deconstruct_matrix(A2);
    }

    void callback_MatrixMultiplication(const customs::srv::MatrixMultiplication::Request::SharedPtr request,
                                       const customs::srv::MatrixMultiplication::Response::SharedPtr response)
    {
        std::vector<std::vector<double>> A = _construct_matrix(request->matrix1,
                                                               request->n1,
                                                               request->m1);

        std::vector<std::vector<double>> B = _construct_matrix(request->matrix2,
                                                               request->n2,
                                                               request->m2);

        std::vector<std::vector<double>> AB = _matrix_multiplication(A, B);

        response->result = _deconstruct_matrix(AB);
    }

    void callback_MatrixInversion(const customs::srv::MatrixInversion::Request::SharedPtr request,
                                  const customs::srv::MatrixInversion::Response::SharedPtr response)
    {
        std::vector<std::vector<double>> A = _construct_matrix(request->matrix,
                                                               request->n,
                                                               request->m);

        std::vector<std::vector<double>> invA = _matrix_inversion(A);

        response->result = _deconstruct_matrix(invA);
    }

    void callback_MatrixAddition(const customs::srv::MatrixAddition::Request::SharedPtr request,
                                 const customs::srv::MatrixAddition::Response::SharedPtr response)
    {
        std::vector<std::vector<double>> A = _construct_matrix(request->matrix1,
                                                               request->n1,
                                                               request->m1);

        std::vector<std::vector<double>> B = _construct_matrix(request->matrix2,
                                                               request->n2,
                                                               request->m2);

        std::vector<std::vector<double>> AB = _matrix_addition(A, B);

        response->result = _deconstruct_matrix(AB);
    }

    void callback_MatrixSubtraction(const customs::srv::MatrixSubtraction::Request::SharedPtr request,
                                    const customs::srv::MatrixSubtraction::Response::SharedPtr response)
    {
        std::vector<std::vector<double>> A = _construct_matrix(request->matrix1,
                                                               request->n1,
                                                               request->m1);

        std::vector<std::vector<double>> B = _construct_matrix(request->matrix2,
                                                               request->n2,
                                                               request->m2);

        std::vector<std::vector<double>> AB = _matrix_subtraction(A, B);

        response->result = _deconstruct_matrix(AB);
    }

    void callback_MatrixMultiplicationByNumber(const customs::srv::MatrixMultiplicationByNumber::Request::SharedPtr request,
                                               const customs::srv::MatrixMultiplicationByNumber::Response::SharedPtr response)
    {
        response->result = _array_multiplication_by_number(request->matrix, request->number);
    }

    void callback_MatrixDivisionByNumber(const customs::srv::MatrixDivisionByNumber::Request::SharedPtr request,
                                         const customs::srv::MatrixDivisionByNumber::Response::SharedPtr response)
    {
        response->result = _array_division_by_number(request->matrix, request->number);
    }

    void callback_MatrixAdditionByNumber(const customs::srv::MatrixAdditionByNumber::Request::SharedPtr request,
                                         const customs::srv::MatrixAdditionByNumber::Response::SharedPtr response)
    {
        response->result = _array_addition_by_number(request->matrix, request->number);
    }

    void callback_MatrixSubtractionByNumber(const customs::srv::MatrixSubtractionByNumber::Request::SharedPtr request,
                                            const customs::srv::MatrixSubtractionByNumber::Response::SharedPtr response)
    {
        response->result = _array_subtraction_by_number(request->matrix, request->number);
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

    size_t _longest_element_size(const std::vector<std::vector<double>> A)
    {
        int n = A.size();
        int m = A[0].size();

        size_t max = 0;

        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < m; j++)
            {
                size_t length = std::to_string(A[i][j]).length();

                if (length > max)
                {
                    max = length;
                }
            }
        }

        return max;
    }

    void _print_matrix(const std::vector<std::vector<double>> A)
    {
        size_t n = A.size();
        size_t m = A[0].size();

        size_t width = _longest_element_size(A);

        std::string buffer = "";

        buffer += "\n[";

        for (size_t i = 0; i < n; i++)
        {
            buffer += '[';

            for (size_t j = 0; j < m; j++)
            {
                size_t length = std::to_string(A[i][j]).length();

                std::string blank = "";

                for (size_t k = 0; k < width - length; k++)
                {
                    blank += ' ';
                }

                buffer += blank + std::to_string(A[i][j]);

                if (j == m - 1)
                {
                    break;
                }

                buffer += ", ";
            }

            buffer += ']';

            if (i == n - 1)
            {
                break;
            }

            buffer += ",\n ";
        }

        buffer += "]\n";

        RCLCPP_INFO(this->get_logger(), "%s", buffer.c_str());
    }

    double _array_max_element(const std::vector<double> a)
    {
        size_t n = a.size();

        double max = INT_MIN;

        for (size_t i = 0; i < n; i++)
        {
            if (a[i] > max)
            {
                max = a[i];
            }
        }

        return max;
    }

    double _array_min_element(const std::vector<double> a)
    {
        size_t n = a.size();

        double min = INT_MAX;

        for (size_t i = 0; i < n; i++)
        {
            if (a[i] < min)
            {
                min = a[i];
            }
        }

        return min;
    }

    std::vector<double> _array_multiplication_by_number(const std::vector<double> a, const double number)
    {
        size_t n = a.size();
        std::vector<double> result;

        for (size_t i = 0; i < n; i++)
        {
            result.push_back(a[i] * number);
        }

        return result;
    }

    std::vector<double> _array_division_by_number(const std::vector<double> a, const double number)
    {
        size_t n = a.size();
        std::vector<double> result;

        for (size_t i = 0; i < n; i++)
        {
            result.push_back(a[i] / number);
        }

        return result;
    }

    std::vector<double> _array_addition_by_number(const std::vector<double> a, const double number)
    {
        size_t n = a.size();
        std::vector<double> result;

        for (size_t i = 0; i < n; i++)
        {
            result.push_back(a[i] + number);
        }

        return result;
    }

    std::vector<double> _array_subtraction_by_number(const std::vector<double> a, const double number)
    {
        size_t n = a.size();
        std::vector<double> result;

        for (size_t i = 0; i < n; i++)
        {
            result.push_back(a[i] - number);
        }

        return result;
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

    double _matrix_min_element(const std::vector<std::vector<double>> A)
    {
        size_t n = A.size();
        size_t m = A[0].size();

        double min = INT_MAX;

        for (size_t i = 0; i < n; i++)
        {
            for (size_t j = 0; j < m; j++)
            {
                if (A[i][j] < min)
                {
                    min = A[i][j];
                }
            }
        }

        return min;
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

    std::vector<std::vector<double>> _matrix_square(const std::vector<std::vector<double>> A)
    {
        size_t n = A.size();
        size_t m = A[0].size();

        if (n != m)
        {
            return {{0.0}};
        }

        std::vector<std::vector<double>> A2;

        for (size_t i = 0; i < n; i++)
        {
            std::vector<double> row;

            for (size_t j = 0; j < n; j++)
            {
                row.push_back(0.0);

                for (size_t k = 0; k < n; k++)
                {
                    row[j] += A[i][k] * A[k][j];
                }
            }

            A2.push_back(row);
        }

        return A2;
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

    std::vector<std::vector<double>> _matrix_addition_by_number(const std::vector<std::vector<double>> A, const double number)
    {
        size_t n = A.size();
        size_t m = A[0].size();

        std::vector<std::vector<double>> AB;

        for (size_t i = 0; i < n; i++)
        {
            std::vector<double> row;

            for (size_t j = 0; j < m; j++)
            {
                row.push_back(A[i][j] + number);
            }

            AB.push_back(row);
        }

        return AB;
    }

    std::vector<std::vector<double>> _matrix_subtraction_by_number(const std::vector<std::vector<double>> A, const double number)
    {
        size_t n = A.size();
        size_t m = A[0].size();

        std::vector<std::vector<double>> AB;

        for (size_t i = 0; i < n; i++)
        {
            std::vector<double> row;

            for (size_t j = 0; j < m; j++)
            {
                row.push_back(A[i][j] - number);
            }

            AB.push_back(row);
        }

        return AB;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MatrixServiceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}