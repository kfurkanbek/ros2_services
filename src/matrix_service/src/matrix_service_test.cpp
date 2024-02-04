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

class MatrixServiceTestNode : public rclcpp::Node
{
public:
    MatrixServiceTestNode() : Node("matrix_service_test_node")
    {
        this->declare_parameter("name", "MatrixServiceTestNode");
        this->declare_parameter("server_wait", 1.0);

        _name = this->get_parameter("name").as_string();
        _server_wait = this->get_parameter("server_wait").as_double();

        _test_runs();
    }

private:
    std::string _name = "";
    double _server_wait = 0.0;

    std::vector<std::thread> _threads;

    void _test_runs()
    {
        double a_0_0 = 1.0;
        double a_0_1 = 2.0;
        double a_1_0 = 3.0;
        double a_1_1 = 4.0;

        std::vector<std::vector<double>> A = {{a_0_0, a_0_1},
                                              {a_1_0, a_1_1}};

        _threads.push_back(std::thread(std::bind(&MatrixServiceTestNode::_PrintMatrix, this, A)));

        _threads.push_back(std::thread(std::bind(&MatrixServiceTestNode::_MatrixMaxElement, this, A)));

        _threads.push_back(std::thread(std::bind(&MatrixServiceTestNode::_MatrixMinElement, this, A)));

        size_t n = 4;

        _threads.push_back(std::thread(std::bind(&MatrixServiceTestNode::_MatrixIdentity, this, n)));

        _threads.push_back(std::thread(std::bind(&MatrixServiceTestNode::_MatrixSquare, this, A)));

        double b_0_0 = 1.0;
        double b_1_0 = 3.0;

        std::vector<std::vector<double>> B = {{b_0_0},
                                              {b_1_0}};

        _threads.push_back(std::thread(std::bind(&MatrixServiceTestNode::_MatrixMultiplication, this, A, B)));

        _threads.push_back(std::thread(std::bind(&MatrixServiceTestNode::_MatrixInversion, this, A)));

        double c_0_0 = 1.0;
        double c_0_1 = 2.0;
        double c_1_0 = 3.0;
        double c_1_1 = 4.0;

        std::vector<std::vector<double>> C = {{c_0_0, c_0_1},
                                              {c_1_0, c_1_1}};

        _threads.push_back(std::thread(std::bind(&MatrixServiceTestNode::_MatrixAddition, this, A, C)));

        _threads.push_back(std::thread(std::bind(&MatrixServiceTestNode::_MatrixSubtraction, this, A, C)));

        double d_0_0 = 4.0;
        double d_0_1 = 4.0;
        double d_1_0 = 4.0;
        double d_1_1 = 4.0;

        std::vector<std::vector<double>> D = {{d_0_0, d_0_1},
                                              {d_1_0, d_1_1}};

        double number = 0.25;

        _threads.push_back(std::thread(std::bind(&MatrixServiceTestNode::_MatrixMultiplicationByNumber, this, D, number)));

        _threads.push_back(std::thread(std::bind(&MatrixServiceTestNode::_MatrixDivisionByNumber, this, D, number)));

        _threads.push_back(std::thread(std::bind(&MatrixServiceTestNode::_MatrixAdditionByNumber, this, D, number)));

        _threads.push_back(std::thread(std::bind(&MatrixServiceTestNode::_MatrixSubtractionByNumber, this, D, number)));
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

    void _print_matrix(const std::vector<std::vector<double>> A, const std::string name)
    {
        size_t n = A.size();
        size_t m = A[0].size();

        size_t width = _longest_element_size(A);

        std::string buffer = "";

        buffer += name;

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

    template <typename T>
    void _print_result(const T result, const std::string name)
    {
        RCLCPP_INFO(this->get_logger(), "%s: %s", name.c_str(), std::to_string(result).c_str());
    }

    void _PrintMatrix(const std::vector<std::vector<double>> A)
    {
        int n = A.size();
        int m = A[0].size();
        std::vector<double> a = _deconstruct_matrix(A);

        auto client = this->create_client<customs::srv::PrintMatrix>("print_matrix");
        while (!client->wait_for_service(std::chrono::milliseconds((int)(1000.0 * _server_wait))))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }

        auto request = std::make_shared<customs::srv::PrintMatrix::Request>();
        request->matrix = a;
        request->n = n;
        request->m = m;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();

            _print_result(response->result, __func__);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

    void _MatrixMaxElement(const std::vector<std::vector<double>> A)
    {
        std::vector<double> a = _deconstruct_matrix(A);

        auto client = this->create_client<customs::srv::MatrixMaxElement>("matrix_max_element");
        while (!client->wait_for_service(std::chrono::milliseconds((int)(1000.0 * _server_wait))))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }

        auto request = std::make_shared<customs::srv::MatrixMaxElement::Request>();
        request->matrix = a;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();

            _print_result(response->result, __func__);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

    void _MatrixMinElement(const std::vector<std::vector<double>> A)
    {
        std::vector<double> a = _deconstruct_matrix(A);

        auto client = this->create_client<customs::srv::MatrixMinElement>("matrix_min_element");
        while (!client->wait_for_service(std::chrono::milliseconds((int)(1000.0 * _server_wait))))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }

        auto request = std::make_shared<customs::srv::MatrixMinElement::Request>();
        request->matrix = a;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();

            _print_result(response->result, __func__);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

    void _MatrixIdentity(const size_t n)
    {
        auto client = this->create_client<customs::srv::MatrixIdentity>("matrix_identity");
        while (!client->wait_for_service(std::chrono::milliseconds((int)(1000.0 * _server_wait))))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }

        auto request = std::make_shared<customs::srv::MatrixIdentity::Request>();
        request->n = n;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();

            _print_matrix(_construct_matrix(response->result, n, n), __func__);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

    void _MatrixSquare(const std::vector<std::vector<double>> A)
    {
        int n = A.size();
        int m = A[0].size();
        std::vector<double> a = _deconstruct_matrix(A);

        auto client = this->create_client<customs::srv::MatrixSquare>("matrix_square");
        while (!client->wait_for_service(std::chrono::milliseconds((int)(1000.0 * _server_wait))))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }

        auto request = std::make_shared<customs::srv::MatrixSquare::Request>();
        request->matrix = a;
        request->n = n;
        request->m = m;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();

            _print_matrix(_construct_matrix(response->result, n, m), __func__);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

    void _MatrixMultiplication(const std::vector<std::vector<double>> A, const std::vector<std::vector<double>> B)
    {
        int n1 = A.size();
        int m1 = A[0].size();
        std::vector<double> a = _deconstruct_matrix(A);

        int n2 = B.size();
        int m2 = B[0].size();
        std::vector<double> b = _deconstruct_matrix(B);

        auto client = this->create_client<customs::srv::MatrixMultiplication>("matrix_multiplication");
        while (!client->wait_for_service(std::chrono::milliseconds((int)(1000.0 * _server_wait))))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }

        auto request = std::make_shared<customs::srv::MatrixMultiplication::Request>();

        request->matrix1 = a;
        request->n1 = n1;
        request->m1 = m1;

        request->matrix2 = b;
        request->n2 = n2;
        request->m2 = m2;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();

            _print_matrix(_construct_matrix(response->result, n1, m2), __func__);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

    void _MatrixInversion(const std::vector<std::vector<double>> A)
    {
        int n = A.size();
        int m = A[0].size();
        std::vector<double> a = _deconstruct_matrix(A);

        auto client = this->create_client<customs::srv::MatrixInversion>("matrix_inversion");
        while (!client->wait_for_service(std::chrono::milliseconds((int)(1000.0 * _server_wait))))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }

        auto request = std::make_shared<customs::srv::MatrixInversion::Request>();
        request->matrix = a;
        request->n = n;
        request->m = m;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();

            _print_matrix(_construct_matrix(response->result, n, m), __func__);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

    void _MatrixAddition(const std::vector<std::vector<double>> A, const std::vector<std::vector<double>> B)
    {
        int n1 = A.size();
        int m1 = A[0].size();
        std::vector<double> a = _deconstruct_matrix(A);

        int n2 = B.size();
        int m2 = B[0].size();
        std::vector<double> b = _deconstruct_matrix(B);

        auto client = this->create_client<customs::srv::MatrixAddition>("matrix_addition");
        while (!client->wait_for_service(std::chrono::milliseconds((int)(1000.0 * _server_wait))))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }

        auto request = std::make_shared<customs::srv::MatrixAddition::Request>();

        request->matrix1 = a;
        request->n1 = n1;
        request->m1 = m1;

        request->matrix2 = b;
        request->n2 = n2;
        request->m2 = m2;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();

            _print_matrix(_construct_matrix(response->result, n1, m2), __func__);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

    void _MatrixSubtraction(const std::vector<std::vector<double>> A, const std::vector<std::vector<double>> B)
    {
        int n1 = A.size();
        int m1 = A[0].size();
        std::vector<double> a = _deconstruct_matrix(A);

        int n2 = B.size();
        int m2 = B[0].size();
        std::vector<double> b = _deconstruct_matrix(B);

        auto client = this->create_client<customs::srv::MatrixSubtraction>("matrix_subtraction");
        while (!client->wait_for_service(std::chrono::milliseconds((int)(1000.0 * _server_wait))))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }

        auto request = std::make_shared<customs::srv::MatrixSubtraction::Request>();

        request->matrix1 = a;
        request->n1 = n1;
        request->m1 = m1;

        request->matrix2 = b;
        request->n2 = n2;
        request->m2 = m2;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();

            _print_matrix(_construct_matrix(response->result, n1, m2), __func__);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

    void _MatrixMultiplicationByNumber(const std::vector<std::vector<double>> A, double number)
    {
        int n = A.size();
        int m = A[0].size();
        std::vector<double> a = _deconstruct_matrix(A);

        auto client = this->create_client<customs::srv::MatrixMultiplicationByNumber>("matrix_multiplication_by_number");
        while (!client->wait_for_service(std::chrono::milliseconds((int)(1000.0 * _server_wait))))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }

        auto request = std::make_shared<customs::srv::MatrixMultiplicationByNumber::Request>();

        request->matrix = a;
        request->number = number;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();

            _print_matrix(_construct_matrix(response->result, n, m), __func__);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

    void _MatrixDivisionByNumber(const std::vector<std::vector<double>> A, double number)
    {
        int n = A.size();
        int m = A[0].size();
        std::vector<double> a = _deconstruct_matrix(A);

        auto client = this->create_client<customs::srv::MatrixDivisionByNumber>("matrix_division_by_number");
        while (!client->wait_for_service(std::chrono::milliseconds((int)(1000.0 * _server_wait))))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }

        auto request = std::make_shared<customs::srv::MatrixDivisionByNumber::Request>();

        request->matrix = a;
        request->number = number;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();

            _print_matrix(_construct_matrix(response->result, n, m), __func__);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

    void _MatrixAdditionByNumber(const std::vector<std::vector<double>> A, double number)
    {
        int n = A.size();
        int m = A[0].size();
        std::vector<double> a = _deconstruct_matrix(A);

        auto client = this->create_client<customs::srv::MatrixAdditionByNumber>("matrix_addition_by_number");
        while (!client->wait_for_service(std::chrono::milliseconds((int)(1000.0 * _server_wait))))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }

        auto request = std::make_shared<customs::srv::MatrixAdditionByNumber::Request>();

        request->matrix = a;
        request->number = number;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();

            _print_matrix(_construct_matrix(response->result, n, m), __func__);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

    void _MatrixSubtractionByNumber(const std::vector<std::vector<double>> A, double number)
    {
        int n = A.size();
        int m = A[0].size();
        std::vector<double> a = _deconstruct_matrix(A);

        auto client = this->create_client<customs::srv::MatrixSubtractionByNumber>("matrix_subtraction_by_number");
        while (!client->wait_for_service(std::chrono::milliseconds((int)(1000.0 * _server_wait))))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }

        auto request = std::make_shared<customs::srv::MatrixSubtractionByNumber::Request>();

        request->matrix = a;
        request->number = number;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();

            _print_matrix(_construct_matrix(response->result, n, m), __func__);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MatrixServiceTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
