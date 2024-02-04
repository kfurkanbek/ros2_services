#include "rclcpp/rclcpp.hpp"

#include "customs/srv/factorial.hpp"

class MathServiceNode : public rclcpp::Node
{
public:
    MathServiceNode() : Node("matrix_service")
    {
        this->declare_parameter("name", "MathService");
        _name = this->get_parameter("name").as_string();

        _factorial_service = this->create_service<customs::srv::Factorial>(
            "factorial", std::bind(&MathServiceNode::callback_Factorial, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "%s has been started.", _name.c_str());
    }

private:
    std::string _name = "";

    rclcpp::Service<customs::srv::Factorial>::SharedPtr _factorial_service;

    void callback_Factorial(const customs::srv::Factorial::Request::SharedPtr request,
                            const customs::srv::Factorial::Response::SharedPtr response)
    {
        response->result = _factorial(request->number);
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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MathServiceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

