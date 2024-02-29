#include "rclcpp/rclcpp.hpp"

#include "customs/srv/register_signals_for_noise_by_noise_power.hpp"
#include "customs/srv/register_signals_for_noise_by_snr.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "example_interfaces/msg/float64_multi_array.hpp"

class SignalServiceTestNode : public rclcpp::Node
{
public:
    SignalServiceTestNode() : Node("signal_service_test_node")
    {
        this->declare_parameter("name", "SignalServiceTestNode");
        this->declare_parameter("server_wait", 1.0);
        this->declare_parameter("frequency", 100.0);

        _name = this->get_parameter("name").as_string();
        _server_wait = this->get_parameter("server_wait").as_double();
        _frequency = this->get_parameter("frequency").as_double();
        _dt = 1.0 / _frequency;

        _threads.push_back(std::thread(std::bind(&SignalServiceTestNode::_RegisterSignalsForNoiseByNoisePower, this)));

        _time_publisher = this->create_publisher<example_interfaces::msg::Float64>(
            "time",
            10);

        _signal_publisher_1 = this->create_publisher<example_interfaces::msg::Float64MultiArray>(
            "velocity",
            10);

        _signal_publisher_2 = this->create_publisher<example_interfaces::msg::Float64MultiArray>(
            "positions",
            10);

        _timer = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / _frequency)),
            std::bind(&SignalServiceTestNode::_callback_timer, this));
    }

private:
    std::string _name = "";
    double _server_wait = 0.0;
    double _frequency = 0.0;
    double _time = 0.0;
    double _dt = 0.0;

    std::string _signal_names = "velocity,positions";
    std::vector<double> noise_powers = {-20.0, -20.0};

    std::vector<double> _signals_1 = {0.0};
    std::vector<double> _signals_2 = {0.0, 0.0};

    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr _time_publisher;
    rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr _signal_publisher_1;
    rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr _signal_publisher_2;
    rclcpp::TimerBase::SharedPtr _timer;

    std::vector<std::thread> _threads;

    template <typename T>
    void _print_result(const T result, const std::string name)
    {
        RCLCPP_INFO(this->get_logger(), "%s: %s", name.c_str(), std::to_string(result).c_str());
    }

    void _RegisterSignalsForNoiseByNoisePower()
    {
        auto client = this->create_client<customs::srv::RegisterSignalsForNoiseByNoisePower>("register_signals_for_noise_by_noise_power");
        while (!client->wait_for_service(std::chrono::milliseconds((int)(1000.0 * _server_wait))))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }

        auto request = std::make_shared<customs::srv::RegisterSignalsForNoiseByNoisePower::Request>();
        request->names = _signal_names;
        request->powers = noise_powers;

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

    void _signals_publish()
    {
        auto signals_1 = example_interfaces::msg::Float64MultiArray();
        signals_1.data = _signals_1;
        _signal_publisher_1->publish(signals_1);

        auto signals_2 = example_interfaces::msg::Float64MultiArray();
        signals_2.data = _signals_2;
        _signal_publisher_2->publish(signals_2);
    }

    void _time_publish()
    {
        auto msg = example_interfaces::msg::Float64();
        msg.data = _time;
        _time_publisher->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Time:[%lf]", _time);
    }

    void _calculate_signals()
    {
        _signals_1[0] = std::pow(std::sin(_time), 3) + std::pow(std::cos(_time), 6);

        _signals_2[0] = std::sin(_time);
        _signals_2[1] = std::cos(_time);
    }

    void _callback_timer()
    {
        _time += _dt;

        _time_publish();

        _calculate_signals();

        _signals_publish();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SignalServiceTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
