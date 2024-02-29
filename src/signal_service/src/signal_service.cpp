#include <vector>
#include <cmath>
#include <random>

#include "rclcpp/rclcpp.hpp"

#include "customs/srv/register_signals_for_noise_by_noise_power.hpp"
#include "customs/srv/register_signals_for_noise_by_snr.hpp"
#include "example_interfaces/msg/float64_multi_array.hpp"

class SignalServiceNode : public rclcpp::Node
{
public:
    SignalServiceNode() : Node("signal_service")
    {
        this->declare_parameter("name", "SignalService");
        _name = this->get_parameter("name").as_string();

        this->declare_parameter("delimiters", ",;");
        _delimiters = this->get_parameter("delimiters").as_string();

        this->declare_parameter("suffix", "_noisy");
        _suffix = this->get_parameter("suffix").as_string();

        this->declare_parameter("seed", 1.0);
        _seed = this->get_parameter("seed").as_double();

        _mersenne_twister_engine.seed(_seed);
        std::normal_distribution d{5.0, 2.0};

        _register_signals_for_noise_by_noise_power_service = this->create_service<customs::srv::RegisterSignalsForNoiseByNoisePower>(
            "register_signals_for_noise_by_noise_power", std::bind(&SignalServiceNode::callback_RegisterSignalsForNoiseByNoisePower, this, std::placeholders::_1, std::placeholders::_2));

        // _register_signals_for_noise_by_snr_service = this->create_service<customs::srv::RegisterSignalsForNoiseBySNR>(
        //     "register_signals_for_noise_by_snr", std::bind(&SignalServiceNode::callback_RegisterSignalsForNoiseBySNR, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "%s has been started.", _name.c_str());
    }

private:
    std::string _name = "";
    std::string _delimiters = "";
    std::string _suffix = "_noisy";
    bool _isInitialized = false;

    std::mt19937 _mersenne_twister_engine;
    double _seed = 0.0;

    // Create custom messages for custom signals

    // RegisterSignalsForNoiseByNoisePower
    rclcpp::Service<customs::srv::RegisterSignalsForNoiseByNoisePower>::SharedPtr _register_signals_for_noise_by_noise_power_service;
    std::vector<std::string> _signal_names_1;
    std::vector<double> _noise_powers;
    std::vector<std::normal_distribution<double>> _normal_distributions_1;
    std::vector<rclcpp::Subscription<example_interfaces::msg::Float64MultiArray>::SharedPtr> _subscriptions_1;
    std::vector<rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr> _publishers_1;

    // RegisterSignalsForNoiseBySNR
    // rclcpp::Service<customs::srv::RegisterSignalsForNoiseBySNR>::SharedPtr _register_signals_for_noise_by_snr_service;
    // std::vector<std::string> _signal_names_2;
    // std::vector<double> _signal_powers;
    // std::vector<double> _snrs;
    // std::vector<rclcpp::Subscription<example_interfaces::msg::Float64MultiArray>::SharedPtr> _subscriptions_2;
    // std::vector<rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr> _publishers_2;

    void callback_RegisterSignalsForNoiseByNoisePower(const customs::srv::RegisterSignalsForNoiseByNoisePower::Request::SharedPtr request,
                                                      const customs::srv::RegisterSignalsForNoiseByNoisePower::Response::SharedPtr response)
    {
        if (_isInitialized)
        {
            response->result = false;
            return;
        }

        _noise_powers = request->powers;
        _create_normal_distributions(_noise_powers);

        _signal_names_1 = _parse_string(request->names, _delimiters);

        if (_noise_powers.size() != _signal_names_1.size())
        {
            response->result = false;
            return;
        }

        if (_signal_names_1.size() > 16)
        {
            response->result = false;
            return;
        }

        for (size_t i = 0; i < _signal_names_1.size(); i++)
        {
            _create_signal_subscriber(i, _signal_names_1[i]);
            _create_signal_publisher(_signal_names_1[i]);
        }


        _isInitialized = true;
        response->result = true;
        return;
    }

    void _create_normal_distributions(const std::vector<double> powers_in_dB)
    {
        for (double power_dB : powers_in_dB)
        {
            double power_Watts = std::pow(10, power_dB / 10);

            std::normal_distribution<double> _normal_distribution (0, std::sqrt(power_Watts));
            _normal_distributions_1.push_back(_normal_distribution);
        }
    }

    std::vector<std::string> _parse_string(const std::string data, const std::string delimiters)
    {
        std::vector<std::string> result;
        std::string token = "";

        for (size_t i = 0; i < data.length(); i++)
        {
            bool isDelimiter = false;
            for (char delimiter : delimiters)
            {
                if (data[i] != delimiter)
                {
                    continue;
                }

                if (token.length() == 0)
                {
                    continue;
                }

                result.push_back(token);
                token = "";

                isDelimiter = true;
                break;
            }

            if (!isDelimiter)
            {
                token += data[i];
            }
        }

        if (token.length() == 0)
        {
            return result;
        }

        result.push_back(token);

        return result;
    }

    double _uniform_noise(double std)
    {
        double value = std::sqrt((double)rand() / RAND_MAX);
        return (value * (6 * std)) - (3 * std);
    }

    std::vector<double> _insert_noise(const std::vector<double> data, const size_t i)
    {
        std::vector<double> noisy_data;

        for (double point : data)
        {
            noisy_data.push_back(point + _normal_distributions_1[i](_mersenne_twister_engine));
        }

        return noisy_data;
    }

    void _create_signal_subscriber(const size_t i, const std::string signal_name)
    {
        rclcpp::Subscription<example_interfaces::msg::Float64MultiArray>::SharedPtr subscriber;

        switch (i)
        {
            break;
        case 0:
            subscriber = this->create_subscription<example_interfaces::msg::Float64MultiArray>(
                signal_name,
                10,
                std::bind(&SignalServiceNode::callback_signal_names_1_0, this, std::placeholders::_1));
            break;
        case 1:
            subscriber = this->create_subscription<example_interfaces::msg::Float64MultiArray>(
                signal_name,
                10,
                std::bind(&SignalServiceNode::callback_signal_names_1_1, this, std::placeholders::_1));
            break;
        case 2:
            subscriber = this->create_subscription<example_interfaces::msg::Float64MultiArray>(
                signal_name,
                10,
                std::bind(&SignalServiceNode::callback_signal_names_1_2, this, std::placeholders::_1));
            break;
        case 3:
            subscriber = this->create_subscription<example_interfaces::msg::Float64MultiArray>(
                signal_name,
                10,
                std::bind(&SignalServiceNode::callback_signal_names_1_3, this, std::placeholders::_1));
            break;
        case 4:
            subscriber = this->create_subscription<example_interfaces::msg::Float64MultiArray>(
                signal_name,
                10,
                std::bind(&SignalServiceNode::callback_signal_names_1_4, this, std::placeholders::_1));
            break;
        case 5:
            subscriber = this->create_subscription<example_interfaces::msg::Float64MultiArray>(
                signal_name,
                10,
                std::bind(&SignalServiceNode::callback_signal_names_1_5, this, std::placeholders::_1));
            break;
        case 6:
            subscriber = this->create_subscription<example_interfaces::msg::Float64MultiArray>(
                signal_name,
                10,
                std::bind(&SignalServiceNode::callback_signal_names_1_6, this, std::placeholders::_1));
            break;
        case 7:
            subscriber = this->create_subscription<example_interfaces::msg::Float64MultiArray>(
                signal_name,
                10,
                std::bind(&SignalServiceNode::callback_signal_names_1_7, this, std::placeholders::_1));
            break;
        case 8:
            subscriber = this->create_subscription<example_interfaces::msg::Float64MultiArray>(
                signal_name,
                10,
                std::bind(&SignalServiceNode::callback_signal_names_1_8, this, std::placeholders::_1));
            break;
        case 9:
            subscriber = this->create_subscription<example_interfaces::msg::Float64MultiArray>(
                signal_name,
                10,
                std::bind(&SignalServiceNode::callback_signal_names_1_9, this, std::placeholders::_1));
            break;
        case 10:
            subscriber = this->create_subscription<example_interfaces::msg::Float64MultiArray>(
                signal_name,
                10,
                std::bind(&SignalServiceNode::callback_signal_names_1_10, this, std::placeholders::_1));
            break;
        case 11:
            subscriber = this->create_subscription<example_interfaces::msg::Float64MultiArray>(
                signal_name,
                10,
                std::bind(&SignalServiceNode::callback_signal_names_1_11, this, std::placeholders::_1));
            break;
        case 12:
            subscriber = this->create_subscription<example_interfaces::msg::Float64MultiArray>(
                signal_name,
                10,
                std::bind(&SignalServiceNode::callback_signal_names_1_12, this, std::placeholders::_1));
            break;
        case 13:
            subscriber = this->create_subscription<example_interfaces::msg::Float64MultiArray>(
                signal_name,
                10,
                std::bind(&SignalServiceNode::callback_signal_names_1_13, this, std::placeholders::_1));
            break;
        case 14:
            subscriber = this->create_subscription<example_interfaces::msg::Float64MultiArray>(
                signal_name,
                10,
                std::bind(&SignalServiceNode::callback_signal_names_1_14, this, std::placeholders::_1));
            break;
        case 15:
            subscriber = this->create_subscription<example_interfaces::msg::Float64MultiArray>(
                signal_name,
                10,
                std::bind(&SignalServiceNode::callback_signal_names_1_15, this, std::placeholders::_1));
            break;
        default:
            break;
        }

        _subscriptions_1.push_back(subscriber);
    }

    void _create_signal_publisher(const std::string signal_name)
    {
        rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr publisher;

        publisher = this->create_publisher<example_interfaces::msg::Float64MultiArray>(
            signal_name + _suffix,
            10);

        _publishers_1.push_back(publisher);
    }

    void callback_signal_names_1_0(const example_interfaces::msg::Float64MultiArray::SharedPtr msg)
    {
        std::vector<double> noisy_data = _insert_noise(msg->data, 0);

        auto new_msg = example_interfaces::msg::Float64MultiArray();
        new_msg.data = noisy_data;

        _publishers_1[0]->publish(new_msg);
    }

    void callback_signal_names_1_1(const example_interfaces::msg::Float64MultiArray::SharedPtr msg)
    {
        std::vector<double> noisy_data = _insert_noise(msg->data, 1);

        auto new_msg = example_interfaces::msg::Float64MultiArray();
        new_msg.data = noisy_data;

        _publishers_1[1]->publish(new_msg);
    }

    void callback_signal_names_1_2(const example_interfaces::msg::Float64MultiArray::SharedPtr msg)
    {
        std::vector<double> noisy_data = _insert_noise(msg->data, 2);

        auto new_msg = example_interfaces::msg::Float64MultiArray();
        new_msg.data = noisy_data;

        _publishers_1[2]->publish(new_msg);
    }

    void callback_signal_names_1_3(const example_interfaces::msg::Float64MultiArray::SharedPtr msg)
    {
        std::vector<double> noisy_data = _insert_noise(msg->data, 3);

        auto new_msg = example_interfaces::msg::Float64MultiArray();
        new_msg.data = noisy_data;

        _publishers_1[3]->publish(new_msg);
    }

    void callback_signal_names_1_4(const example_interfaces::msg::Float64MultiArray::SharedPtr msg)
    {
        std::vector<double> noisy_data = _insert_noise(msg->data, 4);

        auto new_msg = example_interfaces::msg::Float64MultiArray();
        new_msg.data = noisy_data;

        _publishers_1[4]->publish(new_msg);
    }

    void callback_signal_names_1_5(const example_interfaces::msg::Float64MultiArray::SharedPtr msg)
    {
        std::vector<double> noisy_data = _insert_noise(msg->data, 5);

        auto new_msg = example_interfaces::msg::Float64MultiArray();
        new_msg.data = noisy_data;

        _publishers_1[5]->publish(new_msg);
    }

    void callback_signal_names_1_6(const example_interfaces::msg::Float64MultiArray::SharedPtr msg)
    {
        std::vector<double> noisy_data = _insert_noise(msg->data, 6);

        auto new_msg = example_interfaces::msg::Float64MultiArray();
        new_msg.data = noisy_data;

        _publishers_1[6]->publish(new_msg);
    }

    void callback_signal_names_1_7(const example_interfaces::msg::Float64MultiArray::SharedPtr msg)
    {
        std::vector<double> noisy_data = _insert_noise(msg->data, 7);

        auto new_msg = example_interfaces::msg::Float64MultiArray();
        new_msg.data = noisy_data;

        _publishers_1[7]->publish(new_msg);
    }

    void callback_signal_names_1_8(const example_interfaces::msg::Float64MultiArray::SharedPtr msg)
    {
        std::vector<double> noisy_data = _insert_noise(msg->data, 8);

        auto new_msg = example_interfaces::msg::Float64MultiArray();
        new_msg.data = noisy_data;

        _publishers_1[8]->publish(new_msg);
    }

    void callback_signal_names_1_9(const example_interfaces::msg::Float64MultiArray::SharedPtr msg)
    {
        std::vector<double> noisy_data = _insert_noise(msg->data, 9);

        auto new_msg = example_interfaces::msg::Float64MultiArray();
        new_msg.data = noisy_data;

        _publishers_1[9]->publish(new_msg);
    }

    void callback_signal_names_1_10(const example_interfaces::msg::Float64MultiArray::SharedPtr msg)
    {
        std::vector<double> noisy_data = _insert_noise(msg->data, 10);

        auto new_msg = example_interfaces::msg::Float64MultiArray();
        new_msg.data = noisy_data;

        _publishers_1[10]->publish(new_msg);
    }

    void callback_signal_names_1_11(const example_interfaces::msg::Float64MultiArray::SharedPtr msg)
    {
        std::vector<double> noisy_data = _insert_noise(msg->data, 11);

        auto new_msg = example_interfaces::msg::Float64MultiArray();
        new_msg.data = noisy_data;

        _publishers_1[11]->publish(new_msg);
    }

    void callback_signal_names_1_12(const example_interfaces::msg::Float64MultiArray::SharedPtr msg)
    {
        std::vector<double> noisy_data = _insert_noise(msg->data, 12);

        auto new_msg = example_interfaces::msg::Float64MultiArray();
        new_msg.data = noisy_data;

        _publishers_1[12]->publish(new_msg);
    }

    void callback_signal_names_1_13(const example_interfaces::msg::Float64MultiArray::SharedPtr msg)
    {
        std::vector<double> noisy_data = _insert_noise(msg->data, 13);

        auto new_msg = example_interfaces::msg::Float64MultiArray();
        new_msg.data = noisy_data;

        _publishers_1[13]->publish(new_msg);
    }

    void callback_signal_names_1_14(const example_interfaces::msg::Float64MultiArray::SharedPtr msg)
    {
        std::vector<double> noisy_data = _insert_noise(msg->data, 14);

        auto new_msg = example_interfaces::msg::Float64MultiArray();
        new_msg.data = noisy_data;

        _publishers_1[14]->publish(new_msg);
    }

    void callback_signal_names_1_15(const example_interfaces::msg::Float64MultiArray::SharedPtr msg)
    {
        std::vector<double> noisy_data = _insert_noise(msg->data, 15);

        auto new_msg = example_interfaces::msg::Float64MultiArray();
        new_msg.data = noisy_data;

        _publishers_1[15]->publish(new_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SignalServiceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
