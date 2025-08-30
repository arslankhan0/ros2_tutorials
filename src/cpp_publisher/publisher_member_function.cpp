#include <chrono> // For time-related functions like std::chrono::milliseconds
#include <memory> // For smart pointers like std::shared_ptr
#include <string> // For string manipulation functions like std::string
#include "rclcpp/rclcpp.hpp"  // Corrected the include from "rlcpp" to "rclcpp"
#include "std_msgs/msg/string.hpp" // For std_msgs::msg::String

using namespace std::chrono_literals; // For using time literals like 500ms

class MinimalPublisher : public rclcpp::Node   // MinimalPublisher class inherits from rclcpp::Node Class (ROS2 )
{
public:
    MinimalPublisher() : Node("minimal_publisher"), count_(0) // constructor name with inherited class Node (class name), counter variable
    { // publisher creation = class->create_publisher<dataType of message>("topic name", queue size)
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    // timer creation = class->create_wall_timer(duration, callback function(bind - ...
    // function to the current object (this), so the timer knows which object’s function to call.) )
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, ROS 2! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
