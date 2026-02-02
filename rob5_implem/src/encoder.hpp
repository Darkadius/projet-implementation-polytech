#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <gpiod.hpp>
#include <thread>
#include <atomic>
#include <string>
#include <cstdint>

class EncoderNode : public rclcpp::Node {
public:
    EncoderNode(); // Constructor
    ~EncoderNode(); // Destructor

private:
    // GPIO configuration
    std::string chip_name_;
    unsigned int pin_a_;
    unsigned int pin_b_;

    // GPIO objects
    gpiod::chip chip_;
    gpiod::line line_a_;
    gpiod::line line_b_;

    // Encoder state
    std::atomic<int32_t> encoder_count_{0};
    int last_state_a_;
    int last_state_b_;

    // ROS2 publisher and timer
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr encoder_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Thread for GPIO monitoring
    std::thread gpio_thread_;
    std::atomic<bool> running_{true};

    // Methods
    void initGPIO(); // Open GPIO lines and configure edge detection
    void monitorGPIO(); // Thread function to monitor GPIO events
    void publishEncoderCount(); // Publish encoder count to ROS2
};
