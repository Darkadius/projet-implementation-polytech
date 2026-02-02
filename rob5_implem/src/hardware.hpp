#pragma once

#include <memory>
#include <algorithm> // std::min, std::max
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"

// I2C and motor definitions
#define I2C_ADDR 0x14 // I2C address of the motor controller
#define MOTOR_CW 0x02 // Clockwise command
#define MOTOR_CCW 0x03 // Counter-clockwise command
#define OFFSET 20 // Offset to overcome motor deadzone
#define MAX_SPEED 75
#define KP 1.5
#define KI 0.1
#define INTEGRAL_MAX 100.0

class Hardware : public rclcpp::Node
{
public:
    Hardware(); // Constructor
    ~Hardware(); // Destructor

private:
    // Callbacks for ROS subscriptions
    void camera_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void encod_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void goal_callback(const std_msgs::msg::Int32::SharedPtr msg);

    // I2C initialization
    int Init_i2c();

    // Member variables
    double integral_error_;
    int obj = 0;
    rclcpp::Time last_time_;
    double dt;
    rclcpp::Time current_time;
    int errangl;
    unsigned char dir;
    int fd;

    // ROS subscriptions
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr encod_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr goal_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cam_subscription_;
};
