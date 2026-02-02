#pragma once

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <iostream>

class CameraNode : public rclcpp::Node {
public:
    CameraNode(); // Constructor

private:
    // ROS2 publisher and timer
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cam_pub;
    rclcpp::TimerBase::SharedPtr scan;

    bool direction = false; // false:right true:left

    // OpenCV objects
    cv::Mat frame;
    cv::VideoCapture cam;
    std_msgs::msg::Bool msg;

    // Methods
    void Init_cam(); // Initialize the camera
    void timer_callback(); // Timer callback to process the image and publish direction
};
