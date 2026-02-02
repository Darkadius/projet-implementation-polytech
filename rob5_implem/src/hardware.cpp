#include "hardware.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <cmath> // std::round, std::abs
#include <cstdio> // perror

using std::placeholders::_1;

Hardware::Hardware() : Node("hardware"), integral_error_(0.0), last_time_(this->now())
{
    // Subscription to camera direction, used for task n°2
    cam_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "sens", 10, std::bind(&Hardware::camera_callback, this, _1));

    // Subscription to GPIOD encoder angle feedback
    encod_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
        "angle", 10, std::bind(&Hardware::encod_callback, this, _1));

    // Subscription to magnetometer goal angle
    goal_sub = this->create_subscription<std_msgs::msg::Int32>(
        "goal_angle", 10, std::bind(&Hardware::goal_callback, this, _1));

    // Initialize I2C
    if(Init_i2c() != 0){
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize I2C");
    }
}

Hardware::~Hardware()
{
    // Stop the motor on shutdown (I forgot the motor stop command so I just set speed to 0)
    unsigned char write_data[3] = {
        MOTOR_CW,  
        0x01, 
        0x00 
    };
    write(fd, write_data, sizeof(write_data));
    close(fd);
}

// Initialize I2C communication
int Hardware::Init_i2c()
{
    const char *device = "/dev/i2c-1"; // I2C bus on Raspberry Pi
    fd = open(device, O_RDWR); // Open I2C device
    if (fd < 0) { perror("open"); return 1; }
    if (ioctl(fd, I2C_SLAVE, I2C_ADDR) < 0) { perror("ioctl"); close(fd); return 1; } // Set I2C address and slave mode

    RCLCPP_INFO(this->get_logger(), "I2C initialized at address 0x%X", I2C_ADDR);
    return 0;
}

// Spins the motor based on camera direction input
void Hardware::camera_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data == 0) {
        dir = MOTOR_CW;
        RCLCPP_DEBUG(this->get_logger(), "goin right");
    }
    else {
        dir = MOTOR_CCW;
        RCLCPP_DEBUG(this->get_logger(), "goin left");
    }

    unsigned char write_data[3] = {
        dir,  // Register address/command byte
        0x01, // Channel
        MAX_SPEED // Speed
    };
    write(fd, write_data, sizeof(write_data));
}

// PI control based on encoder feedback and goal angle
void Hardware::encod_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received angle: %d", msg->data);

    // Time calculations
    current_time = this->now();
    dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // Error and direction
    errangl = obj - msg->data; // Get proportional error
    RCLCPP_INFO(this->get_logger(), "Error angle: %d", errangl);

    // Get direction and adjust error with offset
    if (errangl >= 0){
        dir = MOTOR_CW;
        errangl += OFFSET;
    }
    else {
        dir = MOTOR_CCW;
        errangl -= OFFSET;
    }

    // PI controller
    integral_error_ += errangl * dt; // Integrate error over time
    integral_error_ = std::max(-INTEGRAL_MAX, std::min(INTEGRAL_MAX, integral_error_)); // Anti-windup
    double control_output = KP * errangl + KI * integral_error_; // PI control law

    // Limit control output to max speed
    int speed = std::min(MAX_SPEED, (int)std::round(std::abs(control_output)));
    RCLCPP_DEBUG(this->get_logger(), "Control output: %.2f, Speed: %d", control_output, speed);

    // Send command to motor controller
    unsigned char write_data[3] = {
        dir,  // Register address/command byte
        0x01, // Channel
        speed // Speed
    };
    write(fd, write_data, sizeof(write_data));
}

// Change goal angle based on magnetometer input
void Hardware::goal_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received goal angle: %d", msg->data);
    obj = msg->data;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Hardware>());
    rclcpp::shutdown();

    return 0;
}