#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/int32.hpp>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>

/* ---------- ICM20600 ---------- */
#define ICM_ADDR 0x69
#define REG_WHO_AM_I   0x75
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H  0x43

/* ---------- AK09918 ---------- */
#define AK_ADDR 0x0C // I2C address of the magnetometer
#define AK_WIA1 0x00 // Who am I register
#define AK_WIA2 0x01 // Who am I register 2
#define AK_CNTL2 0x31 // Control register 2
#define AK_CNTL3 0x32 // Control register 3

#define AK_POWER_DOWN 0x00 // Power down mode
#define AK_CONT_100HZ 0x08 // Continuous measurement mode 2 (100Hz)
#define AK_SOFT_RST 0x01 // Soft reset command

#define ANGLE_OFFSET -60 // Offset to be added to the angle in degrees to account for measuring errors

class ImuNode : public rclcpp::Node {
public:
    ImuNode(); // Constructor

private:
    // I2C file descriptors
    int i2c_fd_icm; // File descriptor for ICM20600
    int i2c_fd_ak;  // File descriptor for magnetometer

    // Angle variables
    int angle_deg; // Goal angle in degrees
    double angle;  // Goal angle in radians

    // Magnetometer readings
    int16_t mx;
    int16_t my;
    int16_t mz;
    int16_t st2; // Status 2 register

    // ROS2 publishers and timer
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr goal_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std_msgs::msg::Int32 angle_msg;

    /* ---------- I2C helpers ---------- */
    uint8_t read_reg(int fd, uint8_t reg);      // Read a byte from a register
    void write_reg(int fd, uint8_t reg, uint8_t val); // Write a byte to a register
    int16_t read_word(int fd, uint8_t reg);     // Read a 16-bit word (big-endian)
    uint8_t mag_read(uint8_t reg);              // Read a byte from magnetometer
    void mag_write(uint8_t reg, uint8_t val);   // Write a byte to magnetometer

    /* ---------- Init ---------- */
    void init_i2c(); // Initialize ICM and magnetometer

    /* ---------- Read + publish ---------- */
    void read_and_publish(); // Read sensor data and publish to ROS2
};
