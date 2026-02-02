#include "magneto.hpp"
#include <cmath>

ImuNode::ImuNode() : Node("imu_node") {
    // ROS2 publishers
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);
    goal_pub_ = this->create_publisher<std_msgs::msg::Int32>("goal_angle", 10);

    // Initialize I2C devices
    init_i2c();

    // Timer to read sensors every 100 ms
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ImuNode::read_and_publish, this));
}

/* ---------- I2C helpers ---------- */
uint8_t ImuNode::read_reg(int fd, uint8_t reg) {
    // Read a byte from the specified register
    uint8_t val;
    write(fd, &reg, 1); // Set register address
    read(fd, &val, 1);  // Read value of the register
    return val;
}

void ImuNode::write_reg(int fd, uint8_t reg, uint8_t val) {
    // Write a byte to the specified register
    uint8_t buf[2] = {reg, val};
    write(fd, buf, 2);
}

int16_t ImuNode::read_word(int fd, uint8_t reg) {
    // Read a 16-bit word from the specified register (big-endian)
    uint8_t high = read_reg(fd, reg);
    uint8_t low  = read_reg(fd, reg + 1);
    return (int16_t)((high << 8) | low);
}

uint8_t ImuNode::mag_read(uint8_t reg) {
    // Read a byte from the magnetometer
    return read_reg(i2c_fd_ak, reg);
}

void ImuNode::mag_write(uint8_t reg, uint8_t val) {
    RCLCPP_INFO(this->get_logger(), "Writing to mag reg 0x%02X value 0x%02X", reg, val);
    write_reg(i2c_fd_ak, reg, val);
}

/* ---------- Init ---------- */
void ImuNode::init_i2c() {
    i2c_fd_icm = open("/dev/i2c-1", O_RDWR); // Open I2C bus
    ioctl(i2c_fd_icm, I2C_SLAVE, ICM_ADDR); // Put ICM20600 in I2C slave mode
    write_reg(i2c_fd_icm, REG_PWR_MGMT_1, 0x01); // Wake up ICM20600
    usleep(10000);

    RCLCPP_INFO(this->get_logger(),
        "ICM20600 WHO_AM_I = 0x%02X",
        read_reg(i2c_fd_icm, REG_WHO_AM_I));

    i2c_fd_ak = open("/dev/i2c-1", O_RDWR); // Open I2C bus for magnetometer
    ioctl(i2c_fd_ak, I2C_SLAVE, AK_ADDR); // Put magnetometer in I2C slave mode

    // Initialize magnetometer
    mag_write(AK_CNTL3, AK_SOFT_RST); // Soft reset for register 3
    usleep(100000);
    mag_write(AK_CNTL2, AK_POWER_DOWN); // Hard reset to power down mode
    usleep(10000);
    mag_write(AK_CNTL2, AK_CONT_100HZ); // Starts continuous measurement mode
    usleep(10000);

    RCLCPP_INFO(this->get_logger(),
        "AK09918 WIA1=0x%02X WIA2=0x%02X",
        mag_read(AK_WIA1), mag_read(AK_WIA2));
}

/* ---------- Read + publish ---------- */
void ImuNode::read_and_publish() {
    RCLCPP_DEBUG(this->get_logger(), "Reading IMU data...");
    auto imu = sensor_msgs::msg::Imu();
    auto mag = sensor_msgs::msg::MagneticField();

    imu.header.stamp = this->now();
    imu.header.frame_id = "imu_link";
    mag.header = imu.header;

    // Read magnetometer data
    mx = (mag_read(0x12) << 8) | mag_read(0x11);
    my = (mag_read(0x14) << 8) | mag_read(0x13);
    mz = (mag_read(0x16) << 8) | mag_read(0x15);
    st2 = mag_read(0x18); // Tells the magnetometer we finished reading

    mag.magnetic_field.x = mx * 1e-6; 
    mag.magnetic_field.y = my * 1e-6;
    mag.magnetic_field.z = mz * 1e-6;

    // Transform mag to goal angle
    angle = atan2(mag.magnetic_field.y, mag.magnetic_field.x); // Atan2 gives angle in radians between -pi and pi
    angle_deg = static_cast<int>(angle * 180.0 / M_PI); // Convert to degrees
    if (angle_deg < 0) {
        angle_deg += 360; // Normalize to [0, 360)
    }

    angle_msg.data = angle_deg + ANGLE_OFFSET; // Apply offset
    RCLCPP_DEBUG(this->get_logger(), "read 0x%02X for x and 0x%02X for y", mx, my);
    goal_pub_->publish(angle_msg);
    mag_pub_->publish(mag);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuNode>());
    rclcpp::shutdown();
    return 0;
}
