#include "encoder.hpp"
#include <chrono>
#include <cmath>   // std::round

using namespace std::chrono;

EncoderNode::EncoderNode() : Node("encoder_node") {
    // Declare parameters
    this->declare_parameter("chip_name", "gpiochip0");
    this->declare_parameter("pin_a", 5); 
    this->declare_parameter("pin_b", 6);
    this->declare_parameter("publish_rate", 10.0);  // Hz

    // Get parameters
    chip_name_ = this->get_parameter("chip_name").as_string();
    pin_a_ = this->get_parameter("pin_a").as_int();
    pin_b_ = this->get_parameter("pin_b").as_int();
    double publish_rate = this->get_parameter("publish_rate").as_double();

    // Initialize GPIO
    try {
        initGPIO();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize GPIO: %s", e.what());
        return;
    }

    // Create publisher
    encoder_pub_ = this->create_publisher<std_msgs::msg::Int32>("angle", 10);

    // Create timer for publishing
    auto period = duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(
        duration_cast<milliseconds>(period),
        std::bind(&EncoderNode::publishEncoderCount, this)
    );

    // Start GPIO monitoring thread, needs to be a thread as to not lose events
    gpio_thread_ = std::thread(&EncoderNode::monitorGPIO, this);

    RCLCPP_INFO(this->get_logger(), 
                "Encoder node started on pins A=%d, B=%d", pin_a_, pin_b_);
}

EncoderNode::~EncoderNode() {
    // Stop GPIO monitoring thread
    running_ = false;
    if (gpio_thread_.joinable()) {
        gpio_thread_.join();
    }
}

// Open GPIO chip and request lines with edge detection
void EncoderNode::initGPIO() {
    // Open GPIO chip
    chip_ = gpiod::chip(chip_name_);

    // Get GPIO lines
    line_a_ = chip_.get_line(pin_a_);
    line_b_ = chip_.get_line(pin_b_);

    // Request lines as inputs with edge detection
    gpiod::line_request config;
    config.consumer = "encoder_node";
    config.request_type = gpiod::line_request::DIRECTION_INPUT | gpiod::line_request::EVENT_BOTH_EDGES; //Counts both rising and falling edges
    config.flags = 0; // No special flags

    // Apply config to lines
    line_a_.request(config); 
    line_b_.request(config);

    // Read initial states
    last_state_a_ = line_a_.get_value();
    last_state_b_ = line_b_.get_value();

    RCLCPP_INFO(this->get_logger(), 
                "GPIO initialized: A=%d, B=%d", last_state_a_, last_state_b_);
}

// State machine for quadrature decoding using the transition table method
void EncoderNode::monitorGPIO() {
    // transition table: index = (prev<<2) | curr
    const int8_t trans_table[16] = {
        0, -1,  1,  0,
        1,  0,  0, -1,
       -1,  0,  0,  1,
        0,  1, -1,  0
    };

    int prev = (line_a_.get_value() << 1) | line_b_.get_value(); // initial state

    while (running_) {
        // wait for an event on either line with timeout, this is the best way to not miss events.
        while (line_a_.event_wait(milliseconds(0))) { // Non-blocking check
            gpiod::line_event ev = line_a_.event_read(); // Read event
            int a = (ev.event_type == gpiod::line_event::RISING_EDGE) ? 1 : 0; // Determine new A state
            int b = line_b_.get_value(); // Read current B state
            int curr = (a << 1) | b; // Current state
            int delta = trans_table[(prev << 2) | curr]; // Determine state change
            if (delta != 0) encoder_count_ += delta; // Update count
            prev = curr; // Update previous state
        }

        /*Note: I know that theoretically we don't need to monitor events of both lines and that 
        theoretically we should add a timeout as it's safer but practically adding either
        of those things causes the encoder to tweak out and I'm not perfectionist enough to bang my head against the wall more than I did*/
        while (line_b_.event_wait(milliseconds(0))) {
            gpiod::line_event ev = line_b_.event_read();
            int a = line_a_.get_value();
            int b = (ev.event_type == gpiod::line_event::RISING_EDGE) ? 1 : 0;
            int curr = (a << 1) | b;
            int delta = trans_table[(prev << 2) | curr];
            if (delta != 0) encoder_count_ += delta;
            prev = curr;
        }
    }
}

// Publish encoder count to ROS2
void EncoderNode::publishEncoderCount() {
    auto msg = std_msgs::msg::Int32();
    int count = std::round(encoder_count_.load() * 0.44); // Assuming 48 pulses per revolution and quadrature encoding

    // Wrap around at 360 degrees
    if (count >= 360) msg.data = count - 360;
    else msg.data = count;

    encoder_pub_->publish(msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EncoderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}