#include "camera.hpp"
#include <chrono>

CameraNode::CameraNode() : Node("camera_node") {
    // Publisher, speed is fixed, we only want to know the direction
    cam_pub = this->create_publisher<std_msgs::msg::Bool>("sens", 10);

    // Timer, 10 Hz is good for a camera
    auto period = std::chrono::duration<double>(0.1);
    scan = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&CameraNode::timer_callback, this));

    // Initialize camera
    Init_cam();
}

// Initialize camera
void CameraNode::Init_cam() {
    // For video purpose it'd be cool if we could get the camera feed on the pc as well
    cam = cv::VideoCapture("/dev/video0", cv::CAP_V4L2); // Should open the first camera found, webcam on the PC, camera on the PI
    if(!cam.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Error opening camera");
        return;
    }
}

// Timer callback to process image and publish direction
void CameraNode::timer_callback() {
    if(!cam.isOpened()){ 
        RCLCPP_ERROR(this->get_logger(), "Camera not opened");
        cam = cv::VideoCapture("/dev/video0", cv::CAP_V4L2); // Prevents complete failure if camera is disconnected
        return;
    }
    else{
        cam >> frame; // Get current frame
        if(frame.empty()){
            RCLCPP_ERROR(this->get_logger(), "Empty frame");
            return;
        }

        // Process image to get filtered line
        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY); // Grayscale
        cv::threshold(frame, frame, 75, 255, cv::THRESH_BINARY); // Thresholding, only black objects should be selected

        // Get first black pixel
        for (int i = 0; i < frame.rows; i++) {
            for (int j = 0; j < frame.cols; j++) {
                if (frame.at<uchar>(i, j) == 0) {
                    if (j < frame.cols / 2) { // Center pixel is at the right
                        direction = true; // go left
                        RCLCPP_DEBUG(this->get_logger(), "go left");
                    } else { // Center pixel is at the left
                        direction = false; // go right
                        RCLCPP_DEBUG(this->get_logger(), "go right");
                    }
                    i = frame.rows; // break outer loop
                    break; // break inner loop
                }
            }
        }
    }

    msg.data = direction;
    cam_pub->publish(msg);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}