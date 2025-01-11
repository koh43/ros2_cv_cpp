#include <iostream>
#include <memory>
#include <chrono>
#include <string>
#include <array>

#include <opencv2/opencv.hpp>

#include "ros2_cv_cpp/cam_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

constexpr size_t DEFAULT_QUEUE_SIZE = 10;

class Camera_Publisher : public rclcpp::Node {
public:
    Camera_Publisher() : 
    Node("cam_pub"),
    queue_size_(DEFAULT_QUEUE_SIZE) {
        
    }

    size_t get_qs() const {
        return queue_size_;
    }

    void set_qs(size_t qs) {
        if (qs <= 0) {
            queue_size_ = DEFAULT_QUEUE_SIZE;
        }
        else {
            queue_size_ = qs;
        }
    }

private:
    void cam_pub_cb () {

    }
    size_t queue_size_;
};

int main() {
    cv_camera cap;

    if (!cap.isopen()) {
        std::cerr << "Error: Could not open the camera!" << std::endl;
        return 1;
    }

    cv::Mat frame;

    while (cap.isopen()) {
        cap.read(frame);
        if (frame.empty()) {
            std::cerr << "Error: Empty frame captured!" << std::endl;
            break;
        }
        cv::imshow("*Webcam*", frame);
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    cv::destroyAllWindows();

    return 0;
}