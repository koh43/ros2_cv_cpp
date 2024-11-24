#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <stdexcept>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

constexpr size_t DEFAULT_COUNT      = 0;
constexpr size_t DEFAULT_QUEUE_SIZE = 10;
constexpr double DEFAULT_FREQUENCY  = 2.0;

class MinimalPublisher : public rclcpp::Node {
public:
	MinimalPublisher()
	: Node("minimal_publisher"),
	count_(DEFAULT_COUNT),
	queue_size_(DEFAULT_QUEUE_SIZE),
	frequency_(DEFAULT_FREQUENCY) {
		update_publisher();
		update_timer();
	}

	void set_qs(size_t qs) {
		if (qs == 0) {
			RCLCPP_WARN(this->get_logger(), "Queue size cannot be zero. Resetting to default: %zu", DEFAULT_QUEUE_SIZE);
			qs = DEFAULT_QUEUE_SIZE;
		}
		if (queue_size_ != qs) {
			queue_size_ = qs;
			update_publisher();
		}
	}

	size_t get_qs() const {
		return queue_size_;
	}

	void set_frequency(double freq) {
		if (freq <= 0.0) freq = DEFAULT_FREQUENCY;
		if (frequency_ != freq) {
			frequency_ = freq;
			update_timer();
		}
	}

	double get_frequency() const {
		return frequency_;
	}

	void set_count(size_t c) {
		count_ = c;
	}

	size_t get_count() const {
		return count_;
	}

private:
	void timer_callback() {
		auto message = std_msgs::msg::String();
		message.data = "Hello World! " + std::to_string(count_++);
		RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
		publisher_->publish(message);
	}

	void update_publisher() {
		publisher_ = this->create_publisher<std_msgs::msg::String>("topic", queue_size_);
	}

	void update_timer() {
		RCLCPP_INFO(this->get_logger(), "Publishing at frequency: %.2f Hz", frequency_);
		auto interval = std::chrono::duration<double> (1.0/frequency_);
		timer_ = this->create_wall_timer(
			interval, std::bind(&MinimalPublisher::timer_callback, this)
		);
	}

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	size_t count_;
	size_t queue_size_;
	double frequency_;
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);

	double freq = DEFAULT_FREQUENCY;
	size_t qs   = DEFAULT_QUEUE_SIZE; 

	for (int i=0; i < argc; ++i) {
		if (std::string(argv[i]) == "--freq" && i + 1 < argc) {
			std::istringstream iss(argv[i + 1]);
			if (!(iss >> freq)) {
				throw std::invalid_argument("Invalid frequency argument!");
			}
		}
		else if (std::string(argv[i]) == "--queue_size" && i + 1 < argc) {
			std::istringstream iss(argv[i + 1]);
			if (!(iss >> qs)) {
				throw std::invalid_argument("Invalid queue_size argument!");
			}
		}
	}

	try {
		auto node = std::make_shared<MinimalPublisher>();
		node->set_qs(qs);
		node->set_frequency(freq);
		rclcpp::spin(node);
	}
	catch (const std::exception &e){
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error: %s", e.what());
		rclcpp::shutdown();
		return 1;
	}
	rclcpp::shutdown();
	return 0;
}
