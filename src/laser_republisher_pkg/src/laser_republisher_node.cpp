#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserRepublisher : public rclcpp::Node
{
public:
  LaserRepublisher()
  : Node("laser_republisher")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/laser_controller/out", 1000, std::bind(&LaserRepublisher::scan_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("repub_scan", 5);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Check for invalid range values and replace them if necessary
    bool has_invalid_ranges = false;
    for (auto& range : msg->ranges) {
      if (std::isinf(range) || std::isnan(range)) {
        has_invalid_ranges = true;
        range = msg->range_max + 1.0;  // Assign a value beyond the max range
      }
    }

    if (has_invalid_ranges) {
      RCLCPP_WARN(this->get_logger(), "Received scan message with invalid range values, replaced with max range + 1");
    } else {
      RCLCPP_INFO(this->get_logger(), "Received valid scan message with %zu ranges", msg->ranges.size());
    }

    publisher_->publish(*msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserRepublisher>());
  rclcpp::shutdown();
  return 0;
}
