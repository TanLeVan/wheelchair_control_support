#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <algorithm>
#include <vector>
#include <cmath>

class LidarFilterNode : public rclcpp::Node {
public:
    LidarFilterNode()
    : Node("lidar_filter_node") {
        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&LidarFilterNode::scan_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("filtered_scan", 10);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto filtered_scan = *msg;
        std::vector<float> filtered_ranges = filter_scan(msg);
        filtered_scan.ranges = filtered_ranges;
        publisher_->publish(filtered_scan);
    }

    std::vector<float> filter_scan(const sensor_msgs::msg::LaserScan::SharedPtr &msg) {
        const auto &ranges = msg->ranges;
        size_t num_ranges = ranges.size();
        std::vector<float> output_ranges(num_ranges, std::numeric_limits<float>::quiet_NaN());

        // Median and Statistical Filters Settings
        int k = 5; // neighbors on each side for median filter
        float stddev_multiplier = 2.0; // threshold multiplier for statistical outlier detection

        // Apply median filter
        for (size_t i = k; i < num_ranges - k; ++i) {
            std::vector<float> window(ranges.begin() + i - k, ranges.begin() + i + k + 1);
            std::nth_element(window.begin(), window.begin() + k, window.end());
            float median = window[k];

            // Calculate local mean and standard deviation
            float mean = 0.0, sq_sum = 0.0;
            for (auto &val : window) {
                mean += val;
                sq_sum += val * val;
            }
            mean /= window.size();
            float variance = (sq_sum / window.size()) - (mean * mean);
            float stddev = std::sqrt(variance);

            // Check if the central value is an outlier
            if (std::abs(ranges[i] - mean) <= stddev_multiplier * stddev) {
                output_ranges[i] = median;
            }
        }

        return output_ranges;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
