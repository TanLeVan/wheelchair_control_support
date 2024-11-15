#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <stdexcept>

// Constants
constexpr double PI = 3.141592653589793;

// Cell structure
struct Cell {
    double r;      // Radial distance (middle point)
    double theta;  // Angular position (middle point)
    double value;  // Occupancy probability (0 to 1)

    double angular_resolution;
    double radial_resolution;

    //Default constructor
    // Cell()
    //     : r(0.0), theta(0.0), value(0.0), angular_resolution(0.0), radial_resolution(0.0) {}

    Cell(double r, double theta, double value, double angular_res, double radial_res)
        : r(r), theta(theta), value(value), angular_resolution(angular_res), radial_resolution(radial_res) {}
};

// EgocircleOccupancyMap class
class EgocircleOccupancyMap {
public:
    EgocircleOccupancyMap(int num_angular_segments, int num_radial_segments, double sensing_range)
        : num_angular_segments(num_angular_segments), num_radial_segments(num_radial_segments), sensing_range(sensing_range) {
        double angular_resolution = 2 * PI / num_angular_segments;
        double radial_resolution = sensing_range / num_radial_segments;

        // Initialize the 2D array of cells
        map.resize(num_angular_segments);
        for (int i = 0; i < num_angular_segments; ++i) {
            double theta = (i + 0.5) * angular_resolution;
            map[i].reserve(num_radial_segments); // Reserve space to avoid multiple reallocations
            for (int j = 0; j < num_radial_segments; ++j) {
                double r = (j + 0.5) * radial_resolution;
                map[i].emplace_back(r, theta, 0.0, angular_resolution, radial_resolution);
        }
    }
    }

    Cell& get_corresponding_cell(double r, double theta) {
        // Normalize theta to the range [0, 2 * PI]
        if (theta < 0) theta += 2 * PI;

        // Determine the angular and radial indices
        int angular_index = static_cast<int>(theta / (2 * PI / num_angular_segments));
        int radial_index = static_cast<int>(r / (sensing_range / num_radial_segments));

        // Check if indices are within bounds
        if (angular_index >= 0 && angular_index < num_angular_segments &&
            radial_index >= 0 && radial_index < num_radial_segments) {
            return map[angular_index][radial_index];
        }

        // Throw exception if out of bounds
        throw std::out_of_range("Requested cell is out of map bounds");
    }

    void update_cell_value(Cell& cell, double new_value) {
        cell.value = new_value; // Apply probabilistic update here (e.g., Bayesian update)
    }


    void update_map_with_lidar_scan(const sensor_msgs::msg::LaserScan& scan) {
        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            double range = scan.ranges[i];
            double angle = scan.angle_min + i * scan.angle_increment;

            // Ignore invalid range values (e.g., inf or NaN)
            if (std::isfinite(range) && range >= 0.0) {
                try {
                    Cell& cell = get_corresponding_cell(range, angle);
                    update_cell_value(cell, 1.0); // Update with appropriate probability
                    std::cout << "Cell Updated at angle: " <<  angle << " and range: " <<range << std::endl;
                } catch (const std::out_of_range&) {
                    int angular_index = static_cast<int>(angle / (2 * PI / num_angular_segments));
                    if (angular_index >= 0 && angular_index < num_angular_segments) {
                        Cell& furthest_cell = map[angular_index].back(); // Furthest cell in the segment
                        update_cell_value(furthest_cell, 1.0);
                        std::cout << "Furthest Cell Updated at angle: " << angle << " (Out of Range)" << std::endl;
                    }
                }
            }
        }
    }


    std::pair<double, double> transform_point_polar(const std::pair<double, double>& polar_point, const Eigen::Matrix3d& transform) {
        // Extract polar coordinates
        double r = polar_point.first;
        double theta = polar_point.second;

        // Convert polar to Cartesian coordinates
        double x = r * std::cos(theta);
        double y = r * std::sin(theta);

        // Apply the SE(2) transformation
        Eigen::Vector3d cartesian_point(x, y, 1.0);
        Eigen::Vector3d transformed_point = transform * cartesian_point;

        // Convert back to polar coordinates
        double r_transformed = std::sqrt(transformed_point.x() * transformed_point.x() + transformed_point.y() * transformed_point.y());
        double theta_transformed = std::atan2(transformed_point.y(), transformed_point.x());
        if (theta_transformed < 0) theta_transformed += 2 * PI; // Normalize θ to [0, 2π]

        return {r_transformed, theta_transformed};
    }

    void transform_map(const Eigen::Matrix3d& transform) {
        EgocircleOccupancyMap new_map(num_angular_segments, num_radial_segments, sensing_range);

        for (int i = 0; i < num_angular_segments; ++i) {
            for (int j = 0; j < num_radial_segments; ++j) {
                const Cell& old_cell = map[i][j];

                // Transform the point using the new function
                auto [r_new, theta_new] = transform_point_polar({old_cell.r, old_cell.theta}, transform);

                try {
                    // Use polar coordinates to find the corresponding cell
                    Cell& new_cell = new_map.get_corresponding_cell(r_new, theta_new);
                    new_cell.value = old_cell.value; // Transfer the occupancy value
                } catch (const std::out_of_range&) {
                    // Ignore cells that are outside the bounds of the new map
                }
            }
        }

        *this = std::move(new_map);
    }

    void visualize_map(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& publisher) {
    visualization_msgs::msg::MarkerArray marker_array;

    for (int i = 0; i < num_angular_segments; ++i) {
        const Cell* closest_occupied_cell = nullptr;

        // Find the first occupied cell in the current angular segment
        for (const auto& cell : map[i]) {
            if (cell.value > 0.5) {  // Threshold for occupancy
                closest_occupied_cell = &cell;
                break; // Exit loop once the first occupied cell is found
            }
        }

        // If no occupied cell is found, use the furthest cell
        if (!closest_occupied_cell && !map[i].empty()) {
            closest_occupied_cell = &map[i].back();
        }

        // Visualize the determined cell
        if (closest_occupied_cell) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_scan";
            marker.header.stamp = rclcpp::Clock().now();
            marker.ns = "egocircle";
            marker.id = marker_array.markers.size();
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = closest_occupied_cell->r * std::cos(closest_occupied_cell->theta);
            marker.pose.position.y = closest_occupied_cell->r * std::sin(closest_occupied_cell->theta);
            marker.pose.position.z = 0;
            marker.scale.x = closest_occupied_cell->radial_resolution;
            marker.scale.y = closest_occupied_cell->radial_resolution;
            marker.scale.z = 0.1;

            marker.color.a = 1.0; // Fully opaque
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            // Set marker lifetime (e.g., 1 second)
            marker.lifetime = rclcpp::Duration::from_seconds(1.0);

            marker_array.markers.push_back(marker);
        }
    }

        publisher->publish(marker_array);
    }

private:
    int num_angular_segments;                     // Number of angular segments
    int num_radial_segments;                      // Number of radial segments
    double sensing_range;                         // Maximum sensing range
    std::vector<std::vector<Cell>> map;           // 2D array of cells
};