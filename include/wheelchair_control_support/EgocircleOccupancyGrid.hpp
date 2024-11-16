#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <cmath>
// Constants
constexpr double PI = 3.141592653589793;

// Cell structure
struct Cell {
    double r;      // Radial distance (middle point)
    double theta;  // Angular position (middle point)
    double value;  // Occupancy probability (0 to 1)

    double angular_resolution;
    double radial_resolution;

    int angular_index;
    int radial_index;
    //Default constructor
    // Cell()
    //     : r(0.0), theta(0.0), value(0.0), angular_resolution(0.0), radial_resolution(0.0) {}

    Cell(double r, double theta, double value, double angular_res, double radial_res, int angular_index, int radial_index)
        : r(r), theta(theta), value(value), angular_resolution(angular_res), radial_resolution(radial_res), angular_index(angular_index), radial_index(radial_index) {}
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
                map[i].emplace_back(r, theta, 0.5, angular_resolution, radial_resolution, i, j);
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

    /*
        Log odd Probability of a cell given a point in lidar scan data lies in the cell
        Can be replace with better model
    */
    double inverse_lidar_model(bool scan_point_inside_cell)
    {
        if (scan_point_inside_cell)
            return 2.19722; // log(0.9/(1-0.9))
        return -2.19722; //log(0.1/(1-0.1))
    } 
    /*
        Update value of a cell based on old value and available scan data
        (Probabilistic robotic)
        @arg decay_factor (from 0->1)
    */
    void update_cell_value(Cell& cell, bool scan_point_inside_cell) {
        double decay_factor{0.7};
        // if (decay_factor < 0 || decay_factor >1)
        // {
        //     std::cout << "Decay factor MUST be within 0 and 1 \n";
        //     decay_factor = 1;
        // }
        double l_it = decay_factor*log(cell.value/(1-cell.value)) + inverse_lidar_model(scan_point_inside_cell); // l_it = l_i{t-1} +  inverse_sensor_model - l_0
        cell.value = exp(l_it)/(1+exp(l_it));
    }


    void update_map_with_lidar_scan(const sensor_msgs::msg::LaserScan& scan) {
        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            double range = scan.ranges[i];
            double angle = scan.angle_min + i * scan.angle_increment;

            // Ignore invalid range values (e.g., inf or NaN)
            if (std::isfinite(range) && range >= 0.0) {
                try {
                    Cell& cell = get_corresponding_cell(range, angle);
                    //Update all cells in angular segment of found cell until found cell
                    for(int j = 0; j < cell.radial_index; j++)
                    {
                        update_cell_value(map[cell.angular_index][j],  false);
                        std::cout << "False Cell Updated at angle: " <<  angle << " and range: " <<map[cell.angular_index][j].r <<" with value " << map[cell.angular_index][j].value << std::endl;
                    }
                    update_cell_value(cell,  true);
                    std::cout << "True Cell Updated at angle: " <<  angle << " and range: " <<range <<" with value " << cell.value << std::endl;
                } catch (const std::out_of_range&) {
                    int angular_index = static_cast<int>(angle / (2 * PI / num_angular_segments));
                    if (angular_index >= 0 && angular_index < num_angular_segments) {
                        Cell& furthest_cell = map[angular_index].back(); // Furthest cell in the segment
                        for(int j = 0; j < furthest_cell.radial_index; j++)
                        {
                            update_cell_value(map[furthest_cell.angular_index][j], false);
                            std::cout << "False furthest Cell Updated at angle: " <<  angle << " and range: " <<map[furthest_cell.angular_index][j].r <<" with value " << map[furthest_cell.angular_index][j].value << std::endl;
                        }
                        update_cell_value(furthest_cell,  true);
                        std::cout << "True furthest Cell Updated at angle: " <<  angle << " and range: " <<range <<" with value " << furthest_cell.value << std::endl;
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
        // Prepare a 2D std::vector to hold Cartesian coordinates of all cells
        std::vector<std::vector<double>> points(num_angular_segments * num_radial_segments, std::vector<double>(3));

        // Fill the 2D vector with coordinates in Cartesian form
        int index = 0;
        for (int i = 0; i < num_angular_segments; ++i) {
            for (int j = 0; j < num_radial_segments; ++j) {
                const Cell& cell = map[i][j];
                points[index][0] = cell.r * std::cos(cell.theta); // x-coordinate
                points[index][1] = cell.r * std::sin(cell.theta); // y-coordinate
                points[index][2] = 1.0;                           // Homogeneous coordinate
                ++index;
            }
        }

        // Flatten the 2D vector to a 1D vector for Eigen::Map
        std::vector<double> flattened;
        flattened.reserve(num_angular_segments * num_radial_segments * 3);
        for (const auto& row : points) {
            flattened.insert(flattened.end(), row.begin(), row.end());
        }

        // Map the flattened vector to an Eigen matrix (3 x N) where N is the total number of cells
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> points_matrix(
            flattened.data(), 3, num_angular_segments * num_radial_segments);

        // Apply the SE(2) transformation
        Eigen::MatrixXd transformed_points = transform * points_matrix;

        // Create a new map
        EgocircleOccupancyMap new_map(num_angular_segments, num_radial_segments, sensing_range);

        // Process transformed points and map them to the new grid
        index = 0;
        for (int i = 0; i < num_angular_segments; ++i) {
            for (int j = 0; j < num_radial_segments; ++j) {
                const Cell& old_cell = map[i][j];

                // Convert transformed Cartesian coordinates back to polar
                double x_new = transformed_points(0, index);
                double y_new = transformed_points(1, index);
                double r_new = std::sqrt(x_new * x_new + y_new * y_new);
                double theta_new = std::atan2(y_new, x_new);
                if (theta_new < 0) theta_new += 2 * PI; // Normalize theta to [0, 2π]

                try {
                    // Map the transformed polar coordinates to the new grid
                    Cell& new_cell = new_map.get_corresponding_cell(r_new, theta_new);
                    new_cell.value = old_cell.value; // Transfer occupancy value
                } catch (const std::out_of_range&) {
                    // If out of range, assign value to the furthest cell in the same angular segment
                    int angular_index = static_cast<int>(theta_new / (2 * PI / num_angular_segments));
                    if (angular_index >= 0 && angular_index < num_angular_segments) {
                        Cell& furthest_cell = new_map.map[angular_index].back(); // Get the furthest cell
                        furthest_cell.value = 1.0; // Assign occupancy value
                    }
                }

                ++index;
            }
        }

        // Update the current map with the new map
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