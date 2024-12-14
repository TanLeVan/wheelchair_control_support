#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <algorithm>

class PolarPoint
{
public:
    double r;
    double theta;
    double value{0.5};
    // Default constructor
    PolarPoint() : r(0), theta(0), value(0.5) {}
    PolarPoint(double r_, double theta_, double value_)
    {
        theta = map_angle_(theta_);
        r = r_;
        value = value_;
    }
    PolarPoint(double r_, double theta_) : PolarPoint(r_, theta_, 0.5){}

private:
    /* Map angle from (-inf, inf) to [0, 2pi) */
    double map_angle_(double theta)
    {
        double twoPi = 2 * M_PI;
        return fmod(fmod(theta, twoPi) + twoPi, twoPi);
    }
};


/**
 * Class to represent a Cell in the Occupancy Grid map.
 * Cell contain mid_point to denote the location of the cell
 * Cell contain virtual obstacles to track the continuous change in position of the robot
 * Number of maximum virtual obstacle in a Cell can be set
 */
class Cell
{
public:
    PolarPoint mid_point;
    std::vector<PolarPoint> virtual_obstacles;
    double angular_resolution;
    double radial_resolution;
    int angular_index; // Angular position of the cell in map
    int radial_index;  // Radial position of the cell in map
    static constexpr int max_virtual_obstacles{5}; // Maximum threshold for virtual obstacles

    Cell(PolarPoint mid_point, double angular_resolution, double radial_resolution, int angular_index, int radial_index)
        : mid_point{mid_point}, angular_resolution{angular_resolution}, radial_resolution{radial_resolution},
          angular_index{angular_index}, radial_index{radial_index} {}

    /** 
     * If Cell is occupied or not
     * **/
    bool is_occupied()
    {
        return get_value() > 0.5;
    }

    double get_value()
    {
        if (virtual_obstacles.empty())
            return 0.0;
        return std::max_element(virtual_obstacles.begin(), virtual_obstacles.end(),
                                [](const PolarPoint &a, const PolarPoint &b) { return a.value < b.value; })
            ->value;
    }

    /**
     * Add new virtual obstacle 
     */
    void update_virtual_obstacles(const PolarPoint &new_point)
    {
        // Add a new virtual obstacle to the obstacle stack
        virtual_obstacles.push_back(new_point);

        // Check if the number of obstacles exceeds the threshold
        if (virtual_obstacles.size() > max_virtual_obstacles)
        {
            aggregate_virtual_obstacles();
        }
    }

    void ensure_virtual_point()
    {
        if (virtual_obstacles.empty())
        {
            virtual_obstacles.emplace_back(mid_point.r, mid_point.theta , 0.5);
        }
    }

    void visualize_cell(double threshold, visualization_msgs::msg::MarkerArray &marker_array, int &marker_id)
    {
        // Visualize the cell if value exceeds threshold
        if (get_value() > threshold)
        {
            visualization_msgs::msg::Marker cell_marker;
            cell_marker.header.frame_id = "base_footprint";
            cell_marker.type = visualization_msgs::msg::Marker::CUBE;
            cell_marker.action = visualization_msgs::msg::Marker::ADD;
            cell_marker.id = marker_id++;

            cell_marker.pose.position.x = mid_point.r * std::cos(mid_point.theta);
            cell_marker.pose.position.y = mid_point.r * std::sin(mid_point.theta);
            cell_marker.pose.position.z = 0;

            cell_marker.scale.x = angular_resolution;
            cell_marker.scale.y = angular_resolution;
            cell_marker.scale.z = 0.1;

            cell_marker.color.a = 0.3;
            cell_marker.color.r = 0.0;
            cell_marker.color.g = 1.0;
            cell_marker.color.b = 0.0;

            marker_array.markers.push_back(cell_marker);
        }
        // Visualize each virtual obstacle
        for (const auto &vop : virtual_obstacles)
        {
            if(vop.value > threshold)
            {
            visualization_msgs::msg::Marker vop_marker;
            vop_marker.header.frame_id = "base_footprint";
            vop_marker.type = visualization_msgs::msg::Marker::SPHERE;
            vop_marker.action = visualization_msgs::msg::Marker::ADD;
            vop_marker.id = marker_id++;

            vop_marker.pose.position.x = vop.r * std::cos(vop.theta);
            vop_marker.pose.position.y = vop.r * std::sin(vop.theta);
            vop_marker.pose.position.z = 0;

            vop_marker.scale.x = 0.02;
            vop_marker.scale.y = 0.02;
            vop_marker.scale.z = 0.02;

            vop_marker.color.a = 1.0;
            vop_marker.color.r = 0.0;
            vop_marker.color.g = 0.0;
            vop_marker.color.b = 1.0; 

            marker_array.markers.push_back(vop_marker);
            }
        }
    }
private:
    void aggregate_virtual_obstacles()
    {
        // Calculate the weighted average position and maximum value
        double sum_r = 0.0;
        double sum_theta = 0.0;
        double max_value = 0.0;

        for (const auto &obstacle : virtual_obstacles)
        {
            sum_r += obstacle.r;
            sum_theta += obstacle.theta;
            max_value = std::max(max_value, obstacle.value);
        }

        double avg_r = sum_r / virtual_obstacles.size();
        double avg_theta = sum_theta / virtual_obstacles.size();

        // Clear the existing obstacles
        virtual_obstacles.clear();

        // Add the new aggregated obstacle
        virtual_obstacles.emplace_back(avg_r, avg_theta, max_value);
    }

};


class EgocircleOccupancyMap
{
public:
    EgocircleOccupancyMap(int num_angular_segments, int num_radial_segments, double sensing_range, bool ensure_virtual_point = false)
        : num_angular_segments{num_angular_segments}, num_radial_segments{num_radial_segments}, sensing_range{sensing_range}
    {
        double angular_resolution = 2 * M_PI / num_angular_segments;
        double radial_resolution = sensing_range / num_radial_segments;

        // Initialize the 2D array of cells
        map.resize(num_angular_segments);
        for (int i = 0; i < num_angular_segments; ++i)
        {
            double theta = (i + 0.5) * angular_resolution;
            //Extend the map one Cell more in radial position to denote Out-of-Range Cell
            map[i].reserve(num_radial_segments + 1); 
            for (int j = 0; j < num_radial_segments + 1; ++j)
            {
                double r = (j + 0.5) * radial_resolution;
                //Out-of-range cell is intialized with a virtual obstacle of value 1 to represent occupied cell
                if (j == num_radial_segments)
                {
                    map[i].emplace_back(PolarPoint(r, theta, 0.5), angular_resolution, radial_resolution, i, j);
                    map[i][j].update_virtual_obstacles(PolarPoint(r, theta, 1.0));
                }

                map[i].emplace_back(PolarPoint(r, theta, 0.5), angular_resolution, radial_resolution, i, j);
            
                // Ensure virtual point if the flag is set
                // This flag is set when the map is first initialized
                if (ensure_virtual_point)
                {
                    // if ((i ==0 && j == 25) || (i == 20 && j == 30) || (i == 90 && j == 45) )
                    // {
                    //     map[i][j].update_virtual_obstacles(PolarPoint(r, theta, 1.0));
                    // }
                    // else
                    // {
                        map[i][j].ensure_virtual_point();
                    // }
                }
            }
        }
    }

    /**
     * Get cell position for a random point 
     */
    Cell &get_corresponding_cell(PolarPoint p)
    {
        int angular_index = static_cast<int>(p.theta / (2 * M_PI / num_angular_segments));
        int radial_index = static_cast<int>(p.r / (sensing_range / num_radial_segments));

        if (angular_index >= 0 && angular_index < num_angular_segments &&
            radial_index >= 0 && radial_index < num_radial_segments)
        {
            return map[angular_index][radial_index];
        }
        throw std::out_of_range("Requested cell is out of map bounds");
    }

    /*Make a full circle scan of the map and return an array of closest cell in each angular segment
     Simulate a laser scan*/
    std::vector<Cell*> scan_map()
    {
        std::vector<Cell*> occupied_cell;
        occupied_cell.reserve(num_angular_segments);
        for (int i=0; i< num_angular_segments; i++)
        {
            for(Cell& cell : map[i])
            {
                if(cell.get_value() > 0.5)
                {
                    occupied_cell.push_back(&cell);
                    break;
                }
            }
        }
        return occupied_cell;
    }

    void transform_map(const Eigen::Matrix3d &transform)
    {
        EgocircleOccupancyMap new_map(num_angular_segments, num_radial_segments, sensing_range);

        for (int i = 0; i < num_angular_segments; ++i)
        {
            for (int j = 0; j < num_radial_segments; ++j)
            {
                Cell &old_cell = map[i][j];
                for (auto &vop : old_cell.virtual_obstacles)
                {
                    Eigen::Vector3d vec;
                    vec[0] = vop.r * std::cos(vop.theta);
                    vec[1] = vop.r * std::sin(vop.theta);
                    vec[2] = 1.0;
                    Eigen::Vector3d transformed = transform * vec;

                    PolarPoint transformed_vop(std::sqrt(transformed[0] * transformed[0] + transformed[1] * transformed[1]),
                                               std::atan2(transformed[1], transformed[0]), vop.value);

                    try
                    {
                        Cell &new_cell = new_map.get_corresponding_cell(transformed_vop);
                        new_cell.update_virtual_obstacles(transformed_vop);
                    }
                    catch (const std::out_of_range &)
                    {
                        // Ignore out-of-bounds VOPs
                    }
                }
            }
        }

        for (int i = 0; i < num_angular_segments; ++i)
        {
            for (int j = 0; j < num_radial_segments; ++j)
            {
                new_map.map[i][j].ensure_virtual_point();
            }
        }

        *this = std::move(new_map);
        std::cout << count_all_virtual_obstacles() << std::endl;
    }

    /*
        Log odd Probability of a cell given a point in lidar scan data lies in the cell
        Can be replace with better model
    */
    double inverse_lidar_model(bool scan_point_inside_cell)
    {
        if (scan_point_inside_cell)
            return 0.8472; // log(0.7/(1-0.3))
        return -0.8472; //log(0.3/(1-0.3))
    } 
    /*
        Update value of a cell based on old value and available scan data
        (Probabilistic robotic)
        @arg decay_factor (from 0->1)
    */
    void update_cell_value(Cell& cell, bool scan_point_inside_cell) {
        double decay_factor{1.0};
        // if (decay_factor < 0 || decay_factor >1)
        // {
        //     std::cout << "Decay factor MUST be within 0 and 1 \n";
        //     decay_factor = 1;
        // }
        for (auto& vob : cell.virtual_obstacles)
        {
            double l_it = decay_factor*log(vob.value/(1.01-vob.value)) + inverse_lidar_model(scan_point_inside_cell); // l_it = l_i{t-1} +  inverse_sensor_model - l_0
            vob.value = exp(l_it)/(1+exp(l_it));
        }
    }

    void update_map_with_lidar_scan(const sensor_msgs::msg::LaserScan& scan) {
        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            double range = std::isfinite(scan.ranges[i])? scan.ranges[i] : sensing_range;
            double angle = scan.angle_min + i * scan.angle_increment;
            // Ignore invalid range values (e.g., inf or NaN)
            if (std::isfinite(range) && range >= 0.0) {
                try {
                    Cell& cell = get_corresponding_cell(PolarPoint(range, angle));
                    //Update all cells in angular segment of found cell until found cell
                    for(int j = 0; j < cell.radial_index; j++)
                    {
                        update_cell_value(map[cell.angular_index][j],  false);
                    }
                    update_cell_value(cell,  true);
                } catch (const std::out_of_range&) {
                    int angular_index = static_cast<int>(angle / (2 * M_PI / num_angular_segments));
                    if (angular_index >= 0 && angular_index < num_angular_segments) {
                        Cell& furthest_cell = map[angular_index].back(); // Furthest cell in the segment
                        for(int j = 0; j < furthest_cell.radial_index; j++)
                        {
                            update_cell_value(map[furthest_cell.angular_index][j], false);
                        }
                        update_cell_value(furthest_cell,  true);
                    }
                }
            }
        }
    }

    // int count_cells_without_virtual_obstacles()
    // {
    //     static int count = 0;
    //     for (int i = 0; i < num_angular_segments; ++i)
    //     {
    //         for (int j = 0; j < num_radial_segments; ++j)
    //         {
    //             if (map[i][j].virtual_obstacles.empty())
    //             {
    //                 ++count;
    //             }
    //         }
    //     }
    //     return count;
    // }

    int count_all_virtual_obstacles()
    {
        int total = 0;
        for (int i = 0; i < num_angular_segments; ++i)
        {
            for (int j = 0; j < num_radial_segments; ++j)
            {
                total += map[i][j].virtual_obstacles.size();
            }
        }
        return total;
    }

    void visualize_map(double threshold, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher)
    {
        visualization_msgs::msg::MarkerArray visualization_array;
        int marker_id = 0;
        std::vector<Cell*> occupied_cells = scan_map();
        for(auto cell: occupied_cells){
            cell->visualize_cell(threshold, visualization_array, marker_id);
        }
        // Publish the marker array
        publisher->publish(visualization_array);
    }


private:
    int num_angular_segments;
    int num_radial_segments;
    double sensing_range;
    std::vector<std::vector<Cell>> map; // 2D array of cells
};
