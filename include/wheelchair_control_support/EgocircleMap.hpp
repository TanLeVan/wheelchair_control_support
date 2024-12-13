#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <cmath>

class PolarPoint
{
public:
    double r;
    double theta;
    double value{0.5};
    //Default constructor
    PolarPoint(): r(0), theta(0), value(0.5){};
    PolarPoint(double r_, double theta_, double value_)
    {
        theta = map_angle_(theta_);
        r = r_;
        value = value_;
    }

    bool operator==(const PolarPoint& point){
        return r==point.r && theta == point.theta;
    }

private:
    /*Map angle from (-inf, inf) to [0, 2pi) */
    double map_angle_(double theta)
    {
        double twoPi = 2 * M_PI;
        return fmod(fmod(theta, twoPi) + twoPi, twoPi);
    }
};

class Cell
/*Represent a cell in the map*/
{
public:
    PolarPoint mid_point;
    PolarPoint virtual_ob;
    double angular_resolution;
    double radial_resolution;
    int angular_index; //Angular position of the cell in map
    int radial_index;  //Radial position of the cell in map

    bool odom_updated{false};

    Cell(PolarPoint mid_point, double angular_resolution, double radial_resolution, int angular_index, int radial_index)
    : mid_point{mid_point}, virtual_ob{mid_point} , angular_resolution{angular_resolution}, radial_resolution{radial_resolution}, 
    angular_index{angular_index}, radial_index{radial_index}
    {}

    bool is_occupied()
    {
        if (get_value() > 0.5)
            return true;
        return false;
    }
    double get_value()
    {
        return virtual_ob.value;
    }

    void set_value(double occupancy_value)
    {
        virtual_ob.value =  occupancy_value;
    }
    

};

class EgocircleOccupancyMap
{
public:
    EgocircleOccupancyMap(int num_angular_segments, int num_radial_segments, double sensing_range, bool )
    : num_angular_segments{num_angular_segments}, num_radial_segments{num_radial_segments}, sensing_range{sensing_range}
    {
        double angular_resolution = 2 * M_PI / num_angular_segments;
        double radial_resolution = sensing_range / num_radial_segments;

        // Initialize the 2D array of cells
        map.resize(num_angular_segments);
        for (int i = 0; i < num_angular_segments; ++i) {
            double theta = (i + 0.5) * angular_resolution;
            map[i].reserve(num_radial_segments + 1); // Reserve 1 more than num_radial_segments to represent 
                                                     // out of range
            for (int j = 0; j < num_radial_segments+1; ++j) {
                double r = (j + 0.5) * radial_resolution;
                if(j == num_radial_segments) 
                //If the most outer Cell, always consider as having obstacle
                {
                    map[i].emplace_back(PolarPoint(r, theta, 1.0), angular_resolution, radial_resolution, i, j);
                }

                if (j == 10 && i == 0)
                {
                    map[i].emplace_back(PolarPoint(r, theta, 1.0), angular_resolution, radial_resolution, i, j);
                    ob_point = PolarPoint(r, theta, 1.0);
                    ob_point2 = PolarPoint(r, theta+angular_resolution, 0.5);
                    ob_point3  = PolarPoint(r, theta-angular_resolution, 0.5);
                }
                else
                    map[i].emplace_back(PolarPoint(r, theta, 0.5), angular_resolution, radial_resolution, i, j);
            }
        }
    }

    Cell& get_corresponding_cell(PolarPoint p) {
        // Determine the angular and radial indices
        int angular_index = static_cast<int>(p.theta / (2 * M_PI / num_angular_segments));
        int radial_index = static_cast<int>(p.r / (sensing_range / num_radial_segments));

        // Check if indices are within bounds
        if (angular_index >= 0 && angular_index < num_angular_segments &&
            radial_index >= 0 && radial_index < num_radial_segments) {
            return map[angular_index][radial_index];
        }
        // Throw exception if out of bounds
        throw std::out_of_range("Requested cell is out of map bounds");
    }

    visualization_msgs::msg::Marker make_point_marker(PolarPoint obs, double marker_id)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_footprint";
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "egocircle";
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.id = marker_id;
        marker.pose.position.x = obs.r * std::cos(obs.theta);
        marker.pose.position.y = obs.r * std::sin(obs.theta);
        marker.pose.position.z = 0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;

        marker.color.a = 1.0; // Fully opaque
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        return marker;
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
                    // if (cell.radial_index == num_radial_segments)
                    // {
                    //     std::cout << i<< "no ";
                    // }
                    // else
                    // {
                    //     std::cout << i << "yes ";
                    // }
                    break;
                }
            }
            // if (occupied_cell.back() == &map[i].back()){
                
            //     std::cout << "No occupied cell found in angular segment number: " << i << std::endl;
            // }
        }
        //std::cout << std::endl;
        return occupied_cell;
    }

    void visualize_map(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& publisher) {
        // static int previous_marker_count = 0;
        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(make_point_marker(this->ob_point, 0));
        marker_array.markers.push_back(make_point_marker(this->ob_point2, 1));
        marker_array.markers.push_back(make_point_marker(this->ob_point3, 2));
        std::vector<Cell*> occupied_cells = scan_map();
        for(Cell* p_cell : occupied_cells)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_footprint";
            marker.header.stamp = rclcpp::Clock().now();
            marker.ns = "egocircle";
            marker.id = marker_array.markers.size();
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = p_cell->mid_point.r * std::cos(p_cell->mid_point.theta);
            marker.pose.position.y = p_cell->mid_point.r * std::sin(p_cell->mid_point.theta);
            marker.pose.position.z = 0;
            marker.scale.x = p_cell->angular_resolution;
            marker.scale.y = p_cell->angular_resolution;
            marker.scale.z = 0.1;

            marker.color.a = 0.5; // Fully opaque
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            marker_array.markers.push_back(marker);
            //std::cout << "Theta "<< p_cell->virtual_ob.theta << " r: " << p_cell->virtual_ob.r<<std::endl;
        }


        /*For each radial sector */
        // for (int i = 0; i < num_angular_segments; ++i) {
        //     const Cell* closest_occupied_cell = nullptr;
        //     // Find the first occupied cell in the current angular segment
        //     for (int j = 0; j < num_radial_segments; j++) {
        //         if (map[i][j].is_occupied()) {  // Threshold for occupancy
        //             closest_occupied_cell = &map[i][j];
        //             break; // Exit loop once the first occupied cell is found
        //         }
        //     }
        //     // Visualize the determined cell
        //     if (closest_occupied_cell) {
        //         visualization_msgs::msg::Marker marker;
        //         marker.header.frame_id = "base_footprint";
        //         marker.header.stamp = rclcpp::Clock().now();
        //         marker.ns = "egocircle";
        //         marker.id = marker_array.markers.size();
        //         marker.type = visualization_msgs::msg::Marker::CUBE;
        //         marker.action = visualization_msgs::msg::Marker::ADD;

        //         marker.pose.position.x = closest_occupied_cell->mid_point.r * std::cos(closest_occupied_cell->mid_point.theta);
        //         marker.pose.position.y = closest_occupied_cell->mid_point.r * std::sin(closest_occupied_cell->mid_point.theta);
        //         marker.pose.position.z = 0;
        //         marker.scale.x = closest_occupied_cell->angular_resolution;
        //         marker.scale.y = closest_occupied_cell->angular_resolution;
        //         marker.scale.z = 0.1;

        //         marker.color.a = 0.1; // Fully opaque
        //         marker.color.r = 1.0;
        //         marker.color.g = 0.0;
        //         marker.color.b = 0.0;

                
        //         marker_array.markers.push_back(marker);
        //         marker_array.markers.push_back(make_point_marker(closest_occupied_cell->virtual_ob, 100));
        //         std::cout << "Theta "<< closest_occupied_cell->virtual_ob.theta << " r: " << closest_occupied_cell->virtual_ob.r<<std::endl;
        //     }

        //     else if(!closest_occupied_cell && !map[i].empty())
        //     // If no occupied cell is found within the sensing range, color the outer most cell red
        //     {
        //         visualization_msgs::msg::Marker marker;
        //         marker.header.frame_id = "base_footprint";
        //         marker.header.stamp = rclcpp::Clock().now();
        //         marker.ns = "egocircle";
        //         marker.id = marker_array.markers.size();
        //         marker.type = visualization_msgs::msg::Marker::CUBE;
        //         marker.action = visualization_msgs::msg::Marker::ADD;
        //         marker.pose.position.x = (map[i].back().mid_point.r) * std::cos(map[i].back().mid_point.theta);
        //         marker.pose.position.y = (map[i].back().mid_point.r) * std::sin(map[i].back().mid_point.theta);
        //         marker.pose.position.z = 0;
        //         marker.scale.x = map[i].back().angular_resolution;
        //         marker.scale.y = map[i].back().angular_resolution;
        //         marker.scale.z = 0.1;
        //         marker.color.a = 1.0; // Fully opaque
        //         marker.color.r = 1.0;
        //         marker.color.g = 0.0;
        //         marker.color.b = 0.0;
        //         // Set marker lifetime (e.g., 1 second)
        //         marker_array.markers.push_back(marker);
        //     }
        // }
        publisher->publish(marker_array);
    }

    void transform_map(const Eigen::Matrix3d& transform)
    {
        EgocircleOccupancyMap new_map(num_angular_segments, num_radial_segments, sensing_range);
        Eigen::Vector3d vec;
        for (int i = 0; i < num_angular_segments; i++)
        {
            for(int j = 0;j<num_radial_segments;j++)
            {
                Cell old_cell = map[i][j];
                vec[0] = old_cell.virtual_ob.r * std::cos(old_cell.virtual_ob.theta);
                vec[1] = old_cell.virtual_ob.r * std::sin(old_cell.virtual_ob.theta);
                vec[2] = 1.0;
                Eigen::Vector3d transformed_ob_point = transform * vec;
                double x_new = transformed_ob_point[0];
                double y_new = transformed_ob_point[1];
                PolarPoint new_virtual_ob(std::sqrt(x_new * x_new + y_new * y_new), std::atan2(y_new, x_new), old_cell.get_value());
                try
                {
                    // Map the transformed obs point to the corresponding cell in new map
                    Cell& new_cell = new_map.get_corresponding_cell(new_virtual_ob);
                    // if(!new_cell.odom_updated)
                    // {
                    //     new_cell.odom_updated = true;
                    //     // std::cout << " old_cell position theta and r: " << old_cell.angular_index << " " << old_cell.radial_index << std::endl;
                    //     // std::cout << " new_cell position theta and r: " << new_cell.angular_index << " " << new_cell.radial_index << std::endl;
                    // }
                    // else if(new_cell.odom_updated)
                    // {
                        
                    //     std::cout << " Written in updated cell" << std::endl;
                    // }
                    new_cell.virtual_ob = new_virtual_ob;
                    
                }
                catch (const std::out_of_range&) {
                    std::cout << "Obstacle out of range" << std::endl;
                }
                
            }
        }
        new_map.ob_point = SE2_transform_polar(this->ob_point, transform);
        new_map.ob_point2 = SE2_transform_polar(this->ob_point2, transform);
        new_map.ob_point3 = SE2_transform_polar(this->ob_point3, transform);
        Cell& new_cell = new_map.get_corresponding_cell(new_map.ob_point);
        //std::cout << "Position of new cell 1 is angle " << new_cell.angular_index << " and radial " << new_cell.radial_index << std::endl;
        Cell& new_cell2 = new_map.get_corresponding_cell(new_map.ob_point2);
        // std::cout << "Position of new cell 2 is angle " << new_cell2.angular_index << " and radial " << new_cell2.radial_index << std::endl;
        Cell& new_cell3 = new_map.get_corresponding_cell(new_map.ob_point3);
        // if(new_cell2.angular_index == new_cell.angular_index || new_cell3.angular_index == new_cell.angular_index)
        // {
        //     std::cout << "2 obs point collide" << std::endl;
        // }
        *this = std::move(new_map);
        std::cout << num_of_no_obs_cell() << std::endl;
    }

    int num_of_no_obs_cell()
    {
        static int num{0};
        for (int i = 0; i < num_angular_segments; i++)
        {
            for(int j = 0;j<num_radial_segments;j++){
                if(map[i][j].mid_point == map[i][j].virtual_ob){
                    num +=1;
                }
            }
        }
        return num;
    }

    PolarPoint SE2_transform_polar(PolarPoint obs, const Eigen::Matrix3d& transform)
    {
        PolarPoint new_point;
        Eigen::Vector3d vec;
        vec[0] = obs.r*std::cos(obs.theta);
        vec[1] = obs.r*std::sin(obs.theta);
        vec[2] = 1.0;
        vec = transform * vec;
        new_point = PolarPoint(std::sqrt(vec[0] * vec[0] + vec[1] * vec[1]), std::atan2(vec[1], vec[0]), obs.value);
        return new_point;
    }

private:
    int num_angular_segments;
    int num_radial_segments;
    double sensing_range;
    std::vector<std::vector<Cell>> map;       // 2D array of cells

    PolarPoint ob_point, ob_point2, ob_point3;
};

