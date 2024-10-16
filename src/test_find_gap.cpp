#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <functional>
#include <utility> //for std::par
#include<cmath>
#include <tuple>
#include <set>

class FindGap : public rclcpp::Node
{
public:
    FindGap() : Node("laser_scan_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&FindGap::scan_callback, this, std::placeholders::_1));
        gap_visualize_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "gap_visualize", 10
        );
    }

private:
    struct Gap{
        std::tuple<double, double, double> first_side;  //(x,y,theta)
        std::tuple<double, double, double> second_side; //(x,y,theta)
        int type{};
        std::function<double(double)> square = [](double x) { return x * x; };

        double calculate_width()
        {
            return sqrt(square(std::get<0>(first_side) - std::get<0>(second_side)) + square(std::get<1>(first_side) - std::get<1>(second_side)));
        }
    };

    /**Implementing wrap around behavior access to a std::vector */
    template <typename T>
        T get_wrapped(const std::vector<T>& vec, int index) {
        int n = vec.size();
        
        if (n == 0) {
            throw std::out_of_range("Vector is empty");
        }
        
        // Handle negative indices
        int wrapped_index = ((index % n) + n) % n;
    
    return vec[wrapped_index];
    }

    //Normalize angle to [0, 2*PI)    
    double normalize_angle(double angle) {
        return fmod(angle + 2 * M_PI, 2 * M_PI);
    }

    // Function to compute the counter-clockwise angular distance from 'a' to 'b'
    double angular_distance_ccw(double a, double b) {
        // Normalize both angles to [0, 2*PI)
        a = normalize_angle(a);
        b = normalize_angle(b);

        // Compute the counter-clockwise angular distance
        double distance = normalize_angle(b - a);

        return distance;  // The result is always positive and in [0, 2*PI)
    }

    // Function to compute the clockwise angular distance from 'a' to 'b'
    double angular_distance_cw(double a, double b) {
        // Normalize both angles to [0, 2*PI)
        a = normalize_angle(a);
        b = normalize_angle(b);

        // Compute the counter-clockwise angular distance
        double ccw_distance = normalize_angle(b - a);

        // Compute the clockwise distance: 2*PI - counter-clockwise distance
        double cw_distance = 2 * M_PI - ccw_distance;

        return cw_distance;  // The result is the clockwise distance in [0, 2*PI)
    }

    std::vector<Gap> find_gaps(sensor_msgs::msg::LaserScan& scan, const float robot_radius)
    {
        std::vector<Gap> gaps;
        const float max_range = scan.range_max;
        float angle = scan.angle_min;

        for (auto& range : scan.ranges) {
            if (std::isinf(range)) {
                range = scan.range_max;
            }
        }

        //Forward loop
        for(std::size_t i = 0; i < scan.ranges.size()-1;)
        {

            float current_range = scan.ranges[i];
            float next_range = scan.ranges[i + 1];

            // If type 1 rising discontinuity
            if(next_range - current_range > 2*robot_radius && next_range < max_range)
            {
                std::cout << "Gap start at " << angle*180/M_PI << std::endl;
                Gap gap;
                gap.first_side= std::make_tuple(current_range*cos(angle), current_range*sin(angle), normalize_angle(angle));

                //Assume the second side is the next immediate obstacle point
                gap.second_side = std::make_tuple(next_range*cos(angle + scan.angle_increment), next_range*sin(angle + scan.angle_increment), normalize_angle(angle + scan.angle_increment));

                std::size_t j = i+1;
                double new_angle = angle + scan.angle_increment; //angle of point j
                //If distance from the first side of the gap to this obs point is less than the gap's current width and angular distance travelled is less than pi
                // /Update the second side
                while (angular_distance_ccw(std::get<2>(gap.first_side), new_angle) <= M_PI)
                {
                    if(sqrt(square(get_wrapped(scan.ranges,j)*cos(new_angle) - std::get<0>(gap.first_side)) + square(get_wrapped(scan.ranges,j)*sin(new_angle) - std::get<1>(gap.first_side))) < gap.calculate_width())
                    {
                        std::cout << "Gap end at " << new_angle*180/M_PI << std::endl;
                        gap.second_side = std::make_tuple(get_wrapped(scan.ranges,j)*cos(new_angle), get_wrapped(scan.ranges,j)*sin(new_angle), normalize_angle(new_angle));
                        angle = new_angle; //Angle of point i = angle of point j
                        i = j; 
                    }
                    j++;
                    new_angle += scan.angle_increment;
                }
                gap.type = 1;
                gaps.push_back(gap);
            }

            //If type 2 rising discontinuity
            else if(next_range >=  max_range && current_range < max_range)
            {
                Gap gap;
                gap.first_side= std::make_tuple(current_range*cos(angle), current_range*sin(angle), normalize_angle(angle));
                std::cout << "Gap start at " << angle*180/M_PI << std::endl;
                //Find descending discontinuity
                double j_angle = angle; // Angle of scan point j
                // if (i + 1 >= scan.ranges.size() || i + 2 >= scan.ranges.size()) {
                //     break; // Prevent out-of-bounds access
                // }
                std::size_t j = i;
                while(true)
                {
                    j++;
                    j_angle = j_angle + scan.angle_increment;
                    if(/*((get_wrapped(scan.ranges,j) - get_wrapped(scan.ranges,j+1)) > 2*robot_radius)||*/ (get_wrapped(scan.ranges,j+1) < scan.range_max && get_wrapped(scan.ranges,j) >= scan.range_max))
                    {
                        std::cout << "range of second end    "<<get_wrapped(scan.ranges,j+1)<< std::endl;
                        std::cout << "angle of second end    "<<angle+scan.angle_increment<< std::endl << "\n";
                        angle = j_angle; //Angle of point i = angle of point j + 1
                        i = j; //Continue searching from the next point
                        gap.second_side = std::make_tuple(get_wrapped(scan.ranges,j+1)*cos(angle+scan.angle_increment), 
                                                            get_wrapped(scan.ranges,j+1)*sin(angle+scan.angle_increment),
                                                            normalize_angle(angle + scan.angle_increment));//Second side is scan point j + 1
                        break;
                    }
                }
                gap.type = 2;
                gaps.push_back(gap);
            }
            std::cout << angle*180/M_PI << std::endl;
            angle += scan.angle_increment;
            i++;
        } 

        //Inverse loop search
        angle = scan.angle_max;
        for(int i = scan.ranges.size()-1; i > 1;)
        {

            float current_range = scan.ranges[i];
            float next_range = scan.ranges[i-1];

            // If type 1 rising discontinuity
            if(next_range - current_range > 2*robot_radius && next_range < max_range)
            {
                Gap gap;
                gap.second_side= std::make_tuple(current_range*cos(angle), current_range*sin(angle), normalize_angle(angle));

                //Assume the second side is the next immediate obstacle point
                gap.first_side = std::make_tuple(next_range*cos(angle - scan.angle_increment), next_range*sin(angle - scan.angle_increment), normalize_angle(angle - scan.angle_increment));

                int j = i-1;
                double new_angle = angle - scan.angle_increment; //angle of point j
                //If distance from the first side of the gap to this obs point is less than the gap's current width and angular distance travelled is less than pi
                // Update the second side
                while (angular_distance_cw(std::get<2>(gap.first_side), new_angle) <= M_PI)
                {
                    if(sqrt(square(get_wrapped(scan.ranges,j)*cos(new_angle) - std::get<0>(gap.first_side)) + square(get_wrapped(scan.ranges,j)*sin(new_angle) - std::get<1>(gap.first_side))) < gap.calculate_width())
                    {
                        gap.first_side = std::make_tuple(get_wrapped(scan.ranges,j)*cos(new_angle), get_wrapped(scan.ranges,j)*sin(new_angle), normalize_angle(new_angle));
                        angle = new_angle ; //Angle of point i = angle of point j
                        i = j; 
                    }
                    j--;
                    new_angle -= scan.angle_increment;
                }
                gap.type = 1;
                gaps.push_back(gap);
            }

            //If type 2 rising discontinuity
            else if(next_range >=  max_range && current_range < max_range)
            {
                Gap gap;
                gap.second_side= std::make_tuple(current_range*cos(angle), current_range*sin(angle), normalize_angle(angle));

                //Find descending discontinuity
                double j_angle = angle; // Angle of scan point j

                int j = i;
                while(true)
                {
                    j--;
                    j_angle = j_angle - scan.angle_increment;
                    if(((get_wrapped(scan.ranges,j) - get_wrapped(scan.ranges,j-1)) > 2*robot_radius) || (get_wrapped(scan.ranges,j-1) < scan.range_max && get_wrapped(scan.ranges,j) >= scan.range_max))
                    {
                        angle = j_angle; //Angle of point i = angle of point j - 1
                        i = j; //Continue searching from the next point
                        gap.first_side = std::make_tuple(get_wrapped(scan.ranges,j-1)*cos(angle-scan.angle_increment), 
                                                            get_wrapped(scan.ranges,j-1)*sin(angle-scan.angle_increment),
                                                            normalize_angle(angle - scan.angle_increment));//Second side is scan point j - 1
                        break;
                    }
                }
                gap.type = 2;
                gaps.push_back(gap);
            }
            angle -= scan.angle_increment;
            i--;
        } 

        /*
        Gap elimination:
        Store the indexes to be deleted in an array then delete the coressponding index
        */
        std::set<std::size_t> index_to_be_deleted; 
        for (std::size_t i = 0; i < gaps.size(); ++i) {
            for (std::size_t j = i + 1; j < gaps.size(); ++j) {
                // If angle of first point of gap i is larger than angle of first point of gap j
                // and angle of second point of gap i is smaller than angle of second point of gap j
                if(std::get<2>(gaps[i].first_side) >= std::get<2>(gaps[j].first_side) && std::get<2>(gaps[i].second_side) <= std::get<2>(gaps[j].second_side))
                {
                    //Elminate gap[i]
                    index_to_be_deleted.insert(i);
                }
                else if(std::get<2>(gaps[i].first_side) <= std::get<2>(gaps[j].first_side) && std::get<2>(gaps[i].second_side) >= std::get<2>(gaps[j].second_side))
                {
                    //Elminate gap[j]
                    index_to_be_deleted.insert(j);
                }
            }
        }
        //Deleting gap at index without preserving order of gaps
        if(!index_to_be_deleted.empty())
        {
            for (auto index: index_to_be_deleted)
            {
                std::cout << index << std::endl;
                if(!gaps.empty())
                {
                    gaps[index] = gaps.back();
                    gaps.pop_back();
                }
            }
        }

        return gaps;
    }

    void clear_markers() {
        visualization_msgs::msg::MarkerArray markers_to_delete;
        if (active_marker_ids_.empty()) {
            return;  // Exit the function early if no markers to delete
        }

        for (int id : active_marker_ids_) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_footprint";
            marker.header.stamp = this->now();
            marker.ns = ns_;
            marker.id = id;
            marker.action = visualization_msgs::msg::Marker::DELETE;

            markers_to_delete.markers.push_back(marker);
        }
        gap_visualize_publisher_->publish(markers_to_delete);
        active_marker_ids_.clear();  // Clear the list after deleting
    }

    void visualize_gaps(const std::vector<Gap>& gaps) {
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;

        for (const auto& gap : gaps) {
            // Line marker for the gap (green line)
            visualization_msgs::msg::Marker line_marker;
            line_marker.header.frame_id = "base_footprint";
            line_marker.header.stamp = this->now();
            line_marker.ns = ns_;
            line_marker.id = id++;
            line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::msg::Marker::ADD;
            line_marker.pose.orientation.w = 1.0;
            
            line_marker.scale.x = 0.05;

            line_marker.color.r = 0.0;
            line_marker.color.g = 1.0;  // Green color for the line
            line_marker.color.b = 0.0;
            line_marker.color.a = 1.0;

            line_marker.points.push_back(make_point(gap.first_side));
            line_marker.points.push_back(make_point(gap.second_side));

            marker_array.markers.push_back(line_marker);
            active_marker_ids_.push_back(line_marker.id);  // Track marker id

            // Dot marker for the first side (blue dot)
            visualization_msgs::msg::Marker dot_marker;
            dot_marker.header.frame_id = "base_footprint";
            dot_marker.header.stamp = this->now();
            dot_marker.ns = ns_;
            dot_marker.id = id++;
            dot_marker.type = visualization_msgs::msg::Marker::SPHERE;
            dot_marker.action = visualization_msgs::msg::Marker::ADD;
            dot_marker.pose.orientation.w = 1.0;
            
            // Set position of the dot (same as the first side of the gap)
            dot_marker.pose.position.x = std::get<0>(gap.first_side);
            dot_marker.pose.position.y = std::get<1>(gap.first_side);
            dot_marker.pose.position.z = 0.0;  // Assuming the gaps are 2D in the XY plane

            // Set the scale of the dot marker (dot size)
            dot_marker.scale.x = 0.1;
            dot_marker.scale.y = 0.1;
            dot_marker.scale.z = 0.1;

            // Set the color of the dot marker (blue)
            dot_marker.color.r = 0.0;
            dot_marker.color.g = 0.0;
            dot_marker.color.b = 1.0;  // Blue color for the dot
            dot_marker.color.a = 1.0;

            marker_array.markers.push_back(dot_marker);
            active_marker_ids_.push_back(dot_marker.id);  // Track marker id
        }

        gap_visualize_publisher_->publish(marker_array);
    }
    

    geometry_msgs::msg::Point make_point(const std::tuple<double, double, double>& coords) {
        geometry_msgs::msg::Point point;
        point.x = std::get<0>(coords);
        point.y = std::get<1>(coords);
        return point;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {
        RCLCPP_INFO(this->get_logger(), "Received scan: [angle_min: %f, angle_max: %f]", msg->angle_min, msg->angle_max);
        std::vector<Gap> gaps = this->find_gaps(*msg, 0.4);
        this->clear_markers();
        this->visualize_gaps(gaps);
        RCLCPP_INFO(this->get_logger(), "Publishing %zu markers", gaps.size());
        for (std::size_t i = 0; i < gaps.size(); ++i) {
            std::cout << "Gap " << i + 1 << " type: " << gaps[i].type << std::endl;
        }
    }


    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gap_visualize_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    std::vector<int> active_marker_ids_;
    std::string ns_{"gap_marker"};
    std::function<double(double)> square = [](double x) { return x * x; };

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FindGap>());
    rclcpp::shutdown();
    return 0;
}
