#ifndef FIND_GAP_HPP_
#define FIND_GAP_HPP_

#include "wheelchair_control_support/Gap.hpp"
#include <cmath>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <iostream>
#include "visualization_msgs/msg/marker.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>


/**
 * Assumption:
 * received scan message is 360 degree 2D scan
 */
class FindGap
{
private:
    const double mk_gap_length_threshold{0.7}; //Threshold for gap length. Gap < threshold will be omitted
    const double mk_gap_angle_threshold{M_PI}; // Gap with arc anngle > threshold will be ommited
    const double mk_merged_gap_angle_threshold {M_PI/2}; // 

public:
    /**
     * Gap is found using 2 method
     * Type 1: If distance from consecutive scan points > mk_gap_length_threshold =>gap
     * Type 2: If distance between 2 points > mk_gap_length_threshold and all points between 
     * the 2 points has maximum range
     * 
     * Caution: The last gap need to be handled seperately as the gap can go around the scan
     */
    void find_gap_from_scan(const sensor_msgs::msg::LaserScan &scan, std::vector<Gap>& observed_gap)
    {
        observed_gap.clear();
        if(scan.ranges.empty()) return;
        double max_scan_dist = *std::max_element(scan.ranges.begin(), scan.ranges.end());
        // double min_scan_dist = *std::min_element(scan.ranges.begin(), scan.ranges.end());
        bool last_point_in_gap; // If the last point is within type 2 gap or not
        std::string frame = scan.header.frame_id;

        double last_dist; // Distance of last point
        double curr_dist = scan.ranges[scan.ranges.size()-1]; //Distance of current point
        last_point_in_gap = curr_dist >= max_scan_dist; //If the previous scan point belong to a gap
        // Default the left of the first gap to the last scan point
        int gap_lidx = scan.ranges.size()-1;
        double gap_ldist{curr_dist};
        for (std::size_t it = 0; it < scan.ranges.size(); it++)
        {
            last_dist = curr_dist;
            curr_dist = scan.ranges[it];
            double scan_diff = std::abs(curr_dist - last_dist);

            /*
            * Detection of type 1 gap.
            * If scan_diff > a threshold (robot diameter) and curr_dist and last_dist is not infinity
            * then this is a type 1 gap.
            */
            if(scan_diff >= mk_gap_length_threshold && !std::isinf(curr_dist) && !std::isinf(last_dist))
            {
                gap_lidx = (it==0) ? scan.ranges.size() - 1 : it - 1; // Handle the case where it = 0
                Gap detected_gap(frame, gap_lidx , last_dist, scan.ranges.size());
                detected_gap.add_right_information(it, curr_dist);
                if (detected_gap.calculate_gap_arc_angle() <= mk_gap_angle_threshold){observed_gap.push_back(detected_gap);}
            }

            /**
             * Detection of type 2 gap
             * If one of the two curr_dist and last_dist is infinity and the other is not
             */
            if((curr_dist>=max_scan_dist) != (last_dist >= max_scan_dist))
            {   
                //If last_point_in_gap, then this mark the end of a gap. 
                // Create a gap from gap_lidx to current it
                if(last_point_in_gap)
                {
                    /* Handle the case when last point value is infinity
                        The type 2 gap that last point belong to will be handled seperately
                    */
                    if (gap_lidx == scan.ranges.size() - 1)
                    {
                        last_point_in_gap = false;
                    }
                    else
                    {
                        last_point_in_gap = false;
                        Gap detected_gap(frame, gap_lidx, gap_ldist, scan.ranges.size());
                        detected_gap.add_right_information(it,curr_dist);
                        if(detected_gap.get_gap_length() >= mk_gap_length_threshold && detected_gap.calculate_gap_arc_angle() <= mk_gap_angle_threshold)
                        {
                            observed_gap.push_back(detected_gap);
                        }
                    }
                }
                else  // If not last_point_in_gap, then this mark the beginning of type 2 gap
                {
                    gap_lidx = (it==0) ? scan.ranges.size() - 1 : it - 1; // Handle the case where it = 0
                    gap_ldist = last_dist;
                    last_point_in_gap = true;
                }
            }
        }

        /* Handle the last gap
         After scan through the gap, if still last_point_in_gap, then there is a type 2 gap that last point belong 
         to that is not closed. Need to close this gap
        */
        if(last_point_in_gap)
        {
            Gap detected_gap(frame, gap_lidx, gap_ldist, scan.ranges.size());
            // Loop through scan one more time to find the right side of the last gap
            for (std::size_t it = 0; it < scan.ranges.size(); it++)
            {
                if(!std::isinf(scan.ranges[it]))
                {
                    detected_gap.add_right_information(it, scan.ranges[it]);
                    if(detected_gap.get_gap_length() > mk_gap_length_threshold  && detected_gap.calculate_gap_arc_angle() <= mk_gap_angle_threshold)
                    {
                        observed_gap.push_back(detected_gap);
                    }
                    break;
                }
            }
        }
    }

    /**
     * If gap is swept gap or axial right gap, consider merging with  previously found gap if conditions are met
     * Conditions:
     * 1: New gap MUST have arc angle < mk_merged_gap_angle_threshold 
     * 2: Points within new gap must have distance > distance from 2 side 
     */
    void merge_gap(const sensor_msgs::msg::LaserScan &scan, std::vector<Gap>& observed_gap)
    {
        if(observed_gap.empty()) return;

       /*
        Rotate observed_gap so that the first element is the first left type radial gap
       */
       int first_left_radial_index{-1};
       for (int i = 0; i < observed_gap.size();i++)
        {
            if(observed_gap[i].is_left_type() && observed_gap[i].is_radial())
            {
                first_left_radial_index = i;
                break;
            }
        }
        if (first_left_radial_index != -1) {
            std::rotate(observed_gap.begin(), observed_gap.begin() + first_left_radial_index, observed_gap.end());
        }

        /**
        * Begining merging gap
        * If left type radial gap, then cannot merge
        * If axial or right type radial gap, merge if possible
        * 
        * Possible merge: New gap sastify 2 condition:
        * merge_angle_cond: Arc angle < angle threshold (merge_angle_condition)
        * merge_distance_cond: Between 2 edge point in the new gap, no point have length > than the 2 edge point
        ***/
        std::vector<Gap> merged_gaps;
        for (int i = 0; i < (int)observed_gap.size(); i++)
        {
            if(observed_gap[i].is_left_type() && observed_gap[i].is_radial())
            {
                merged_gaps.push_back(observed_gap[i]);
            }
            else
                // Merge
            {
                int right_idx = observed_gap[i].get_r_idx();
                double right_dist = observed_gap[i].get_r_dist();
                int last_mergable = -1;
                for(int j = (int) merged_gaps.size()-1; j>=0;j--)
                {
                    bool merge_distance_cond{false};
                    bool merge_angle_cond{false};
                    int left_idx = merged_gaps[j].get_l_idx();
                    double left_dist = merged_gaps[j].get_l_dist();

                    Gap new_gap(observed_gap[i].get_frame(), left_idx, left_dist, scan.ranges.size());
                    new_gap.add_right_information(right_idx, right_dist);

                    /**Check if any point inside newly merged gap has distance smaller than distance of 2 edge point
                     * **/
                    if(left_idx > right_idx) // Handle edge case where new gap is wrapped around
                    {
                        double min;
                        if(left_idx == scan.ranges.size() - 1 && right_idx > 0) //If left_idx is the last point and right_idx is not the first point
                        {
                            min = *std::min_element(scan.ranges.begin(), scan.ranges.begin() + right_idx);
                            merge_distance_cond = (min >= right_dist) && (min >= left_dist);
                        }
                        else if (left_idx < scan.ranges.size() - 1 && right_idx == 0)//If left_idx is not the last point and right_idx is the first point
                        {
                            min = *std::min_element(scan.ranges.begin() + left_idx + 1, scan.ranges.end());
                            merge_distance_cond = (min >= right_dist) && (min >= left_dist);
                        }
                        else if (left_idx < scan.ranges.size() - 1 && right_idx > 0)
                        {
                            auto min1 = std::min_element(scan.ranges.begin(), scan.ranges.begin() + right_idx) ; 
                            auto min2 = std::min_element(scan.ranges.begin() + left_idx + 1, scan.ranges.end());
                            min = std::min(*min1, *min2);
                            merge_distance_cond = (min >= right_dist) && (min >= left_dist);
                        }
                        else //If left_idx is the last element and right_idx is first element, thus they are consecutive, then 
                        {
                            merge_distance_cond = true;
                        }
                    }
                    else //in normal case
                    {
                        if(right_idx - left_idx <= 1) //If 2 end of gap are consecutive point, there will be no point in between, mergable
                            merge_distance_cond = true;
                        else
                        {
                            double min = *std::min_element(scan.ranges.begin() + left_idx + 1, scan.ranges.begin() + right_idx);
                            merge_distance_cond =  (min >= right_dist) && (min >= left_dist);
                        }
                    }
                    /*Check for merge angle condition*/
                    if (new_gap.calculate_gap_arc_angle() <= mk_merged_gap_angle_threshold)
                    {
                        merge_angle_cond = true;
                    }

                    // std::cout << j << " " << i << " merge distance cond: "<< merge_distance_cond << "  merge angle cond: " << merge_angle_cond << " angle "<< new_gap.calculate_gap_arc_angle()*180/M_PI << std::endl;
                    if( merge_angle_cond && merge_distance_cond)
                    {
                        last_mergable = j;  
                    }
                }
                if (last_mergable!=-1)
                {
                    auto left_idx = merged_gaps[last_mergable].get_l_idx();
                    auto left_dist = merged_gaps[last_mergable].get_l_dist();
                    Gap new_gap(observed_gap[i].get_frame(), left_idx, left_dist, scan.ranges.size());
                    new_gap.add_right_information(right_idx, right_dist);
                    merged_gaps.erase(merged_gaps.begin() + last_mergable, merged_gaps.end());
                    if (new_gap.get_gap_length() >= mk_gap_length_threshold)
                        merged_gaps.push_back(new_gap);
                }
                else
                {
                    merged_gaps.push_back(observed_gap[i]);
                }

            }
        }
        observed_gap = std::move(merged_gaps);
    }

    visualization_msgs::msg::MarkerArray visualize_gaps(const sensor_msgs::msg::LaserScan &scan,
                                                        const std::vector<Gap> &observed_gap,
                                                        int marker_id_offset = 0)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        int id = marker_id_offset; // Start IDs from the offset

        double max_confident{0};
        for (const auto &gap : observed_gap)
        {
            if (gap.get_confident() > max_confident) {max_confident = gap.get_confident();}
        }

        for (const auto &gap : observed_gap)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = scan.header.frame_id;
            marker.header.stamp = scan.header.stamp;
            marker.ns = "gaps";
            marker.id = id++; // Increment ID for each marker
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.lifetime = rclcpp::Duration::from_seconds(0.1);

            // Set color for the marker (e.g., blue for merged gaps)
            if (gap.get_confident() == max_confident)
            {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }
            else
            {
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
            }
            
            marker.color.a = 1.0; // Fully opaque
            marker.scale.x = 0.02; // Line width

            // Compute Cartesian coordinates for the left and right points of the gap
            geometry_msgs::msg::Point p_left, p_right;
            // double angle_left = scan.angle_min + gap.get_l_idx() * scan.angle_increment;
            // double angle_right = scan.angle_min + gap.get_r_idx() * scan.angle_increment;

            auto left_point = gap.get_l_cartesian();
            auto right_point = gap.get_r_cartesian();
            // p_left.x = gap.get_l_dist() * cos(angle_left);
            // p_left.y = gap.get_l_dist() * sin(angle_left);

            p_left.x = left_point.first;
            p_left.y = left_point.second;
            p_left.z = 0.0; // Assuming a 2D scan

            p_right.x = right_point.first;
            p_right.y = right_point.second;
            p_right.z = 0.0; // Assuming a 2D scan

            // Add points to the marker
            marker.points.push_back(p_left);
            marker.points.push_back(p_right);

            // Add the marker to the marker array
            marker_array.markers.push_back(marker);
        }

        return marker_array;
    }
};
#endif // FIND_GAP_HPP_