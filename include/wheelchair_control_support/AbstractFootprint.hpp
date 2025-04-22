#ifndef ABSTRACT_FOOTPRINT_
#define ABSTRACT_FOOTPRINT_
#include <visualization_msgs/msg/marker.hpp>

class AbstractFootprint 
{
public: 
    AbstractFootprint() = default;
    virtual ~AbstractFootprint() = default;
    /**
     * Move the origin of the footprint from (0,0, 0) to (x,y, theta) position
     * **/
    virtual void move_footprint(const double x, const double y, const double theta)=0;

    /**
     * Check if a point at (x,y) is inside the footprint
     * **/
    virtual bool check_point_inside_footprint(const double x, const double y)=0;

    virtual double distance_from_point_to_footprint(const double x, const double y)=0;

    /**
     * Return visualization marker for visualizing footprint with rviz
     * **/
    virtual visualization_msgs::msg::Marker generate_footprint_marker(std::string frame_id, int marker_id)=0;
};
#endif