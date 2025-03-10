#ifndef GAP_
#define GAP_

#include <cmath>
#include <string>
#include <utility>

/***
 * A gap is defined by 2 points in the scan message
 * - Left point is point in the gap that is detected FIRST
 * - Left type gap has distance to left point smaller than distance to right point. The inverse is right type gap
 * - radial gap is gap that is approximately parralel to the direction of the ray direction of the range sensor
 * 
 * - Gap is constructed with left information first, then right information is added later, concluding the finding of one gap
 * Implementation reference: https://github.com/ivaROS/PotentialGap/
 */

class Gap
{
private:
    int m_left_idx;
    int m_right_idx;
    double m_left_distance;
    double m_right_distance;
    std::string m_frame; //frame of the gap (for Rviz2 visualization)
    bool m_is_radial;
    bool m_is_left_type;
    int m_total_scan_point;
    double m_min_scan_angle{-M_PI}; // Minimum angle of the scan message. This should depends on the scan message received but I just defined a constant for now
    double m_confident{0}; //Probability of gap being the wanted gap
    
    static constexpr double mk_radial_gap_threshold{M_PI/2}; // Threshold to consider a gap is radial or not

public:
    Gap(std::string frame, int left_idx, double left_distance, int total_scan_point, double min_scan_angle = -M_PI)
        :   m_frame{frame}, m_left_idx{left_idx}, m_left_distance{left_distance}, m_total_scan_point{total_scan_point}, m_min_scan_angle{min_scan_angle}
    {}



    ~Gap(){}

    void set_l_idx(int left_idx) {m_left_idx = left_idx;}
    void set_l_dist(double left_distance) {m_left_distance = left_distance;}
    void set_r_idx(int right_idx) {m_right_idx = right_idx;}
    void set_r_dist(double right_distance) {m_right_distance = right_distance;}
    void set_frame(std::string frame) {m_frame = frame;}
    void set_confident(double confident) {m_confident = confident;}
    
    int    get_l_idx() const {return m_left_idx;}
    double get_l_dist() const {return m_left_distance;}
    int    get_r_idx()  const {return m_right_idx;}
    double get_r_dist() const {return m_right_distance;}
    std::string get_frame() const {return m_frame;}
    double get_confident() const {return m_confident;}

    /**
     * Get the cartesian coordinate (x,y) of the right end point
     */
    std::pair<double, double>  get_r_cartesian() const {
        double resolution = 2*M_PI / m_total_scan_point;
        return {m_right_distance*cos(resolution*m_right_idx + m_min_scan_angle), m_right_distance*sin(resolution*m_right_idx + m_min_scan_angle)};
    }

    /**
     * Get the cartesian coordinate (x,y) of the left end point
     */
    std::pair<double, double>  get_l_cartesian() const {
        double resolution = 2*M_PI / m_total_scan_point;
        return {m_left_distance*cos(resolution*m_left_idx + m_min_scan_angle), m_left_distance*sin(resolution*m_left_idx + m_min_scan_angle)};
    }


    bool is_radial() const {return m_is_radial;}
    bool is_left_type() const { return m_is_left_type;}

    /**
     * Return the smaller angle between 2 points forming a gap
     */
    double calculate_gap_arc_angle() 
    {
        double resolution = 2*M_PI / m_total_scan_point;
        if(m_right_idx >= m_left_idx)
        {
            return (m_right_idx - m_left_idx) * resolution;
        }
        else
        {
            return (m_total_scan_point - m_left_idx + m_right_idx) * resolution;
        }
    }

    /*Conclude the gap after constructing with left information*/
    void add_right_information(int right_idx, double right_distance)
    {
        m_right_idx = right_idx;
        m_right_distance = right_distance;
        m_is_left_type = m_right_distance > m_left_distance;

        //Check if gap is radial or not
        float angle1 = calculate_gap_arc_angle();
        float short_side = m_is_left_type? m_left_distance : m_right_distance;
        float opp_side = (float) sqrt(pow(m_left_distance, 2) + pow(m_right_distance, 2) - 2 * m_left_distance * m_right_distance * (float)cos(angle1));
        float small_angle = (float) asin(short_side / opp_side * (float) sin(angle1));
        m_is_radial = (M_PI - small_angle - angle1 >  mk_radial_gap_threshold);
    }

    //Calculate the lenght based on law of cosine
    double get_gap_length()
    {
        return sqrt(pow(m_left_distance, 2) + pow(m_right_distance, 2) - 2 * m_left_distance * m_right_distance * (cos(calculate_gap_arc_angle())));
    }

    double calculate_distance_to_robot() {
        // Get the arc angle (θ) between the left and right distances
        double angle = calculate_gap_arc_angle();

        // Calculate the base using the law of cosines
        double base = get_gap_length();

        // Calculate the area of the triangle
        double area = 0.5 * m_left_distance * m_right_distance * sin(angle);

        // Calculate and return the height
        return (base != 0) ? (2 * area / base) : 0.0; // Avoid division by zero
    }
};
#endif