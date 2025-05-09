#include "wheelchair_control_support/RectangularFootprint.hpp"
#include <cassert>
int main()
{
    std::unique_ptr<RectangularFootprint> footprint = std::make_unique<RectangularFootprint>(0, 0, 0, 0.6, 1.0);
    // std::cout << footprint->square_distance_from_point_to_footprint(2.0, 0.6) << std::endl;
    // std::cout << footprint->square_distance_from_point_to_footprint(1.0, 0.6) << std::endl;
    // assert(footprint->square_distance_from_point_to_footprint(2.0, 0.0) -1.0 < 1e-6);
    // assert(footprint->square_distance_from_point_to_footprint(0.6, 1.0) < 0);
    assert(footprint->check_point_inside_footprint(0.0, 0.0) == true);
    assert(footprint->check_point_inside_footprint(0.5, 0.3) == true);
    assert(footprint->check_point_inside_footprint(-0.5, -0.3) == true);
    assert(footprint->check_point_inside_footprint(0.6, 0.3) == false);
    assert(footprint->check_point_inside_footprint(-0.6, -0.3) == false);
    assert(footprint->check_point_inside_footprint(0.0, 0.31) == false);

    footprint->move_footprint(1.0, 1.0, 0.0);
    assert(footprint->check_point_inside_footprint(1.0, 1.0) == true);
    assert(footprint->check_point_inside_footprint(1.5, 1.3) == true);
    assert(footprint->check_point_inside_footprint(1.6, 1.3) == false);
    assert(footprint->check_point_inside_footprint(1.0, 1.31) == false);
    assert(footprint->check_point_inside_footprint(1.0, 0.0) == false);
    assert(footprint->check_point_inside_footprint(0.0, 1.0) == false);

    footprint->move_footprint(1.0, 1.0, M_PI/2);
    assert(footprint->check_point_inside_footprint(1.0, 1.0) == true);
    assert(footprint->check_point_inside_footprint(1.3, 1.5) == true);
    assert(footprint->check_point_inside_footprint(1.3, 1.6) == false);
    assert(footprint->check_point_inside_footprint(0.7, 0.5) == true);
    assert(footprint->check_point_inside_footprint(1.3, 0.5) == true);

    footprint->move_footprint(0.0,0.0,0.0);
    std::cout << footprint->square_distance_from_point_to_footprint(1.0, 0.0) << std::endl;
    assert(abs(footprint->square_distance_from_point_to_footprint(1.0, 0.0) -0.25)< 1e-6);
    std::cout << footprint->square_distance_from_point_to_footprint(1.0, 0.8) << std::endl;
    assert(abs(footprint->square_distance_from_point_to_footprint(1.0, 0.8) -2*0.25) < 1e-6);
    std::cout << footprint->square_distance_from_point_to_footprint(0.0, 1.0) << std::endl;
    assert(abs(footprint->square_distance_from_point_to_footprint(0.0, 1.0) -0.7*0.7)< 1e-6);

    footprint->move_footprint(1.0, 2.0, M_PI/2);
    std::cout << footprint->square_distance_from_point_to_footprint(0.0, 0.0) << std::endl;
    assert(abs(footprint->square_distance_from_point_to_footprint(0.0, 0.0) -(0.7*0.7+1.5*1.5))< 1e-6);
    std::cout << footprint->square_distance_from_point_to_footprint(1.0, 2.0) << std::endl;
    assert(abs(footprint->square_distance_from_point_to_footprint(1.0, 2.0) + 1)< 1e-6);
    std::cout << footprint->square_distance_from_point_to_footprint(1.0, 0.0) << std::endl;
    assert(abs(footprint->square_distance_from_point_to_footprint(1.0, 0.0) - 1.5*1.5)< 1e-6);

    return 0;
}