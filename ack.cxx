#include <iostream>
#include <limits>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "caliper.h"

/*
 A vacancy is a polygon (or possible a non-simply connected region with polygon boundaries) that has only vertical and horizontal edges.
 
 Setting up the vacancy:
    From polygons:
    Compute the boundary connected components and compute an orientation
    Join any colinear segments into larger segments.
 
 Divide the vacancy into patrols:
    Patrol shows the top and bottom points
 
 
 For each rectangle:
     Given a rectangle size width by height:
     
     Generate a caliper with the width:
        Compute all caliper positions across the patrol top and bottom
        Compute the heights available at all positions
        Get a list of all rectangle spots available
     
     Pick the lowest, then leftest one.
     
     Go back and update the patrols.
 */


namespace exda{
    std::vector<std::array<double,3>> grid_points = {
        {0, 0, 0},
        {5, 0, 0},
        {5, 1, 0},
        {6, 1, 0},
        {6, 2, 0},
        {5, 2, 0},
        {5, 3, 0},
        {1, 3, 0},
        {1, 2, 0},
        {2, 2, 0},
        {2, 1, 0},
        {0, 1, 0},
        {3, 1, 0},
        {4, 1, 0},
        {4, 2, 0},
        {3, 2, 0},
    };

    std::vector<std::vector<int>> lines = {
        {0,1},
        {1,2},
        {2,3},
        {3,4},
        {4,5},
        {5,6},
        {6,7},
        {7,8},
        {8,9},
        {9,10},
        {10,11},
        {11,0},
        {12,13},
        {13,14},
        {14,15},
        {15,12}
    };
}

void example1(){
    CabbyCurveCollection ccc;
    ccc.insertEdges(exda::grid_points, exda::lines);
    std::cout << ccc.get_number_of_points();
    for (int foo=0; foo<exda::lines.size(); foo++){
        std::cout << "e " << foo << " prev " << ccc.get_prev_edge(foo) << " next " << ccc.get_next_edge(foo) << std::endl;
    }
    std::cout << "true is " << true << std::endl;
    std::cout << "false is " << false << std::endl;
    std::cout << "(3.5,1.5) is out " << ccc.is_point_in({3.5,1.5}) << std::endl;
    std::cout << "(2.5,1.5) is in " << ccc.is_point_in({2.5,1.5}) << std::endl;
    std::cout << "(1.5,1.5) is out " << ccc.is_point_in({1.5,1.5}) << std::endl;

    // ccc should have a cyclic ordering of the points
    // if we have a point to edge look up we can manage a traversal
};

void example2(){
    Vacancy vac;
    vac.insertCurves(exda::grid_points, exda::lines);
    vac.populateNeighbors();
    vac.findFrontier();
    std::cout << "frontier west" << vac.westmost_frontier_id_ << std::endl;
    std::cout << "frontire east" << vac.eastmost_frontier_id_ << std::endl;
    
    for (int foo=0; foo<vac.vsides_.size(); foo++){
        std::cout << foo << std::endl;
        VacancySide vs = vac.vsides_[foo];
        std::cout << " pt " << vs.point_ids[0] << " @ " << vs.points[0].transpose() << std::endl;
        std::cout << " pt " << vs.point_ids[1] << " @ " << vs.points[1].transpose() << std::endl;
        std::cout << "length " << vs.length << " ";
        if (vs.orientation == EdgeOrientation::kEast) std::cout << "horiz";
        if (vs.orientation == EdgeOrientation::kNorth) std::cout << "verti" ;
        if (vs.neighbor_shape == NeighborhoodShape::kZag)            std::cout << "zag" << std::endl;
        if (vs.neighbor_shape == NeighborhoodShape::kNotchNorth)            std::cout << "north" << std::endl;
        if (vs.neighbor_shape == NeighborhoodShape::kNotchEast)            std::cout << "east" << std::endl;
        if (vs.neighbor_shape == NeighborhoodShape::kNotchSouth)            std::cout << "south" << std::endl;
        if (vs.neighbor_shape == NeighborhoodShape::kNotchWest)            std::cout << "west" << std::endl;
    }
    
    std::cout << "Patrol down from 11 " << std::endl;
    Trail patty11 = vac.spawnPatrol(11);
    Trail patty7 = vac.spawnPatrol(7);
    
};

void example3(){
    Trail t;
    t.landmarks_valley = {
        Eigen::Vector2d({0,0}),
        Eigen::Vector2d({2,0}),
        Eigen::Vector2d({2,-1}),
        Eigen::Vector2d({5,-1}),
        Eigen::Vector2d({5,3}),
        Eigen::Vector2d({6,3}),
        Eigen::Vector2d({6,1}),
        Eigen::Vector2d({7,1}),
        Eigen::Vector2d({7,0}),
        Eigen::Vector2d({8,0}),
        Eigen::Vector2d({8,1})
    };
    std::cout << "Hike!" << std::endl;
    CaliperHiker calh(3.1, t);
    calh.hikeTrail();
    //
//    std::cout << "New hike!" << std::endl;

//    CaliperHiker calj(0.9, t);
//    calj.hikeTrail();
    // Answer:
    /*
     camp at -0.9    0
     camp at 2 0
     camp at  2 -1
     camp at 4.1  -1
     camp at 4.1   3
     camp at 6 3
     camp at 6 1
     camp at 7 1
     camp at 7 0
     camp at 7.1   0
     camp at 7.1   1
     */
};

int main() {
    std::cout << "Saluton Mundo!" << std::endl;
    example3();
}
