//
//  vacancy.h
//  ack
//
//  Created by Shane Scott on 6/9/20.
//

#ifndef vacancy_h
#define vacancy_h

#include <array>
#include <list>

#include "cabby_curve_collection.h"
#include "patrol.h"

enum class EdgeOrientation{
    kBad,
    kNorth,
    kEast
};

enum class OutsideSide{
    kUnknown,
    kLeft,
    kRight
};

enum class NeighborhoodShape{
    kUnknown,
    kZag, // looks like _|-
    //Nothches are differentiated by their opening direction
    kNotchNorth, // looks like |_|
    kNotchEast,
    kNotchSouth,
    kNotchWest
};

struct VacancySide{
    //This struct is pretty silly. Refactor it away.
    int edge_id;
    double length = 0.0;
    EdgeOrientation orientation = EdgeOrientation::kBad;
    OutsideSide outside = OutsideSide::kUnknown;
    NeighborhoodShape neighbor_shape = NeighborhoodShape::kUnknown;
    std::array<int,2> point_ids = {0,0};
    //SS: question for future self: do you just want to represent as pt * interval?
    std::array<Eigen::Vector2d,2> points = {Eigen::Vector2d({0,0}),Eigen::Vector2d({0,0})};
    //
    VacancySide(){};
    VacancySide(const int edge_id_0, const std::array<Eigen::Vector2d,2> &i_points, const std::array<int,2> &i_point_ids);
};

struct Trail{
    //Trails cover the Vacancy.
    //Trails have a path of lower and upper points in R2
    //Trails do not contain east or west notches.
    Cabbie landmarks_mountain_;
    Cabbie landmarks_valley_;
    VacancySide start;
    VacancySide terminus;
};

struct Vacancy{
    CabbieCurveCollection curves;
    std::vector<VacancySide> vsides_;
    int westmost_frontier_id_ = 0;
    int eastmost_frontier_id_ = 0;
    
    Vacancy(){};
    
    void findFrontier();
    
    NeighborhoodShape getNeighborhoodShape(int dim_of_interest, double pos_prev, double pos_foo, double pos_next);
    
    void insertCurves(const std::vector<std::array<double,3>> &grid_points, const std::vector<std::vector<int>> &lines);
    
    void populateNeighbors();
    
    void nextCollideNorth(const Eigen::Vector2d &origin, const bool &is_following_orientation, int &point_at, int &edge_at, Eigen::Vector2d &impact);
    
    Trail spawnPatrol(int start_edge_id);
};


#endif /* vacancy_h */
