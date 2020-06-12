//
//  cabby_curve_collection.hpp
//  ack
//
//  Created by Shane Scott on 5/29/20.
//

#ifndef cabby_curve_collection_hpp
#define cabby_curve_collection_hpp

#include <list>

#include "curve_collection.h"

using vedge = std::array<Eigen::Vector2d,2>;
using hedge = std::array<Eigen::Vector2d,2>;


class CabbieCurveCollection : public CurveCollection {
public:

    //Returns true if there is an impact, otherwise false.
    //If true, impact_location is populated with the impact point on the line segment.
};

bool rayTraceNorth(const Eigen::Vector2d &origin, const std::array<Eigen::Vector2d,2> &segment,  Eigen::Vector2d &impact);

bool rayTraceEast(const Eigen::Vector2d &origin, const std::array<Eigen::Vector2d,2> &segment, Eigen::Vector2d &impact);

struct Cabbie{
    //Cabby paths should move only alternatingly east ~(1,0) and north ~(0,1)
    int num_walls_;
    int num_plateaus_;
    std::vector<Eigen::Vector2d> points_ = {};
    //
    void copyPointList(const std::list<Eigen::Vector2d> &points);
    void push_back(const Eigen::Vector2d &x);
    void resize(const std::vector<Eigen::Vector2d>::size_type &x);
    vedge getWall(const int &index) const;
    hedge getPlateau(const int &index) const;
};


#endif /* cabby_curve_collection_hpp */
