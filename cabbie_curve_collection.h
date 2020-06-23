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

using Vedge = std::array<Eigen::Vector2d,2>;
using Hedge = std::array<Eigen::Vector2d,2>;
using PointList = std::vector<Eigen::Vector2d>;


class CabbieCurveCollection : public CurveCollection {
public:

    //Returns true if there is an impact, otherwise false.
    //If true, impact_location is populated with the impact point on the line segment.
};

bool rayTraceNorth(const Eigen::Vector2d &origin, const std::array<Eigen::Vector2d,2> &segment,  Eigen::Vector2d &impact);

bool rayTraceEast(const Eigen::Vector2d &origin, const std::array<Eigen::Vector2d,2> &segment, Eigen::Vector2d &impact);

struct CabbiePath{
    //Cabby paths should move only alternatingly east ~(1,0) and north ~(0,1)
    int num_walls_ = 0;
    int num_plateaus_ = 0;
    std::vector<Eigen::Vector2d> points_ = {};
    //
    void copyPointList(const std::list<Eigen::Vector2d> &points);
    void push_back(const Eigen::Vector2d &x);
    void resize(const std::vector<Eigen::Vector2d>::size_type &x);
    Vedge getWall(const int &index) const;
    Hedge getPlateau(const int &index) const;
};

std::vector<PointList> addRectangleModZ2(std::vector<PointList> &chains, const Eigen::Vector2d &position, const double &width, const double &height);

enum class SegmentRelation {
	kDisjoint,
	kSubsegment, // a contained by b
	kSupersegment, // a contains b
	kDirectStagger, // abab
	kReverseStagger // baba
};

std::string _debug_seg_rel_print(SegmentRelation x);

std::string _debug_edge_print(std::array<Eigen::Vector2d,2> x);

SegmentRelation edgeIntersection(const std::array<Eigen::Vector2d, 2>& a, const std::array<Eigen::Vector2d, 2>& b, const bool is_horizontal, std::array<Eigen::Vector2d, 2>& isect);



#endif /* cabby_curve_collection_hpp */
