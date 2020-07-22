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

using Vedge = std::array<Eigen::Vector2d, 2>; // vertical edge
using Hedge = std::array<Eigen::Vector2d, 2>; // horizontal edge
using Cedge = std::array<Eigen::Vector2d, 2>; // cardinal edge


enum class OutsideSide {
	kUnknown,
	kNegative,
	kPositive
};


class CardinalCurveCollection : public CurveCollection {
public:
	std::vector<bool> is_horizontals_;
	std::vector<OutsideSide> outsidesides_;
	void removeTangentRectangle(const Eigen::Vector2d &lower_left, const double &width, const double &height);
    //Returns true if there is an impact, otherwise false.
    //If true, impact_location is populated with the impact point on the line segment.
    void mergeParallelEdges();
	void populateSlopes();
	bool rayTraceEast(const Eigen::Vector2d &origin, Eigen::Vector2d &impact, int& impact_edge_id);
	bool rayTraceNorth(const Eigen::Vector2d &origin, Eigen::Vector2d &impact, int& impact_edge_id);
	void generateFromDifferenceSequence(std::vector<double> ss, double scale = 1);
};

bool rayNorthSegmentIntersect(const Eigen::Vector2d &origin, const Hedge &segment,  Eigen::Vector2d &impact);

bool rayEastSegmentIntersect(const Eigen::Vector2d &origin, const Vedge &segment, Eigen::Vector2d &impact);

struct CardinalPath{
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
