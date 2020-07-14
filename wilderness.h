#ifndef wilderness_h
#define wilderness_h

#include <array>
#include <list>

#include "caliper.h"

enum class EdgeOrientation{
    kBad,
    kNorth,
    kEast
};

enum class OutsideSide{
    kUnknown,
    kNegative,
    kPositive
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

//struct VacancySide{
//    //This struct is pretty silly. Refactor it away.
//    int edge_id;
//    double length = 0.0;
//    EdgeOrientation orientation = EdgeOrientation::kBad;
//    OutsideSide outside = OutsideSide::kUnknown;
//    NeighborhoodShape neighbor_shape = NeighborhoodShape::kUnknown;
//    std::array<int,2> point_ids = {0,0};
//    //SS: question for future self: do you just want to represent as pt * interval?
//    std::array<Eigen::Vector2d,2> points = {Eigen::Vector2d({0,0}),Eigen::Vector2d({0,0})};
//    //
//    VacancySide(){};
//    VacancySide(const int edge_id_0, const std::array<Eigen::Vector2d,2> &i_points, const std::array<int,2> &i_point_ids);
//};

struct ZoningCommisioner{
    CardinalCurveCollection vacant_;
	std::vector<NeighborhoodShape> shapes_;
	std::vector<bool> is_edge_horizontals_;
	std::vector<OutsideSide> outsidesides_;
    //std::vector<VacancySide> vsides_;
    int westmost_frontier_id_ = 0;
    int eastmost_frontier_id_ = 0;
    std::vector<Trail> trails_;
    
    ZoningCommisioner(){};
    
	void insertCurves(const std::vector<std::array<double, 3>> &grid_points, const std::vector<std::vector<int>> &lines);

	Cedge getCedge(const int id, bool& is_horizontal) const;
	Cedge getCedge(const int id, bool& is_horizontal, std::array<int,2> &point_ids) const;

    void findFrontier();
    
    NeighborhoodShape getNeighborhoodShape(int dim_of_interest, double pos_prev, double pos_foo, double pos_next);
    
	void populateSlopes();
    void populateNeighbors();
    
    void nextCollideNorth(const Eigen::Vector2d &origin, const bool &is_following_orientation, int &point_at, int &edge_at, Eigen::Vector2d &impact);
    
    Trail trailblaze(int start_edge_id);
    
    void trailblaze();
    
    bool findPlacement(const double& width, const double& height, Eigen::Vector2d& placement) const;

    void zoneOff(const Eigen::Vector2d& position, const double& width, const double& height);
    
};


#endif /* wilderness_h */
