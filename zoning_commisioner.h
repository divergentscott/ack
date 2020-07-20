#ifndef wilderness_h
#define wilderness_h

#include <array>
#include <list>

#include "zone_surveyor.h"

/*
A vacancy is a polygon (or possible a non-simply connected region with polygon boundaries) that has only vertical and horizontal edges.
 
 Setting up the vacancy:
    From polygons:
    Compute the boundary connected components and compute an orientation
    Join any colinear segments into larger segments.
 
 Divide the vacancy into zones:
    Zone shows the top and bottom points
 
 For each rectangle:
     Given a rectangle size width by height:
     
     Generate a Surveyor with the width:
        Compute all sufficiently wide positions across the patrol top and bottom
        Compute the heights available at all positions
        Get a list of all rectangle spots available
     
     Pick the lowest, then leftest one.
     
     Go back and update the zones.
 */

enum class NeighborhoodShape{
    kUnknown,
    kZag, // looks like _|-
    //Nothches are differentiated by their opening direction
    kNotchNorth, // looks like |_|
    kNotchEast,
    kNotchSouth,
    kNotchWest
};

struct ZoningCommisioner{
    CardinalCurveCollection vacant_;
	std::vector<NeighborhoodShape> shapes_;
    int westmost_frontier_id_ = 0;
    int eastmost_frontier_id_ = 0;
    std::vector<Zone> trails_;
    
    ZoningCommisioner(){};
    
	void insertCurves(const std::vector<Eigen::Vector2d> &grid_points, const std::vector<std::vector<int>> &lines);

	Cedge getCedge(const int id, bool& is_horizontal) const;
	Cedge getCedge(const int id, bool& is_horizontal, std::array<int,2> &point_ids) const;

    void findFrontier();
    
    NeighborhoodShape getNeighborhoodShape(int dim_of_interest, double pos_prev, double pos_foo, double pos_next);
    
    void populateNeighbors();
        
    Zone trailblaze(int start_edge_id);
    
    void trailblaze();
    
    bool findPlacement(const double& width, const double& height, Eigen::Vector2d& placement) const;

    void zoneOff(const Eigen::Vector2d& position, const double& width, const double& height);
    
};


#endif /* wilderness_h */
