//
//  cabby_curve_collection.hpp
//  ack
//
//  Created by Shane Scott on 5/29/20.
//

#ifndef curve_collection_hpp4774101
#define curve_collection_hpp4774101

#include <array>
#include <iostream>
#include <vector>

#include "Eigen/Dense"

static const double repsilon = std::sqrt(std::numeric_limits<double>::epsilon());

class CurveCollection {
protected:
    bool is_valid_ = true;
    //Input must represent a collection of disjoint simple closed curves in the plane.
    std::vector<Eigen::Vector2d> points_{};
    std::vector<std::array<int, 2>> edges_{};

    // Next point in the traversal
    std::vector<int> point_order_next_{};
    // Previous point in the traversal
    std::vector<int> point_order_prev_{};
    // Edge from point lookup should be ordered [prev_edge, next_edge]
    std::vector<std::array<int,2>> edges_of_points_;
    // Next edge in the traversal
    std::vector<int> edge_order_next_{};
    // Previous edge in the traversal
    std::vector<int> edge_order_prev_{};
    // One basepoint for each connected component
    std::vector<int> basepoints_{};
    
    // Tangent vectors by discrete difference
    std::vector<Eigen::Vector2d> tangents_{};
    void compute_tangents();
    // Normal vectors by orthogonality to tangents
    std::vector<Eigen::Vector2d> normals_{};
    bool is_normals_computed_ = false;
    void compute_normals();
    // Compute a traversal
    bool orient_curves();
    std::array<int, 2> neighborhood_orientation(int pointid);
    

public:
    CurveCollection(){};
    // Poly data must have lines forming closed curves.
//    CabbyCurveCollection(const std::vector<std::array<double,2>> &grid_points, const std::vector<std::vector<int>> &one_cells);
//    CabbyCurveCollection(const std::vector<std::array<double,3>> &grid_points, const std::vector<std::vector<int>> &one_cells);

    bool insertEdges(const std::vector<std::array<double,2>> &grid_points, const std::vector<std::vector<int>> &one_cells);
    
    bool insertEdges(const std::vector<std::array<double,3>> &grid_points, const std::vector<std::vector<int>> &one_cells);

    
    // Ray to segment intersection computation for ray trace
    Eigen::Vector2d ray_segment_intersect(const Eigen::Vector2d orig, const Eigen::Vector2d dir, const Eigen::Vector2d a, const Eigen::Vector2d b);
    bool is_ray_segment_intersect(const Eigen::Vector2d orig, const Eigen::Vector2d dir, const Eigen::Vector2d a, const Eigen::Vector2d b);
    double distance_till_impact(const int pointid, const Eigen::Vector2d dir);


    // Compute if a point lies in the interior of the curve collection
    bool is_point_in(const Eigen::Vector2d point);

//        // Write to file for debugging.
//        void write_to_vtp(const std::string filename);

    Eigen::Vector2d get_normal(int p_id);
    
    // Point access
    int get_number_of_points() const;
    
    int get_number_of_components() const;
    int get_basepoint(int) const;
    
    // Cycle information access
    int get_prev_point(const int e_id) const;
    int get_next_point(const int e_id) const;

    
    // Cycle information access
    int get_prev_edge(const int e_id) const;
    int get_next_edge(const int e_id) const;

    Eigen::Vector2d get_point(const int point_id) const;

    std::vector<int> get_edges_of_point(const int point_id) const;
    
    std::array<int, 2> get_points_of_edge(const int e_id) const;

};

#endif /* cabby_curve_collection_hpp */
