#include <array>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "cabby_curve_collection.h"

static const double repsilon = std::sqrt(std::numeric_limits<double>::epsilon());



/* fix this interiority check
bool is_point_in(Eigen::Vector2d point) {
    Determine if a point lies in a curve collecion via parity of a random
     * direction ray trace.
    Eigen::Vector2d direct{cos(theta), sin(theta)};
    int isect_count = 0;
    for (int foo = 0; foo < edges_.size(); foo++) {
        point::r2 point_foo = points_[edges_[foo][0]];
        point::r2 point_next = points_[edges_[foo][1]];
        std::array<double, 2> isect_params =
            ray_segment_intersect(point, direct, point_foo, point_next);
        // Failure!
        if ((isect_params[1] == 0.0) & (isect_params[0] > 0.0)) {
            // If an edge is a subset of a ray, the pairity test will fail.
            // The only hope to resolve is to check the direction of nonparallel
            // edges o This is slightly dangerous! Technically could be an
            // infinite loop... If the segment fully lies inside the ray, we
            // reroll the ray direction and restart. std::cout << "Segment is
            // subset of ray! Reroll";
            theta = random_angle();
            direct = {cos(theta), sin(theta)};
            isect_count = 0;
            foo = -1;
            continue;
        }
        // But normally the ray hits the segment interior.
        if ((-repsilon < isect_params[1]) &
            (isect_params[1] < 1 + point::epsilon)) {
            if (std::abs(isect_params[0]) <= point::epsilon) {
                return true;
            } else if (isect_params[0] > 0) {
                isect_count++;
            }
        }
    }
    return (isect_count % 2) == 1;
}

 double CurveCollection::distance_till_impact(const int pointid,
                                              const point::r2 dir) {
     Compute the distance on a line at orig with direction +/- dir travels
      * before hitting an edge not containing pointid
     double dist = std::numeric_limits<double>::infinity();
     point::r2 at = points_[pointid];
     for (auto edge : edges_) {
         if ((pointid != edge[0]) & (pointid != edge[1])) {
             point::r2 impact = ray_segment_intersect(at, dir, points_[edge[0]],
                                                      points_[edge[1]]);
             // impact[0] is the length along the ray, impact[1] is the length
             // along the line containing edge
             if ((impact[1] < 1 + point::epsilon) &
                 (impact[1] > -point::epsilon)) {
                 dist = std::min(std::abs(impact[0]), dist);
             }
         }
     }
     return dist;
 }
 */


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
    double length = 0.0;
    EdgeOrientation orientation = EdgeOrientation::kBad;
    OutsideSide outside = OutsideSide::kUnknown;
    NeighborhoodShape neighbor_shape = NeighborhoodShape::kUnknown;
    std::array<int,2> point_ids = {0,0};
    //SS: question for future self: do you just want to represent as pt * interval?
    std::array<Eigen::Vector2d,2> points = {Eigen::Vector2d({0,0}),Eigen::Vector2d({0,0})};
    
    VacancySide(){};
    VacancySide(const std::array<Eigen::Vector2d,2> &i_points, const std::array<int,2> &i_point_ids){
        //Vacancy side constructor from vertex ids and positions
        double xlen = std::abs((i_points[0][0] - i_points[1][0]));
        double ylen = std::abs((i_points[0][1] - i_points[1][1]));
        if ( (xlen < repsilon) & (ylen > repsilon) ){
            //Decide that the segment is vertical
            orientation = EdgeOrientation::kNorth;
            length = ylen;
            if (i_points[0][1] <= i_points[1][1]){
                point_ids = i_point_ids;
                points = i_points;
            } else {
                point_ids = {i_point_ids[1], i_point_ids[0]};
                points = {i_points[1], i_points[0]};
            }
        }
        if ( (xlen > repsilon) & (ylen < repsilon) ){
            //Decide that the segment is horizontal
            orientation = EdgeOrientation::kEast;
            length = xlen;
            if (i_points[0][0] <= i_points[1][0]){
                point_ids = i_point_ids;
                points = i_points;
            } else {
                point_ids = {i_point_ids[1], i_point_ids[0]};
                points = {i_points[1], i_points[0]};
            }

        }
    };
};

struct Vacancy{
    CabbyCurveCollection curves;
    std::vector<VacancySide> vsides_;
    int westmost_frontier_id_ = 0;
    
    Vacancy(){};
    
    void insertCurves(const std::vector<std::array<double,3>> &grid_points, const std::vector<std::vector<int>> &lines){
        curves.insertEdges(grid_points, lines);
        vsides_.resize(lines.size());
        for (int foo=0; foo < lines.size(); foo++){
            int p0 = lines[foo][0];
            int p1 = lines[foo][1];
            Eigen::Vector2d p0coor = {grid_points[p0][0], grid_points[p0][1]};
            Eigen::Vector2d p1coor {grid_points[p1][0], grid_points[p1][1]};
            std::array<Eigen::Vector2d,2> foopts = {p0coor, p1coor};
            VacancySide vs(foopts, {p0,p1});
            vsides_[foo] = vs;
        }
        //Populate the neighborhood shapes.
        for (int foo=0; foo<vsides_.size(); foo++){
            std::cout << foo;
            VacancySide vs_foo = vsides_[foo];
            int dim_of_interest = vs_foo.orientation == EdgeOrientation::kNorth ? 0 : 1;
            auto edgefoo = curves.get_points_of_edge(foo);
            int pfoos = edgefoo[0];
            int pfoot = edgefoo[1];
            int pnext = curves.get_next_point(pfoot);
            int pprev = curves.get_prev_point(pfoos);
            std::cout << pprev <<" " << pfoos <<" " << pfoot << " " << pnext << std::endl;
            double pos_foo = curves.get_point(pfoos)[dim_of_interest];
            double pos_prev = curves.get_point(pprev)[dim_of_interest];
            double pos_next = curves.get_point(pnext)[dim_of_interest];
            bool is_prev_foo = pos_prev < pos_foo;
            bool is_foo_next = pos_foo < pos_next;
            if ((is_prev_foo & is_foo_next) | (!is_prev_foo & !is_foo_next)) {
                vs_foo.neighbor_shape = NeighborhoodShape::kZag;
                std::cout << "zag " << std::endl;
            } else {
                if (is_prev_foo & !is_foo_next){
                    if (dim_of_interest == 1){
                        std::cout << "sout " << std::endl;
                        vs_foo.neighbor_shape = NeighborhoodShape::kNotchSouth;
                    }
                    if (dim_of_interest == 0){
                    std::cout << "west " << std::endl; vs_foo.neighbor_shape = NeighborhoodShape::kNotchWest;
                    }
                }
                if (!is_prev_foo & is_foo_next){
                    if (dim_of_interest == 1){
                        std::cout << "north " << std::endl;
                        vs_foo.neighbor_shape = NeighborhoodShape::kNotchNorth;
                    }
                    if (dim_of_interest == 0) {
                        std::cout << "east " << std::endl;
                        vs_foo.neighbor_shape = NeighborhoodShape::kNotchEast;
                    }
                }
            }
            //side foo is at this position
        }
    };
};

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
    for (int foo=0; foo<vac.vsides_.size(); foo++){
        std::cout << foo << std::endl;
        VacancySide vs = vac.vsides_[foo];
        std::cout << " pt " << vs.point_ids[0] << " @ " << vs.points[0].transpose() << std::endl;
        std::cout << " pt " << vs.point_ids[1] << " @ " << vs.points[1].transpose() << std::endl;
        std::cout << "length " << vs.length << " ";
        if (vs.orientation == EdgeOrientation::kEast) std::cout << "east";
        if (vs.orientation == EdgeOrientation::kNorth) std::cout << "north";
        if (vs.neighbor_shape == NeighborhoodShape::kZag)            std::cout << "zag" << std::endl;

    }
};


int main() {
    std::cout << "Saluton Mundo!" << std::endl;
    example2();
}
