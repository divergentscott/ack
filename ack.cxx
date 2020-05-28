#include <array>
#include <iostream>
#include <limits>
#include <vector>
#include <Eigen/Dense>
static const double repsilon = std::sqrt(std::numeric_limits<double>::epsilon());

Eigen::Vector2d ray_segment_intersect(const Eigen::Vector2d orig,
                                                 const Eigen::Vector2d dir,
                                                 const Eigen::Vector2d a,
                                                 const Eigen::Vector2d b) {
    // Compute ray, line segment intersection.
    // Ray has origin orig and direction r. That is: { orig + t * r | t>0 }
    // Line segment endpoints a b. That is: { a * s + b * (1-s) | s in [0,1] }
    // Computes the parameter t and s at intersection.
    // If the ray and segment are colinear, there may be infinitely many
    // solutions. In that case return the s = 0 solution,
    // so t is such that: orig + t * r = b.
    double a01 = b[0] - a[0];
    double a11 = b[1] - a[1];
    double det = dir[0] * a11 - a01 * dir[1];
    if (std::abs(det) > repsilon) {
        // Noncolinear
        double s0 = b[0] - orig[0];
        double s1 = b[1] - orig[1];
        double t0 = (a11 * s0 - a01 * s1) / det;
        double t1 = (dir[0] * s1 - dir[1] * s0) / det;
        return {t0, t1};
    }
    // Colinear
    double t1;
    if (a01 > repsilon) {
        t1 = (b[0] - orig[0]) / a01;
    } else {
        t1 = (b[1] - orig[1]) / a11;
    }
    if ((-repsilon < t1) & (t1 < 1 + repsilon)) {
        return {0.0, t1};
    }
    if (dir[0] > repsilon) {
        return {(b[0] - orig[0]) / dir[0], 0.0};
    }
    return {(b[1] - orig[1]) / dir[1], 0.0};
}

bool is_ray_segment_intersect(const Eigen::Vector2d orig,
                              const Eigen::Vector2d dir,
                              const Eigen::Vector2d a,
                              const Eigen::Vector2d b) {
    Eigen::Vector2d t = ray_segment_intersect(orig, dir, a, b);
    return (t[0] > -repsilon) & (-repsilon < t[1]) &
           (t[1] < 1 + repsilon);
}

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

struct VacancySide{
    double length = 0.0;
    EdgeOrientation orientation = EdgeOrientation::kBad;
    OutsideSide outside = OutsideSide::kUnknown;
    std::array<int,2> point_ids = {0,0};
    //SS: question for future self: do you just want to represent as pt * interval?
    std::array<Eigen::Vector2d,2> points;
    VacancySide(const std::array<int,2> &i_point_ids, const std::array<Eigen::Vector2d,2> &i_points){
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


void example1(){
    
    std::vector<Eigen::Vector2d> grid_points = {
        {0, 0},
        {5, 0},
        {5, 1},
        {6, 1},
        {6, 2},
        {5, 2},
        {5, 3},
        {1, 3},
        {1, 2},
        {2, 2},
        {2, 1},
        {0, 1},
        {3, 1},
        {4, 1},
        {4, 2},
        {3, 2},
    };
    
    std::vector<std::array<int,2>> lines = {
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
        {15,12},
    };

    std::vector<std::array<int,2>> point_to_line;
    std::vector<EdgeOrientation> line_orientation;
    std::vector<OutsideSide> line_outside_side;
    for (int foo=0; foo < lines.size(); foo++){
        int p0 = lines[foo][0];
        int p1 = lines[foo][1];
        std::array<Eigen::Vector2d,2> foopts = {grid_points[p0], grid_points[p1]};
        VacancySide vs(lines[foo], foopts);
        std::cout << foo << std::endl;
        std::cout << " pt " << vs.point_ids[0] << " @ " << vs.points[0].transpose() << std::endl;
        std::cout << " pt " << vs.point_ids[1] << " @ " << vs.points[1].transpose() << std::endl;
        std::cout << "length " << vs.length << " ";
        if (vs.orientation == EdgeOrientation::kEast) std::cout << "east" << std::endl;
        if (vs.orientation == EdgeOrientation::kNorth) std::cout << "north" << std::endl;
    }
};


int main() {
    std::cout << "Saluton Mundo!" << std::endl;
    example1();

}
