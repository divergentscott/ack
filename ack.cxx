#include <array>
#include <deque>
#include <iostream>
#include <limits>
#include <list>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "cabby_curve_collection.h"

using vedge = std::array<Eigen::Vector2d,2>;
using hedge = std::array<Eigen::Vector2d,2>;

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
    int edge_id;
    double length = 0.0;
    EdgeOrientation orientation = EdgeOrientation::kBad;
    OutsideSide outside = OutsideSide::kUnknown;
    NeighborhoodShape neighbor_shape = NeighborhoodShape::kUnknown;
    std::array<int,2> point_ids = {0,0};
    //SS: question for future self: do you just want to represent as pt * interval?
    std::array<Eigen::Vector2d,2> points = {Eigen::Vector2d({0,0}),Eigen::Vector2d({0,0})};
    
    VacancySide(){};
    VacancySide(const int edge_id_0, const std::array<Eigen::Vector2d,2> &i_points, const std::array<int,2> &i_point_ids){
        //Vacancy side constructor from vertex ids and positions
        edge_id = edge_id_0;
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

struct Patrol{
    std::list<Eigen::Vector2d> landmarks_high;
    std::list<Eigen::Vector2d> landmarks_low;
    VacancySide start;
    VacancySide terminus;
};

struct Vacancy{
    CabbyCurveCollection curves;
    std::vector<VacancySide> vsides_;
    int westmost_frontier_id_ = 0;
    int eastmost_frontier_id_ = 0;
    
    Vacancy(){};
    
    void findFrontier(){
        int westmost = 0;
        double westest = vsides_[0].points[0][0];
        int eastmost = 0;
        double eastest = vsides_[0].points[0][0];
        for(int foo=0; foo<vsides_.size(); foo++){
            if (vsides_[foo].orientation == EdgeOrientation::kNorth){
                double longitude = vsides_[foo].points[0][0];
                if (longitude < westest){
                    westmost = foo;
                    westest = longitude;
                }
                if (eastest < longitude){
                    eastmost = foo;
                    eastest = longitude;
                }
            }
        }
        westmost_frontier_id_ = westmost;
        eastmost_frontier_id_ = eastmost;
    };
    
    NeighborhoodShape getNeighborhoodShape(int dim_of_interest, double pos_prev, double pos_foo, double pos_next){
        bool is_prev_foo = pos_prev < pos_foo;
        bool is_foo_next = pos_foo < pos_next;
        if ((is_prev_foo & is_foo_next) | (!is_prev_foo & !is_foo_next)) {
            return NeighborhoodShape::kZag;
        } else {
            if (is_prev_foo & !is_foo_next){
                if (dim_of_interest == 1){
                    return NeighborhoodShape::kNotchSouth;
                }
                if (dim_of_interest == 0){
                    return NeighborhoodShape::kNotchWest;
                }
            }
            if (!is_prev_foo & is_foo_next){
                if (dim_of_interest == 1){
                    return NeighborhoodShape::kNotchNorth;
                }
                if (dim_of_interest == 0) {
                   return NeighborhoodShape::kNotchEast;
                }
            }
        }
        return NeighborhoodShape::kUnknown;
    };
    
    void insertCurves(const std::vector<std::array<double,3>> &grid_points, const std::vector<std::vector<int>> &lines){
        curves.insertEdges(grid_points, lines);
        vsides_.resize(lines.size());
        for (int foo=0; foo < lines.size(); foo++){
            int p0 = lines[foo][0];
            int p1 = lines[foo][1];
            Eigen::Vector2d p0coor = {grid_points[p0][0], grid_points[p0][1]};
            Eigen::Vector2d p1coor {grid_points[p1][0], grid_points[p1][1]};
            std::array<Eigen::Vector2d,2> foopts = {p0coor, p1coor};
            VacancySide vs(foo, foopts, {p0,p1});
            vsides_[foo] = vs;
        };
    };
    
    void populateNeighbors(){
        //Populate the neighborhood shapes.
        for (int foo=0; foo<vsides_.size(); foo++){
            std::cout << foo;
            VacancySide &vs_foo = vsides_[foo];
            int dim_of_interest = vs_foo.orientation == EdgeOrientation::kNorth ? 0 : 1;
            auto edgefoo = curves.get_points_of_edge(foo);
            int pfoos = edgefoo[0];
            int pfoot = edgefoo[1];
            int pnext = curves.get_next_point(pfoot);
            int pprev = curves.get_prev_point(pfoos);
            double pos_foo = curves.get_point(pfoos)[dim_of_interest];
            double pos_prev = curves.get_point(pprev)[dim_of_interest];
            double pos_next = curves.get_point(pnext)[dim_of_interest];
            vs_foo.neighbor_shape = getNeighborhoodShape(dim_of_interest, pos_prev, pos_foo, pos_next);
            if (vs_foo.neighbor_shape == NeighborhoodShape::kNotchEast){
                Eigen::Vector2d left_p = 0.5 * (vs_foo.points[0] + vs_foo.points[1]) + Eigen::Vector2d(-10 * repsilon, 0.0);
                bool is_left_in = curves.is_point_in(left_p);
                vs_foo.outside = is_left_in ? OutsideSide::kRight : OutsideSide::kLeft;
            }
        };
    };
    
    void nextCollideNorth(const Eigen::Vector2d &origin, const bool &is_following_orientation, int &point_at, int &edge_at, Eigen::Vector2d &impact){
        //Continue along, testing horizontal edges for an impact
        //Advances the point and edge location.
        bool is_impact = false;
        if (is_following_orientation){
            edge_at = curves.get_next_edge(edge_at);
            point_at = curves.get_next_point(point_at);
        } else {
            edge_at = curves.get_prev_edge(edge_at);
            point_at = curves.get_prev_point(point_at);
        }
        while (!is_impact){
            if (is_following_orientation){
                edge_at = curves.get_next_edge(edge_at);
                point_at = curves.get_next_point(point_at);
            } else {
                edge_at = curves.get_prev_edge(edge_at);
                point_at = curves.get_prev_point(point_at);
            }
            if (vsides_[edge_at].orientation == EdgeOrientation::kEast){
                is_impact = curves.rayTraceNorth(origin, vsides_[edge_at].points, impact);
                std::cout << edge_at << " has impact " << is_impact << std::endl;
            }
        }
    }
    
    Patrol spawnPatrol(int start_edge_id){
        Patrol patrol;
        patrol.start = vsides_[start_edge_id];
        //first the lower edge of the patrol
        
        //Determine which direction to traverse.
        int p_at;
        int p_next;
        std::cout << "start at " << start_edge_id << " points " << patrol.start.point_ids[0] << " " << patrol.start.point_ids[1] << std::endl;
        bool is_following_orient;
        if (patrol.start.outside == OutsideSide::kLeft){
            p_at = patrol.start.point_ids[0];
            p_next = curves.get_next_point(p_at);
            is_following_orient = (p_next != patrol.start.point_ids[1]);
        } else {
            p_at = patrol.start.point_ids[1];
            p_next = curves.get_next_point(p_at);
            is_following_orient = (p_next != patrol.start.point_ids[0]);
        }
        //The BOTTOM
        std::cout << "BOTTOM " << std::endl;
        std::cout<< "TRUTH " << true << std::endl;
        std::cout << "with orientation " << is_following_orient << std::endl;
        bool is_end_found = false;
        int e_at = start_edge_id;
        patrol.landmarks_low.push_back(curves.get_point(p_at));
        while(!is_end_found){
            if (is_following_orient){
                e_at = curves.get_next_edge(e_at);
                p_at = curves.get_next_point(p_at);
            } else {
                e_at = curves.get_prev_edge(e_at);
                p_at = curves.get_prev_point(p_at);
            }
            if (vsides_[e_at].neighbor_shape == NeighborhoodShape::kNotchWest){
                patrol.terminus = vsides_[e_at];
                std::cout << "end at " << e_at << std::endl;
                is_end_found = true;
            } else {
                patrol.landmarks_low.push_back(curves.get_point(p_at));
                std::cout << "add point " << p_at << std::endl;
            }
        }
        //TOP
        std::cout << "TOP " << std::endl;
        is_following_orient = !is_following_orient;
        is_end_found = false;
        e_at = start_edge_id;
        if (patrol.start.outside == OutsideSide::kLeft){
            p_at = patrol.start.point_ids[1];
        } else {
            Eigen::Vector2d impact;
            p_at = patrol.start.point_ids[0];
            Eigen::Vector2d loc = curves.get_point(p_at);
            nextCollideNorth(loc, is_following_orient, p_at, e_at, impact);
            patrol.landmarks_high.push_back(impact);
        }
        patrol.landmarks_high.push_back(curves.get_point(p_at));
        std::cout << " add point " << curves.get_point(p_at).transpose() << std::endl;
        while(!is_end_found){
            //Advance to next edge
            if (is_following_orient){
                e_at = curves.get_next_edge(e_at);
                p_at = curves.get_next_point(p_at);
            } else {
                e_at = curves.get_prev_edge(e_at);
                p_at = curves.get_prev_point(p_at);
            }
            //check for the end
            if ((e_at == patrol.terminus.edge_id) | (e_at == eastmost_frontier_id_)){
                patrol.terminus = vsides_[e_at];
                std::cout << "end at " << e_at << std::endl;
                is_end_found = true;
            } else {
                //check if the you hit a westnotch and need to project north
                if(vsides_[e_at].neighbor_shape == NeighborhoodShape::kNotchWest){
                    Eigen::Vector2d impact;
                    int p_prev;
                    if (is_following_orient){
                        p_prev = curves.get_prev_point(p_at);
                    } else {
                        p_prev = curves.get_next_point(p_at);
                    }
                    Eigen::Vector2d loc = curves.get_point(p_prev);
                    nextCollideNorth(loc, is_following_orient, p_at, e_at, impact);
                    patrol.landmarks_high.push_back(impact);
                    Eigen::Vector2d landmark = curves.get_point(p_at);
                    patrol.landmarks_high.push_back(landmark);
                    std::cout << "add point " << p_at << " location " << landmark.transpose() << std::endl;
                } else {
                    Eigen::Vector2d landmark = curves.get_point(p_at);
                    patrol.landmarks_high.push_back(landmark);
                    std::cout << "add point " << p_at << " location " << landmark.transpose() << std::endl;
                }
            }
        }
        return patrol;
    }
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
    Patrol patty11 = vac.spawnPatrol(11);
    Patrol patty7 = vac.spawnPatrol(7);
    
};

bool rayTraceEast(const Eigen::Vector2d &origin, const vedge &segment, Eigen::Vector2d &impact){
    //Returns true if there is an impact, otherwise false.
    //If true, impact_location is populated with the impact point on the line segment.
    //Assumes segment is vertical AND ordered with segment[0][1] < segment[1][1] and segment[0][0] == semgent[1][0]
    if ((origin[0] > segment[0][0]) | (segment[0][1] > origin[1]) | (origin[1] > segment[1][1])) return false;
    impact[1] = origin[1];
    impact[0] = segment[0][0];
    return true;
}

struct CaliperSlider{
    std::list<Eigen::Vector2d> caliper_spots_;
    std::list<vedge> landmarks_;
    Eigen::Vector2d support_;
    std::deque<Eigen::Vector2d> deck_;
    double length_;
    Eigen::Vector2d caliper_pos_;
    //
    CaliperSlider(){};
    //
    std::list<Eigen::Vector2d> bottomCaliperPositions(std::list<vedge> landmarks, double length){
    // for now landmarks are paired as edges low, high
        landmarks_ = landmarks;
        length_ = length;
        std::list<vedge>::iterator landmark_it = landmarks.begin();
        vedge line0 = *landmark_it;
        caliper_pos_ = line0[0] - Eigen::Vector2d({length,0});
        caliper_spots_.push_back(caliper_pos_);
        //
        landmark_it++;
        //find the support
        rayTraceEast(caliper_pos_, *landmark_it, support_);
        slide(1);
        for (std::list<Eigen::Vector2d>::iterator it=caliper_spots_.begin(); it != caliper_spots_.end(); ++it){
            //Need to purge thie points that are problems
        }
        
        return caliper_spots_;
    }
    void slide(int start_position){
        const int num_landmarks = landmarks_.size();
        while (start_position <= num_landmarks){
            int ii = start_position;
            std::list<vedge>::iterator landmark_it = landmarks_.begin();
            for (int foo=0; foo<ii; foo++) landmark_it++;
//            vedge landmark = ;
            //slide on support
            double x_patrol_spot = (*landmark_it)[0][0];
            while(x_patrol_spot <= support_[0] + length_){
                double top_y = (*landmark_it)[1][1];
                if (top_y > caliper_pos_[1]){
                    //vedge pushes the caliper up
                    caliper_pos_ = {(*landmark_it)[1][0] - length_, top_y};
                    caliper_spots_.push_back(caliper_pos_);
                    deck_.clear();
                    landmark_it++;
                    rayTraceEast(caliper_pos_, *landmark_it, support_);
                    slide(ii+11);
                    break;
                } else if (ii == num_landmarks){
                    break;
                } else {
                    landmark_it++;
                    ii++;
                    x_patrol_spot = (*landmark_it)[0][0];
                }
            }
            //fall down to the next support
            caliper_pos_ = support_;
            caliper_spots_.push_back(caliper_pos_);
            std::deque<Eigen::Vector2d> ondeck = setup(start_position, ii);
            merge(deck_, ondeck);
            hedge segment = *deck.begin();
            deck_.pop_front();
            caliper_pos_[1] = segment[0][1];
            caliper_spots_.push_back(caliper_pos_);
            start_position = ii;
            support_ = segment[1];
        }
    };
    
    std::deque<Eigen::Vector2d> setup(int start_position, int end){
        std::deque<Eigen::Vector2d> ondeck;
        int ii = end - 1;
        
        while( ii > start_position ){
        }
    };
    
    void merge(std::deque<Eigen::Vector2d> &q_0, std::deque<Eigen::Vector2d> &q_1){
        
    };
};
    
    
    

void example3(){
    std::list<std::array<Eigen::Vector2d,2>> landmarks_low;
    landmarks_low.push_back({Eigen::Vector2d({0,0}),Eigen::Vector2d({0,1})});
    landmarks_low.push_back({Eigen::Vector2d({2,-1}),Eigen::Vector2d({2,0})});
    landmarks_low.push_back({Eigen::Vector2d({5,-1}),Eigen::Vector2d({5,3})});
    landmarks_low.push_back({Eigen::Vector2d({6,0}),Eigen::Vector2d({6,1})});
    landmarks_low.push_back({Eigen::Vector2d({7,0}),Eigen::Vector2d({7,1})});
    landmarks_low.push_back({Eigen::Vector2d({8,0}),Eigen::Vector2d({8,1})});
    CaliperSlider cs;
    auto calipos = cs.bottomCaliperPositions(landmarks_low, 2.4);
};

int main() {
    std::cout << "Saluton Mundo!" << std::endl;
    example3();
}
