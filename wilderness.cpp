#include <unordered_map>

#include "wilderness.h"

VacancySide::VacancySide(const int edge_id_0, const std::array<Eigen::Vector2d,2> &i_points, const std::array<int,2> &i_point_ids){
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

void ZoningCommisioner::findFrontier(){
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

NeighborhoodShape ZoningCommisioner::getNeighborhoodShape(int dim_of_interest, double pos_prev, double pos_foo, double pos_next){
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

void ZoningCommisioner::insertCurves(const std::vector<std::array<double,3>> &grid_points, const std::vector<std::vector<int>> &lines){
    vacant_.setGridPointsAndCells(grid_points, lines);
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

void ZoningCommisioner::populateNeighbors(){
    //Populate the neighborhood shapes.
    for (int foo=0; foo<vsides_.size(); foo++){
        VacancySide &vs_foo = vsides_[foo];
        int dim_of_interest = vs_foo.orientation == EdgeOrientation::kNorth ? 0 : 1;
        auto edgefoo = vacant_.get_points_of_edge(foo);
        int pfoos = edgefoo[0];
        int pfoot = edgefoo[1];
        int pnext = vacant_.get_next_point(pfoot);
        int pprev = vacant_.get_prev_point(pfoos);
        double pos_foo = vacant_.get_point(pfoos)[dim_of_interest];
        double pos_prev = vacant_.get_point(pprev)[dim_of_interest];
        double pos_next = vacant_.get_point(pnext)[dim_of_interest];
        vs_foo.neighbor_shape = getNeighborhoodShape(dim_of_interest, pos_prev, pos_foo, pos_next);
        if (vs_foo.neighbor_shape == NeighborhoodShape::kNotchEast){
            Eigen::Vector2d left_p = 0.5 * (vs_foo.points[0] + vs_foo.points[1]) + Eigen::Vector2d(-10 * repsilon, 0.0);
            bool is_left_in = vacant_.is_point_in(left_p);
            vs_foo.outside = is_left_in ? OutsideSide::kRight : OutsideSide::kLeft;
        }
    };
};
    
void ZoningCommisioner::nextCollideNorth(const Eigen::Vector2d &origin, const bool &is_following_orientation, int &point_at, int &edge_at, Eigen::Vector2d &impact){
    //Continue along, testing horizontal edges for an impact
    //Advances the point and edge location.
    bool is_impact = false;
    if (is_following_orientation){
        edge_at = vacant_.get_next_edge(edge_at);
        point_at = vacant_.get_next_point(point_at);
    } else {
        edge_at = vacant_.get_prev_edge(edge_at);
        point_at = vacant_.get_prev_point(point_at);
    }
    while (!is_impact){
        if (is_following_orientation){
            edge_at = vacant_.get_next_edge(edge_at);
            point_at = vacant_.get_next_point(point_at);
        } else {
            edge_at = vacant_.get_prev_edge(edge_at);
            point_at = vacant_.get_prev_point(point_at);
        }
        if (vsides_[edge_at].orientation == EdgeOrientation::kEast){
            is_impact = rayTraceNorth(origin, vsides_[edge_at].points, impact);
        }
    }
};

Trail ZoningCommisioner::trailblaze(int start_edge_id){
    Trail patrol;
    VacancySide patrol_start  = vsides_[start_edge_id];
    //first the lower edge of the patrol
    
    //Determine which direction to traverse.
    int p_at;
    int p_next;
    bool is_following_orient;
    if (patrol_start.outside == OutsideSide::kLeft){
        p_at = patrol_start.point_ids[0];
        p_next = vacant_.get_next_point(p_at);
        is_following_orient = (p_next != patrol_start.point_ids[1]);
    } else {
        p_at = patrol_start.point_ids[1];
        p_next = vacant_.get_next_point(p_at);
        is_following_orient = (p_next != patrol_start.point_ids[0]);
    }
    //The BOTTOM
    bool is_end_found = false;
    int e_at = start_edge_id;
    patrol.landmarks_valley_.push_back(vacant_.get_point(p_at));
    VacancySide patrol_terminus;
    while(!is_end_found){
        if (is_following_orient){
            e_at = vacant_.get_next_edge(e_at);
            p_at = vacant_.get_next_point(p_at);
        } else {
            e_at = vacant_.get_prev_edge(e_at);
            p_at = vacant_.get_prev_point(p_at);
        }
        if (vsides_[e_at].neighbor_shape == NeighborhoodShape::kNotchWest){
            patrol_terminus = vsides_[e_at];
            is_end_found = true;
            patrol.landmarks_valley_.push_back(vacant_.get_point(p_at));
        } else {
            patrol.landmarks_valley_.push_back(vacant_.get_point(p_at));
        }
    }
    //TOP
    is_following_orient = !is_following_orient;
    is_end_found = false;
    e_at = start_edge_id;
    if (patrol_start.outside == OutsideSide::kLeft){
        p_at = patrol_start.point_ids[1];
    } else {
        Eigen::Vector2d impact;
        p_at = patrol_start.point_ids[0];
        Eigen::Vector2d loc = vacant_.get_point(p_at);
        nextCollideNorth(loc, is_following_orient, p_at, e_at, impact);
        patrol.landmarks_mountain_.push_back(impact);
    }
    patrol.landmarks_mountain_.push_back(vacant_.get_point(p_at));
    while(!is_end_found){
        //Advance to next edge
        if (is_following_orient){
            e_at = vacant_.get_next_edge(e_at);
            p_at = vacant_.get_next_point(p_at);
        } else {
            e_at = vacant_.get_prev_edge(e_at);
            p_at = vacant_.get_prev_point(p_at);
        }
        //check for the end
        if ((e_at == patrol_terminus.edge_id) | (e_at == eastmost_frontier_id_)){
            patrol_terminus = vsides_[e_at];
            patrol.landmarks_mountain_.push_back(vacant_.get_point(p_at));
            is_end_found = true;
        } else {
            //check if the you hit a westnotch and need to project north
            if(vsides_[e_at].neighbor_shape == NeighborhoodShape::kNotchWest){
                Eigen::Vector2d impact;
                int p_prev;
                if (is_following_orient){
                    p_prev = vacant_.get_prev_point(p_at);
                } else {
                    p_prev = vacant_.get_next_point(p_at);
                }
                Eigen::Vector2d loc = vacant_.get_point(p_prev);
                nextCollideNorth(loc, is_following_orient, p_at, e_at, impact);
                patrol.landmarks_mountain_.push_back(impact);
                Eigen::Vector2d landmark = vacant_.get_point(p_at);
                patrol.landmarks_mountain_.push_back(landmark);
            } else {
                Eigen::Vector2d landmark = vacant_.get_point(p_at);
                patrol.landmarks_mountain_.push_back(landmark);
            }
        }
    }
    return patrol;
};

void ZoningCommisioner::trailblaze(){
    for (int foo=0; foo<vsides_.size(); foo++){
        if (vsides_[foo].neighbor_shape == NeighborhoodShape::kNotchEast){
//            std::cout << "eastnotch at " << foo;
            trails_.push_back(trailblaze(foo));
        }
    }
};

bool ZoningCommisioner::findPlacement(const double& width, const double& height, Eigen::Vector2d& placement) const {
    bool is_placeable = false;
    for(const Trail& t : trails_){
        CaliperHiker caliper_hiker(t, width, height);
        caliper_hiker.hike();
        if (caliper_hiker.valid_.size()>0){
            Eigen::Vector2d val = caliper_hiker.getMostValid();
            if (is_placeable){
                if (val[1] < placement[1]) placement = val;
                if (std::abs(val[1]-placement[1]) < repsilon){
                    if (val[0] < placement[0]) placement = val;
                }
            } else {
                placement = val;
                is_placeable = true;
            }
        }
    };
    return is_placeable;
};

struct RectContact {
	int trail_id = -1;
	int wall_id = -1;
	int plateau_id = -1;
	SegmentRelation segment_relation;
};

void ZoningCommisioner::zoneOff(const Eigen::Vector2d& position, const double& width, const double& height) {
	//Removes the specified rectangle from the vacancy space.
	//Specifically assuming bottom left placement style.
	vacant_.removeTangentRectangle(position, width, height);
	vsides_.resize(vacant_.get_number_of_edges());
	for (int foo = 0; foo < vacant_.get_number_of_edges(); foo++) {
		auto eps = vacant_.get_points_of_edge(foo);
		int p0 = eps[0];
		int p1 = eps[1];
		Eigen::Vector2d p0coor = vacant_.get_point(p0);
		Eigen::Vector2d p1coor = vacant_.get_point(p1);
		std::array<Eigen::Vector2d, 2> foopts = { p0coor, p1coor };
		VacancySide vs(foo, foopts, { p0,p1 });
		vsides_[foo] = vs;
	};
	populateNeighbors();
	findFrontier();
	trails_.clear();
	trailblaze();
};


