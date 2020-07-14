#include <unordered_map>

#include "wilderness.h"

//VacancySide::VacancySide(const int edge_id_0, const std::array<Eigen::Vector2d,2> &i_points, const std::array<int,2> &i_point_ids){
//    //Vacancy side constructor from vertex ids and positions
//    edge_id = edge_id_0;
//    double xlen = std::abs((i_points[0][0] - i_points[1][0]));
//    double ylen = std::abs((i_points[0][1] - i_points[1][1]));
//    if ( (xlen < repsilon) & (ylen > repsilon) ){
//        //Decide that the segment is vertical
//        orientation = EdgeOrientation::kNorth;
//        length = ylen;
//        if (i_points[0][1] <= i_points[1][1]){
//            point_ids = i_point_ids;
//            points = i_points;
//        } else {
//            point_ids = {i_point_ids[1], i_point_ids[0]};
//            points = {i_points[1], i_points[0]};
//        }
//    }
//    if ( (xlen > repsilon) & (ylen < repsilon) ){
//        //Decide that the segment is horizontal
//        orientation = EdgeOrientation::kEast;
//        length = xlen;
//        if (i_points[0][0] <= i_points[1][0]){
//            point_ids = i_point_ids;
//            points = i_points;
//        } else {
//            point_ids = {i_point_ids[1], i_point_ids[0]};
//            points = {i_points[1], i_points[0]};
//        }
//
//    }
//};

void ZoningCommisioner::findFrontier(){
    int westmost = -1;
    int eastmost = -1;
	double eastest = std::numeric_limits<double>::max();
	double westest = std::numeric_limits<double>::min();
	for(int foo = 0; foo < vacant_.get_number_of_edges(); foo++){
        if (!is_edge_horizontals_[foo]){
			bool _;
			Cedge c0 = getCedge(0, _);
            double longitude = c0[0][0];
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
};

Cedge ZoningCommisioner::getCedge(const int id, bool& is_horizontal) const {
	auto pids = vacant_.get_points_of_edge(id);
	Eigen::Vector2d a = vacant_.get_point(pids[0]);
	Eigen::Vector2d b = vacant_.get_point(pids[1]);
	if (std::abs(a[1] - b[1]) < repsilon) {
		is_horizontal = true;
		if (a[0] < b[0]) return { a,b };
		return { b,a };
	}
	is_horizontal = false;
	if (a[1] < b[1]) return { a,b };
	return { b,a };
};

Cedge ZoningCommisioner::getCedge(const int id, bool& is_horizontal, std::array<int, 2> &point_ids) const {
	auto pids = vacant_.get_points_of_edge(id);
	Eigen::Vector2d a = vacant_.get_point(pids[0]);
	Eigen::Vector2d b = vacant_.get_point(pids[1]);
	if (std::abs(a[1] - b[1]) < repsilon) {
		is_horizontal = true;
		if (a[0] < b[0]) {
			point_ids = pids;
			return { a,b };
		}
		point_ids = {pids[1], pids[0]};
		return { b,a };
	}
	is_horizontal = false;
	if (a[1] < b[1]) {
		point_ids = pids;
		return { a,b };
	}
	point_ids = { pids[1], pids[0] };
	return { b,a };
};


void ZoningCommisioner::populateSlopes() {
	is_edge_horizontals_.resize(vacant_.get_number_of_edges());
	outsidesides_.resize(vacant_.get_number_of_edges());
	for (auto foo = 0; foo < vacant_.get_number_of_edges(); foo++) {
		auto edgefoo = vacant_.get_points_of_edge(foo);
		int pfoo_s_id = edgefoo[0];
		int pfoo_t_id = edgefoo[1];
		Eigen::Vector2d pfoo_s = vacant_.get_point(pfoo_s_id);
		Eigen::Vector2d pfoo_t = vacant_.get_point(pfoo_t_id);
		double xlen = std::abs((pfoo_s[0] - pfoo_t[0]));
		double ylen = std::abs((pfoo_s[1] - pfoo_t[1]));
		if ((xlen > repsilon) & (ylen < repsilon)) {
			is_edge_horizontals_[foo] = true;
			Eigen::Vector2d test_p = 0.5 * (pfoo_s + pfoo_t) - Eigen::Vector2d({0, 10*repsilon});
			outsidesides_[foo] = vacant_.is_point_in(test_p) ? OutsideSide::kPositive : OutsideSide::kNegative;
		}
		else {
			is_edge_horizontals_[foo] = false;
			Eigen::Vector2d test_p = 0.5 * (pfoo_s + pfoo_t) - Eigen::Vector2d({10 * repsilon, 0});
			outsidesides_[foo] = vacant_.is_point_in(test_p) ? OutsideSide::kPositive : OutsideSide::kNegative;
		}
	};
}

void ZoningCommisioner::populateNeighbors(){
	// Populate the slopes.
    populateSlopes();
	// Populate the neighborhood shapes.
	shapes_.resize(vacant_.get_number_of_edges());
	//
    for (int foo=0; foo<vacant_.get_number_of_edges(); foo++){
        int dim_of_interest = is_edge_horizontals_[foo] ? 1 : 0;
        auto edgefoo = vacant_.get_points_of_edge(foo);
        int pfoos = edgefoo[0];
        int pfoot = edgefoo[1];
        int pnext = vacant_.get_next_point(pfoot);
        int pprev = vacant_.get_prev_point(pfoos);
        double pos_foo = vacant_.get_point(pfoos)[dim_of_interest];
        double pos_prev = vacant_.get_point(pprev)[dim_of_interest];
        double pos_next = vacant_.get_point(pnext)[dim_of_interest];
        shapes_[foo] = getNeighborhoodShape(dim_of_interest, pos_prev, pos_foo, pos_next);
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
        if (is_edge_horizontals_[edge_at]){
			bool _;
            is_impact = rayTraceNorth(origin, getCedge(edge_at,_), impact);
        }
    }
};

Trail ZoningCommisioner::trailblaze(int start_edge_id){
    Trail patrol;
    //first the lower edge of the patrol
    
    //Determine which direction to traverse.
    int p_at;
    int p_next;
    bool is_following_orient;
	bool _;
	std::array<int, 2> pids;
	getCedge(start_edge_id, _, pids);
	if (outsidesides_[start_edge_id] == OutsideSide::kNegative){
        p_at = pids[0];
        p_next = pids[1];
    } else {
		p_at = pids[1];
		p_next = pids[0];
	}
	is_following_orient = (p_next != vacant_.get_next_point(p_at));
	//The BOTTOM
    bool is_end_found = false;
    int e_at = start_edge_id;
    patrol.landmarks_valley_.push_back(vacant_.get_point(p_at));
	int patrol_terminus;
    while(!is_end_found){
        if (is_following_orient){
            e_at = vacant_.get_next_edge(e_at);
            p_at = vacant_.get_next_point(p_at);
        } else {
            e_at = vacant_.get_prev_edge(e_at);
            p_at = vacant_.get_prev_point(p_at);
        }
        if (shapes_[e_at] == NeighborhoodShape::kNotchWest){
            patrol_terminus = e_at;
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
    if (outsidesides_[start_edge_id] == OutsideSide::kNegative){
        p_at = pids[1];
    } else {
        Eigen::Vector2d impact;
        p_at = pids[0];
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
        if ((e_at == patrol_terminus) | (e_at == eastmost_frontier_id_)){
            patrol_terminus = e_at;
            patrol.landmarks_mountain_.push_back(vacant_.get_point(p_at));
            is_end_found = true;
        } else {
            //check if the you hit a westnotch and need to project north
            if(shapes_[e_at] == NeighborhoodShape::kNotchWest){
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
    for (auto foo=0; foo< vacant_.get_number_of_edges(); foo++){
        if (shapes_[foo] == NeighborhoodShape::kNotchEast){
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
	populateNeighbors();
	findFrontier();
	trails_.clear();
	trailblaze();
};


