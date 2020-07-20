#include <unordered_map>

#include "zoning_commisioner.h"


void ZoningCommisioner::findFrontier(){
	int n_edges = vacant_.get_number_of_edges();
	if (n_edges < 4) return;
    int westmost = -1;
    int eastmost = -1;
	double eastest = std::numeric_limits<double>::lowest();
	double westest = std::numeric_limits<double>::max();
	for(auto e_foo = 0; e_foo < n_edges; e_foo++){
        if (!vacant_.is_horizontals_[e_foo]){
			bool _;
			Cedge c0 = getCedge(e_foo, _);
            double longitude = c0[0][0];
            if (longitude < westest){
                westmost = e_foo;
                westest = longitude;
            }
            if (longitude > eastest){
                eastmost = e_foo;
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

void ZoningCommisioner::insertCurves(const std::vector<Eigen::Vector2d> &grid_points, const std::vector<std::vector<int>> &lines){
    vacant_.setGridPointsAndCells(grid_points, lines);
    vacant_.mergeParallelEdges();
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

void ZoningCommisioner::populateNeighbors(){
	// Populate the slopes.
    vacant_.populateSlopes();
	// Populate the neighborhood shapes.
	shapes_.resize(vacant_.get_number_of_edges());
	//
    for (int foo=0; foo<vacant_.get_number_of_edges(); foo++){
        int dim_of_interest = vacant_.is_horizontals_[foo] ? 1 : 0;
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
    
//void ZoningCommisioner::nextCollideNorth(const Eigen::Vector2d &origin, const bool &is_following_orientation, int &point_at, int &edge_at, Eigen::Vector2d &impact){
//    //Continue along, testing horizontal edges for an impact
//    //Advances the point and edge location.
//    bool is_impact = false;
//    if (is_following_orientation){
//        edge_at = vacant_.get_next_edge(edge_at);
//        point_at = vacant_.get_next_point(point_at);
//    } else {
//        edge_at = vacant_.get_prev_edge(edge_at);
//        point_at = vacant_.get_prev_point(point_at);
//    }
//    while (!is_impact){
//        if (is_following_orientation){
//            edge_at = vacant_.get_next_edge(edge_at);
//            point_at = vacant_.get_next_point(point_at);
//        } else {
//            edge_at = vacant_.get_prev_edge(edge_at);
//            point_at = vacant_.get_prev_point(point_at);
//        }
//        if (edge_at == eastmost_frontier_id_) break; //?
//        if (vacant_.is_horizontals_[edge_at]){
//			bool _;
//            is_impact = rayNorthSegmentIntersect(origin, getCedge(edge_at,_), impact);
//        }
//    }
//};

Zone ZoningCommisioner::trailblaze(int start_edge_id){
    Zone patrol;
    //first the lower edge of the patrol
    
    //Determine which direction to traverse.
    int p_at;
    int p_next;
    bool is_following_orient;
	bool _;
	std::array<int, 2> pids;
	getCedge(start_edge_id, _, pids);
	if (vacant_.outsidesides_[start_edge_id] == OutsideSide::kNegative){
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
    patrol.landmarks_downtown_.push_back(vacant_.get_point(p_at));
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
			is_end_found = true;
			OutsideSide outside_side = vacant_.outsidesides_[e_at];
			if (outside_side == OutsideSide::kPositive) {
				patrol_terminus = e_at;
				patrol.landmarks_downtown_.push_back(vacant_.get_point(p_at));
			} else {
				Eigen::Vector2d near_end = vacant_.get_point(p_at);
				patrol.landmarks_downtown_.push_back(near_end);
				Eigen::Vector2d hit_west;
				int hit_west_e_id;
				vacant_.rayTraceEast(near_end, hit_west, hit_west_e_id);
				patrol.landmarks_downtown_.push_back(hit_west);
				bool _;
				patrol.landmarks_downtown_.push_back(getCedge(hit_west_e_id,_)[1]);
				patrol_terminus = hit_west_e_id;
			}
        } else {
            patrol.landmarks_downtown_.push_back(vacant_.get_point(p_at));
        }
    }
    //TOP
    is_following_orient = !is_following_orient;
    is_end_found = false;
    e_at = start_edge_id;
    if (vacant_.outsidesides_[start_edge_id] == OutsideSide::kNegative){
        p_at = pids[1];
    } else {
        Eigen::Vector2d impact;
        p_at = pids[0];
        Eigen::Vector2d loc = vacant_.get_point(p_at);
		bool is_a_ceil = vacant_.rayTraceNorth(loc, impact, e_at);
		if (!is_a_ceil) throw std::logic_error("Error in zone covering.");
		if (is_following_orient) {
			p_at = vacant_.get_points_of_edge(e_at)[1];
		} else {
			p_at = vacant_.get_points_of_edge(e_at)[0];
		}
        //nextCollideNorth(loc, is_following_orient, p_at, e_at, impact);
        patrol.landmarks_uptown_.push_back(impact);
    }
    patrol.landmarks_uptown_.push_back(vacant_.get_point(p_at));
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
//            patrol_terminus = e_at;
            patrol.landmarks_uptown_.push_back(vacant_.get_point(p_at));
            is_end_found = true;
        } else {
			//check if the you hit a westnotch and need to project north
			if (shapes_[e_at] == NeighborhoodShape::kNotchWest) {
				// The outsideside matter here, why isn't it considered?
				int p_prev;
				if (is_following_orient) {
					p_prev = vacant_.get_prev_point(p_at);
				}
				else {
					p_prev = vacant_.get_next_point(p_at);
				}
				Eigen::Vector2d loc = vacant_.get_point(p_prev);
				Eigen::Vector2d impact;
				bool is_a_ceil = vacant_.rayTraceNorth(loc + Eigen::Vector2d({repsilon,0}), impact, e_at);
				if (!is_a_ceil) throw std::logic_error("Error in zone covering: Nothing is north of current position!");

				if (is_following_orient) {
					p_at = vacant_.get_points_of_edge(e_at)[1];
				}
				else {
					p_at = vacant_.get_points_of_edge(e_at)[0];
				}
				patrol.landmarks_uptown_.push_back(impact);
				Eigen::Vector2d landmark = vacant_.get_point(p_at);
				patrol.landmarks_uptown_.push_back(landmark);

            } else {
                Eigen::Vector2d landmark = vacant_.get_point(p_at);
                patrol.landmarks_uptown_.push_back(landmark);
            }
        }
    }
    return patrol;
};

void ZoningCommisioner::trailblaze(){
	findFrontier();
    for (auto foo=0; foo< vacant_.get_number_of_edges(); foo++){
        if (shapes_[foo] == NeighborhoodShape::kNotchEast){
//            std::cout << "eastnotch at " << foo;
            trails_.push_back(trailblaze(foo));
        }
    }
};

bool ZoningCommisioner::findPlacement(const double& width, const double& height, Eigen::Vector2d& placement) const {
    bool is_placeable = false;
    for(const Zone& t : trails_){
        Surveyor caliper_hiker(t, width, height);
        caliper_hiker.hike();
        if (caliper_hiker.valid_lots_.size()>0){
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
    vacant_.mergeParallelEdges();
	populateNeighbors();
	trails_.clear();
	trailblaze();
};


