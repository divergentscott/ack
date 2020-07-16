#include "cabbie_curve_collection.h"

#include <unordered_map>

// Tao is 2pi
const double kConstantTao =
    6.28318530717958647692528676655900576839433879875021164194;

const double kREpsilon = std::sqrt(std::numeric_limits<double>::epsilon());

// Random number generator
double randomAngle() {
    /*
    Random double in the range (0, 2pi)
    */
    return ((double)std::rand()) / RAND_MAX * kConstantTao;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* COLLECTION OF SIMPLE CLOSED CURVES IN THE PLANE */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

Eigen::Vector2d CurveCollection::ray_segment_intersect(const Eigen::Vector2d orig,
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
    if (std::abs(det) > kREpsilon) {
        // Noncolinear
        double s0 = b[0] - orig[0];
        double s1 = b[1] - orig[1];
        double t0 = (a11 * s0 - a01 * s1) / det;
        double t1 = (dir[0] * s1 - dir[1] * s0) / det;
        return {t0, t1};
    }
    // Colinear
    double t1;
    if (a01 > kREpsilon) {
        t1 = (b[0] - orig[0]) / a01;
    } else {
        t1 = (b[1] - orig[1]) / a11;
    }
    if ((-kREpsilon < t1) & (t1 < 1 + kREpsilon)) {
        return {0.0, t1};
    }
    if (dir[0] > kREpsilon) {
        return {(b[0] - orig[0]) / dir[0], 0.0};
    }
    return {(b[1] - orig[1]) / dir[1], 0.0};
}

bool CurveCollection::is_ray_segment_intersect(const Eigen::Vector2d orig,
                                               const Eigen::Vector2d dir,
                                               const Eigen::Vector2d a,
                                               const Eigen::Vector2d b) {
    Eigen::Vector2d t = ray_segment_intersect(orig, dir, a, b);
    return (t[0] > -kREpsilon) & (-kREpsilon < t[1]) &
           (t[1] < 1 + kREpsilon);
}

bool CurveCollection::is_point_in(const Eigen::Vector2d point) {
    /* Determine if a point lies in a curve collecion via parity of a random
     * direction ray trace. */
    double theta = randomAngle();
    Eigen::Vector2d direct{cos(theta), sin(theta)};
    int isect_count = 0;
    for (int foo = 0; foo < edges_.size(); foo++) {
        Eigen::Vector2d point_foo = points_[edges_[foo][0]];
        Eigen::Vector2d point_next = points_[edges_[foo][1]];
        Eigen::Vector2d isect_params =
            ray_segment_intersect(point, direct, point_foo, point_next);
        // Failure!
        if ((isect_params[1] == 0.0) & (isect_params[0] > 0.0)) {
            // If an edge is a subset of a ray, the pairity test will fail.
            // The only hope to resolve is to check the direction of nonparallel
            // edges o This is slightly dangerous! Technically could be an
            // infinite loop... If the segment fully lies inside the ray, we
            // reroll the ray direction and restart. std::cout << "Segment is
            // subset of ray! Reroll";
            theta = randomAngle();
            direct = {cos(theta), sin(theta)};
            isect_count = 0;
            foo = -1;
            continue;
        }
        // But normally the ray hits the segment interior.
        if ((-kREpsilon < isect_params[1]) &
            (isect_params[1] < 1 + kREpsilon)) {
            if (std::abs(isect_params[0]) <= kREpsilon) {
                return true;
            } else if (isect_params[0] > 0) {
                isect_count++;
            }
        }
    }
    return (isect_count % 2) == 1;
}

double CurveCollection::distance_till_impact(const int pointid,
                                             const Eigen::Vector2d dir) {
    /* Compute the distance on a line at orig with direction +/- dir travels
     * before hitting an edge not containing pointid.*/
    double dist = std::numeric_limits<double>::infinity();
    Eigen::Vector2d at = points_[pointid];
    for (auto edge : edges_) {
        if ((pointid != edge[0]) & (pointid != edge[1])) {
            Eigen::Vector2d impact = ray_segment_intersect(at, dir, points_[edge[0]],
                                                     points_[edge[1]]);
            // impact[0] is the length along the ray, impact[1] is the length
            // along the line containing edge
            if ((impact[1] < 1 + kREpsilon) &
                (impact[1] > -kREpsilon)) {
                dist = std::min(std::abs(impact[0]), dist);
            }
        }
    }
    return dist;
}

std::array<int,2> CurveCollection::neighborhood_orientation(int pointid) {
    /*
     Compute curve orientation at a point by determining if the interior lies to
     the left or right. Return <previous point, next point> Ray traces against
     all edges from at least 3 points.
    */
    // Construct a point to the left to test if that's inside the curve
    Eigen::Vector2d at = points_[pointid];
    int p0 = point_order_next_[pointid];
    int p1 = point_order_prev_[pointid];
    Eigen::Vector2d tan = (points_[p0] - points_[p1]).normalized();
    Eigen::Vector2d check_dir{-tan[1], tan[0]};
    // If nearby edges are closer then then check_len, there's no guarantee of
    // correctness!
    double impact_dist = distance_till_impact(pointid, check_dir);
    double check_len = 0.5 * std::min(1.0, impact_dist);
    bool is_left_in, is_right_in;
    do {
        // Ensure that the point is one-sided.
        Eigen::Vector2d check_left = at + check_len * check_dir;
        Eigen::Vector2d check_right = at - check_len * check_dir;
        is_left_in = is_point_in(check_left);
        is_right_in = is_point_in(check_right);
        check_len *= 0.5;
    } while ((is_left_in == is_right_in) & (check_len > kREpsilon));
    if (is_left_in) {
        // If the left side is inward, keep the orientation
        return {p1, p0};
    } else {
        // If the right side is inward, swap the orientation
        return {p0, p1};
    }
}

bool CurveCollection::orient_curves() {
    /*
    Compute the next and previous point in a traversal of all curves.
    */
    bool is_valid_curve = true;
    // First initialize the orientation by the edge list.
    point_order_next_.resize(points_.size());
    point_order_prev_.resize(points_.size());
    for (auto foo = 0; foo < edges_.size(); foo++) {
        point_order_next_[edges_[foo][0]] = edges_[foo][1];
        point_order_prev_[edges_[foo][1]] = edges_[foo][0];
    }
    // Traverse the points and swap the incorrectly oriented edges.
    std::vector<bool> point_done(points_.size(), false);
    for (auto foo = 0; foo < points_.size(); foo++) {
        if (point_done[foo])
            continue;
        else {
            int basepoint = foo;
            basepoints_.push_back(basepoint);
            int p_was = basepoint;
            // Orient so that inside lies on the left of the curve.
            std::array<int, 2> orient = neighborhood_orientation(basepoint);
            point_order_prev_[basepoint] = orient[0];
            point_order_next_[basepoint] = orient[1];
            int p_at = point_order_next_[basepoint];
            point_done[basepoint] = true;
            // Flow around the connected component, correcting the direction.
            while (p_at != basepoint) {
                if (point_order_prev_[p_at] == p_was) {
                    point_done[p_at] = true;
                    p_was = p_at;
                    p_at = point_order_next_[p_at];
                } else {
                    // correct the next point.
                    int true_next = point_order_prev_[p_at];
                    point_order_next_[p_at] = true_next;
                    point_order_prev_[p_at] = p_was;
                    point_done[p_at] = true;
                    // increment
                    p_was = p_at;
                    p_at = true_next;
                }
            }
        }
    }
    // Now that the edges are oriented we populate their order and point-edge lookups.
    edges_of_points_.resize(points_.size());
    for (auto edge_foo=0; edge_foo<edges_.size(); edge_foo++){
        int p0 = edges_[edge_foo][0];
        int p1 = edges_[edge_foo][1];
        edges_of_points_[p0][1] = edge_foo;
        edges_of_points_[p1][0] = edge_foo;
    }
    // Populate the edge ordering
    edge_order_next_.resize(edges_.size());
    edge_order_prev_.resize(edges_.size());
    for (int edge_foo=0; edge_foo<edges_.size(); edge_foo++){
        int p1 = edges_[edge_foo][1];
        int enext = edges_of_points_[p1][1];
        edge_order_next_[edge_foo] = enext;
        int p0 = edges_[edge_foo][0];
        int eprev = edges_of_points_[p0][0];
        edge_order_prev_[edge_foo] = eprev;
    }
    return is_valid_curve;
}

void CurveCollection::compute_tangents() {
    tangents_.resize(points_.size());
    for (int foo = 0; foo < points_.size(); foo++) {
        Eigen::Vector2d pnext = points_[point_order_next_[foo]];
        Eigen::Vector2d pprev = points_[point_order_prev_[foo]];
        tangents_[foo] = (pnext - pprev).normalized();
    }
}

void CurveCollection::compute_normals() {
    compute_tangents();
    normals_.resize(points_.size());
    for (int foo = 0; foo < points_.size(); foo++) {
        Eigen::Vector2d normal{tangents_[foo][1], -tangents_[foo][0]};
        normals_[foo] = normal;
    }
    is_normals_computed_ = true;
}


bool CurveCollection::setGridPointsAndCells(const std::vector<std::array<double,3>> &grid_points, const std::vector<std::vector<int>> &one_cells) {
	// Copy the edges
	int p_cnt = 0;
	std::unordered_map<int, int> p_alias;
	p_alias.reserve(edges_.size());
	edges_.resize(one_cells.size());
	for (auto foo = 0; foo < one_cells.size(); foo++) {
		if (one_cells[foo].size() != 2) {
			is_valid_ = false;
			return is_valid_;
		}
		for (int ii : {0, 1}) {
			int pid = one_cells[foo][ii];
			if (p_alias.find(pid) == p_alias.end()) {
				p_alias[pid] = p_cnt;
				p_cnt++;
			}
		}
        edges_[foo] = {p_alias[one_cells[foo][0]], p_alias[one_cells[foo][1]]};
    }
	// Size allocation
	points_.resize(p_alias.size());
	// Copy the good points
	for (const auto& foo : p_alias) {
		points_[foo.second] = { grid_points[foo.first][0], grid_points[foo.first][1]};
	}
	// Probably should not proceed if curves are invalid.
    if (is_valid_) {
        is_valid_ = orient_curves();
    }
    return is_valid_;
}

//void CurveCollection::write_to_vtp(const std::string outputfilename) {
//    /* Debugging output writer for evaluating the point normals. */
//    update_polydata();
//    auto writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
//    writer->SetFileName(outputfilename.c_str());
//    writer->SetInputData(polydata_);
//    writer->Update();
//};

int CurveCollection::get_number_of_points() const {
    /*Point access*/
    return points_.size();
}
int CurveCollection::get_number_of_edges() const
{
	return edges_.size();
}
;

Eigen::Vector2d CurveCollection::get_point(int p_id) const {
    /*Point access*/
    return points_[p_id];
};

Eigen::Vector2d CurveCollection::get_normal(int p_id) {
    /*Normal access*/
    if (!is_normals_computed_) compute_normals();
    return normals_[p_id];
};

int CurveCollection::get_next_edge(const int e_id) const {
    return edge_order_next_[e_id];
}

int CurveCollection::get_prev_edge(const int e_id) const {
    return edge_order_prev_[e_id];
}

int CurveCollection::get_next_point(const int p_id) const {
    return edge_order_next_[p_id];
}

int CurveCollection::get_prev_point(const int p_id) const {
    return edge_order_prev_[p_id];
}


std::array<int,2> CurveCollection::get_points_of_edge(const int e_id) const {
    return edges_[e_id];
}

int CurveCollection::get_number_of_components() const {
    return basepoints_.size();
};

int CurveCollection::get_basepoint(int x) const {
    return basepoints_[x];
};

std::vector<PointList> CurveCollection::getPointCycles() const {
	std::vector<PointList> cycles;
	cycles.reserve(get_number_of_components());
	for (auto bp : basepoints_) {
		PointList cycle;
		cycle.reserve(get_number_of_points());
		int at = bp;
		do {
			cycle.push_back(get_point(at));
			at = get_next_point(at);
		} while (at != bp);
		cycle.push_back(get_point(bp));
		cycles.push_back(cycle);
	}
	return cycles;
};

bool CurveCollection::setPointCycles(const  std::vector<PointList>& cycles){
	//
	int point_count = points_.size();
	int cnt;
	for (const auto & cycle : cycles) {
		int basepoint = point_count;
		cnt = point_count;
		point_count += (cycle.size() - 1);
		points_.reserve(point_count);
		points_.insert(std::end(points_), std::begin(cycle), std::end(cycle)-1);
		//Write in all the edges in the cycle.
		edges_.reserve(point_count);
		while (cnt < point_count-1) {
			edges_.push_back({cnt, cnt+1});
			cnt++;
		}
		edges_.push_back({cnt,basepoint});
	}
	bool is_valid = orient_curves();
	//!!!!
	int pat = basepoints_[0];
	std::cout << "Vacancy became: " << std::endl;
	do {
		std::cout << get_point(pat).transpose() << std::endl;
		pat = get_next_point(pat);
	} while (pat != basepoints_[0]);

	//!!!!
	return is_valid;
};

void CurveCollection::clear() {
	points_.clear();
	edges_.clear();
	point_order_next_.clear();
	point_order_prev_.clear();
	edges_of_points_.clear();
	edge_order_next_.clear();
	edge_order_prev_.clear();
	basepoints_.clear();
	tangents_.clear();
	normals_.clear();
};
