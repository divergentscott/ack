#include "cabbie_curve_collection.h"

bool rayTraceNorth(const Eigen::Vector2d &origin, const Hedge &segment, Eigen::Vector2d &impact){
    //Returns true if there is an impact, otherwise false.
    //If true, impact_location is populated with the impact point on the line segment.
    //Assumes segment is horizontal AND ordered with segment[0][0] < segment[1][0] and segment[0][1] == semgent[1][1]
    if ((origin[1] > segment[0][1]) | (segment[0][0] > origin[0]) | (origin[0] > segment[1][0])) return false;
    impact[0] = origin[0];
    impact[1] = segment[0][1];
    return true;
}


bool rayTraceEast(const Eigen::Vector2d &origin, const Vedge &segment, Eigen::Vector2d &impact){
    //Returns true if there is an impact, otherwise false.
    //If true, impact_location is populated with the impact point on the line segment.
    //Assumes segment is vertical AND ordered with segment[0][1] < segment[1][1] and segment[0][0] == semgent[1][0]
    if ((origin[0] > segment[0][0]) | (segment[0][1] > origin[1]) | (origin[1] > segment[1][1])) return false;
    impact[1] = origin[1];
    impact[0] = segment[0][0];
    return true;
};

Vedge CabbiePath::getWall(const int &east_index) const {
    auto a = points_[2*east_index+1];
    auto b = points_[2*east_index+2];
    if (a[1] < b[1]) return {a,b};
    return {b,a};
};

Hedge CabbiePath::getPlateau(const int &east_index) const {
    auto a = points_[2*east_index];
    auto b = points_[2*east_index+1];
    if (a[0] < b[0]) return {a,b};
    return {b,a};
};

void CabbiePath::copyPointList(const std::list<Eigen::Vector2d> &x){
    if (x.size()>0){
        points_.resize(x.size());
        num_walls_ = (x.size()-1)/2;
        num_plateaus_ = (x.size())/2;
        std::list<Eigen::Vector2d>::const_iterator xit = x.cbegin();
        for (int foo=0; foo<x.size(); foo++){
            auto p0 = *xit;
            points_[foo] = p0;
            xit++;
        }
    }

};

void CabbiePath::push_back(const Eigen::Vector2d &x){
    points_.push_back(x);
    num_walls_ = (points_.size()-1)/2;
    num_plateaus_ = points_.size()/2;
};

void CabbiePath::resize(const std::vector<Eigen::Vector2d>::size_type &x){
    points_.resize(x);
    num_walls_ = (points_.size()-1)/2;
    num_plateaus_ = points_.size()/2;
};

//enum class SegmentRelation {
//	kDisjoint,
//	kSubsegment, // a contained by b
//	kSupersegment, // a contains b
//	kDirectStagger, // abab
//	kReverseStagger // baba
//};

SegmentRelation edgeIntersection(const std::array<Eigen::Vector2d,2>& a, const std::array<Eigen::Vector2d,2>& b, const bool is_horizontal, std::array<Eigen::Vector2d, 2>& isect) {
	int dim = is_horizontal ? 0 : 1;
	int offdim = is_horizontal ? 1 : 0;
	if (std::abs(a[0][offdim] - b[0][offdim]) > repsilon) return SegmentRelation::kDisjoint;
	//So a and b have sufficiently close y.
	if (a[0][dim] <= b[0][dim]) {
		if (a[1][dim] < b[0][dim]) return SegmentRelation::kDisjoint;
		// pattern aa bb
		if (b[1][dim] <= a[1][dim]) {
			//pattern abba
			//SegRelation::kSupersegment;
			isect = b;
			return SegmentRelation::kSupersegment;
		}
		// pattern abab
		//SegRelation::kDirectStack;
		isect = { b[0],a[1] };
		return SegmentRelation::kDirectStagger;
	}
	// So b[0][0]<a[0][0]
	if (b[1][dim] < a[0][dim]) {
		//pattern bb aa
		//SegRelation::kDisjoint;
		return SegmentRelation::kDisjoint;
	}
	//
	if (a[1][dim] <= b[1][dim]) {
		//SegRelation::kSubsegment;
		//pattern baab
		isect = a;
		return SegmentRelation::kSubsegment;
	}
	//SegRelation::kReverseStack
	//pattern baba
	isect = { a[0], b[1] };
	return SegmentRelation::kReverseStagger;
};

std::array<Eigen::Vector2d,2> edgify(const Eigen::Vector2d& a, const Eigen::Vector2d& b, bool& is_hedge){
    if (std::abs(a[1] - b[1]) < repsilon){
        is_hedge = true;
        if (a[0]<b[0]) return {a,b};
        return {b,a};
    }
    if (std::abs(a[0] - b[0]) < repsilon){
        is_hedge = false;
        if (a[1] < b[1]) return {a,b};
        return {b,a};
    }
    std::cout << "WARNING: noncardinal edge!" << std::endl;
    return {a,b};
}

struct ChainHit {
	std::array<Eigen::Vector2d,2> segment;
	int id0;
	bool is_front0;
	int id1;
	bool is_front1;
};

std::vector<PointList> addRectangleModZ2(std::vector<PointList> &chains, const Eigen::Vector2d &position, const double &width, const double &height){
    //
    Eigen::Vector2d se = {position[0]+width, position[1]};
    Eigen::Vector2d ne = {position[0]+width, position[1]+height};
    Eigen::Vector2d nw = {position[0], position[1]+height};
    std::array<Eigen::Vector2d,4> rect_corners = {nw, ne, se, position};
    //
    Hedge rect_s = {position,se};
    Hedge rect_n = {nw,ne};
	Vedge rect_w = {position,nw};
    Vedge rect_e = {se,ne};
	std::array<std::array<Eigen::Vector2d, 2>,4> rect_sides = { rect_w, rect_n, rect_e, rect_s };
    //
	std::array<std::vector<ChainHit>,4> hits;
	//
	// PART 1: SHATTER THE INCOMING CHAINS
	//
	//Search for overlapping linesegments and trim the chains by deleting all contacts with the rectangle
	std::vector<PointList>	trimmed_chains;
	PointList trim_chain;
	for(const auto& chain : chains){
        for (int foo=0; foo < chain.size() - 1 ; foo++){
            bool is_hedge;
            auto edge = edgify(chain[foo], chain[foo+1], is_hedge);
			for (int rectsideii = 0; rectsideii < 4; rectsideii++) {
				auto rectedge = rect_sides[rectsideii];
				bool is_rect_horizontal = ((rectsideii) == 1) | ((rectsideii) == 3);
				if (is_hedge == is_rect_horizontal) {
					std::array<Eigen::Vector2d, 2> isect;
					SegmentRelation segrel = edgeIntersection(rectedge, edge, is_rect_horizontal, isect);
					if (segrel == SegmentRelation::kDisjoint) {
						trim_chain.push_back(chain[foo]);
					}
					else {
						if ((segrel == SegmentRelation::kSupersegment) | (segrel == SegmentRelation::kReverseStagger)) {
							trim_chain.push_back(chain[foo]);
						}
						trim_chain.push_back(isect[0]);
						trimmed_chains.push_back(trim_chain);
						trim_chain = { isect[1] };
						if ((segrel == SegmentRelation::kSupersegment) | (segrel == SegmentRelation::kDirectStagger)) {
							trim_chain.push_back(chain[foo + 1]);
						}
						//Record the intersection for relinking purposes
						ChainHit ch;
						ch.segment = isect;
						ch.id0 = trimmed_chains.size() - 1;
						ch.is_front0 = false;
						ch.id1 = trimmed_chains.size();
						ch.is_front1 = true;
					}
				}
			}
        }
		trim_chain.push_back(*(chain.end()-1));
		trimmed_chains.push_back(trim_chain);
		trim_chain = {};
    }
    //
	//
	// PART 2: SHATTER THE RECTANGLE
	//
	//order hits by going around the rectangle clockwise from west
	std::sort(hits[0].begin(), hits[0].end(), [](const ChainHit& a, const ChainHit&b) {return a.segment[0][1] < b.segment[0][1];});
	std::sort(hits[1].begin(), hits[1].end(), [](const ChainHit& a, const ChainHit&b) {return a.segment[0][0] < b.segment[0][0];});
	std::sort(hits[2].begin(), hits[2].end(), [](const ChainHit& a, const ChainHit&b) {return a.segment[0][1] > b.segment[0][1];});
	std::sort(hits[3].begin(), hits[3].end(), [](const ChainHit& a, const ChainHit&b) {return a.segment[0][0] > b.segment[0][0];});
    //
    //
	//Rect_segs has the connected components of the rectangle boundary - intersecting curves 
    std::vector<PointList> rectangle_remnants;
	std::vector<std::array<int,2>> remnant_end_ids;
	std::vector<std::array<bool,2>> remnant_end_directions;
	//
	//Now we reorganize the rectangle boundary that hasn't been removed into segments
	PointList rect_remnant;
	std::array<int, 2> remnant_end_id;
	std::array<bool, 2> remnant_end_direct;
	//
	int start = -1;
    for (int foo=0; foo<4; foo++){
        if ((hits[foo].size() > 0) & (start < 0)){
            //Find the first impact
            start = foo;
            foo=0;
            rect_remnant.push_back(hits[start][0].segment[1]);
			remnant_end_id = { hits[start][0].id1, -1 };
			remnant_end_direct[0] = hits[start][0].is_front1;
			//list all the segments on start edge
			for (int bar=1; bar < hits[start].size(); bar++){
                rect_remnant.push_back(hits[start][bar].segment[0]);
				remnant_end_id[1] = hits[start][bar].id0;
				remnant_end_direct[1] = hits[start][bar].is_front0;
				//
                rectangle_remnants.push_back(rect_remnant);
				remnant_end_ids.push_back(remnant_end_id);
				remnant_end_directions.push_back(remnant_end_direct);
				//
				//remnant_end_id.push_back();
                rect_remnant = {hits[start][bar].segment[1]};
				remnant_end_id = { hits[start][bar].id1 , -1};
				remnant_end_direct[0] = hits[start][bar].is_front1;
			}
			//See if the you need the corner
			if ( (rect_corners[start] - *(rect_remnant.end()-1) ).norm() < repsilon) {
				rect_remnant.push_back(rect_corners[start]);
			}
        } else {
            //traverse around the other edges and write down segments
            int at = (foo + start) % 4;
			for (int bar = 0; bar < hits[at].size(); bar++) {
				rect_remnant.push_back(hits[at][bar].segment[0]);	
				remnant_end_id[1] = hits[at][bar].id0;
				remnant_end_direct[1] = hits[at][bar].is_front0;
				//
				rectangle_remnants.push_back(rect_remnant);
				remnant_end_ids.push_back(remnant_end_id);
				remnant_end_directions.push_back(remnant_end_direct);
				//
				rect_remnant = { hits[at][bar].segment[1] };
				remnant_end_id = { hits[at][bar].id1, -1 };
				remnant_end_direct[0] = hits[at][bar].is_front1;
			}
			//See if the you need the corner
			if ((rect_corners[start] - *(rect_remnant.end() - 1)).norm() < repsilon) {
				rect_remnant.push_back(rect_corners[start]);
			}
        }
    }
	rect_remnant.push_back(hits[start][0].segment[0]);
	remnant_end_id[1] = hits[start][0].id0;
	remnant_end_direct[1] = hits[start][0].is_front0;
	//
	rectangle_remnants.push_back(rect_remnant);
	remnant_end_ids.push_back(remnant_end_id);
	remnant_end_directions.push_back(remnant_end_direct);
	//
	// PART 3: REFORGE THE CHAIN PIECES TOGETHER WITH THE RECTANGLE PIECE
	//
	std::vector<PointList> chainsout;

	return chainsout;
};
