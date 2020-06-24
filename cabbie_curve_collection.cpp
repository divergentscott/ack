#include <deque>

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

std::string _debug_seg_rel_print(SegmentRelation x) {
	if (x == SegmentRelation::kDisjoint) return "disjoint";
	if (x == SegmentRelation::kSubsegment) return "subsegment";
	if (x == SegmentRelation::kSupersegment) return "supersegment";
	if (x == SegmentRelation::kDirectStagger) return "direct_stag";
	if (x == SegmentRelation::kReverseStagger) return "rev_stag";
	return "error";
};


SegmentRelation edgeIntersection(const std::array<Eigen::Vector2d, 2>& a, const std::array<Eigen::Vector2d, 2>& b, const bool is_horizontal, std::array<Eigen::Vector2d, 2>& isect) {
	int dim = is_horizontal ? 0 : 1;
	int offdim = is_horizontal ? 1 : 0;
	if (std::abs(a[0][offdim] - b[0][offdim]) > repsilon) return SegmentRelation::kDisjoint;
	//So a and b have sufficiently close y.

	if (b[0][dim] <= a[0][dim]) {
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
	}
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
};

Cedge edgify(const Eigen::Vector2d& a, const Eigen::Vector2d& b, bool& is_hedge, bool& is_forward){
    if (std::abs(a[1] - b[1]) < repsilon){
        is_hedge = true;
        is_forward = (a[0]<b[0]);
        if (is_forward) return {a,b};
        return {b,a};
    }
    if (std::abs(a[0] - b[0]) < repsilon){
        is_hedge = false;
        is_forward = (a[1]<b[1]);
        if (is_forward) return {a,b};
        return {b,a};
    }
    std::cout << "WARNING: noncardinal edge!" << std::endl;
    return {a,b};
}


void addEdgetoPointlist(PointList & point_list, std::array<Eigen::Vector2d,2> x){
    if (point_list.size()>0){
        point_list = {x[0], x[1]};
        return;
    }
    if ((x[0] - *(point_list.end()-1)).norm() < repsilon){
        point_list.push_back(x[1]);
        return;
    }
    if ((x[1] - *(point_list.end()-1)).norm() < repsilon){
        point_list.push_back(x[0]);
        return;
    }
    std::cout << "WARNING! Edge does not connect to point list." << std::endl;
}

struct ChainForge {
	// For collecting line segments with shared endpoints and organizing them into paths
	// These are expected to be fairly few chains, if larger then we need a more sophisticated spatial search structure
	// Already kind of a lot of copying
	std::vector<std::deque<Eigen::Vector2d>> chains_;
	void addLink(Cedge x){
		bool is_matched = false;
		for (auto& chain : chains_) {
			if ((chain.back() - x[0]).norm() < repsilon) {
				chain.push_back(x[1]);
				is_matched = true;
				break;
			}
			if ((chain.back() - x[1]).norm() < repsilon) {
				chain.push_back(x[0]);
				is_matched = true;
				break;
			} 
			if ((chain.front() - x[0]).norm() < repsilon) {
				chain.push_front(x[1]);
				is_matched = true;
				break;
			}
			if ((chain.front() - x[1]).norm() < repsilon) {
				chain.push_front(x[0]);
				is_matched = true;
				break;
			} 
		}
		if (!is_matched) {
			chains_.push_back({x[0],x[1]});
		}
	};
	template<class TIteratable>
	void addChain(TIteratable x){
		bool is_matched = false;
		for (auto& chain : chains_) {
			if ((chain.back() - x.front()).norm() < repsilon) {
				for (int foo = 1; foo < x.size(); foo++) chain.push_back(x[foo]);
				is_matched = true;
				break;
			}
			if ((chain.back() - x.back()).norm() < repsilon) {
				for (int foo = x.size()-2; foo >=0; foo--) chain.push_back(x[foo]);
				is_matched = true;
				break;
			}
			if ((chain.front() - x.front()).norm() < repsilon) {
				for (int foo = 1; foo < x.size(); foo++) chain.push_front(x[foo]);
				is_matched = true;
				break;
			}
			if ((chain.front() - x.back()).norm() < repsilon) {
				for (int foo = x.size() - 2; foo >= 0; foo--) chain.push_front(x[foo]);
				is_matched = true;
				break;
			}
		}
		if (!is_matched) {
			std::deque<Eigen::Vector2d> chain;
			for (int foo = 0; foo < x.size(); foo++) chain.push_back(x[foo]);
			chains_.push_back(chain);
		}
	};
	void relink() {
		//This is EXTREMELY inefficient.
		//Speed improve by preventing
		bool is_continuing = true;
		while (is_continuing) {
			ChainForge reforge;
			for (auto &chain : chains_) {
				reforge.addChain(chain);
			}
			if (chains_.size() == reforge.chains_.size()) is_continuing = false;
			chains_ = reforge.chains_;
		}
	}
	std::vector<PointList> getPointList() {
		std::vector<PointList> pls;
		for (auto& chain : chains_) pls.push_back({chain.begin(), chain.end()});
		return pls;
	}
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
	std::array<std::vector<Cedge>,4> hits;
	//
	// PART 1: SHATTER THE INCOMING CHAINS
	//
	// Search for overlapping linesegments and trim the chains.
	// Pieces that intersect the rectangle are recorded for deletion from the rectangle,
	// Otherwise the get dumped into the chain forge
	ChainForge chain_forge;
	for(const auto& chain : chains){
        for (int foo=0; foo < chain.size() - 1 ; foo++){
            bool is_hedge;
            bool is_forward;
			Cedge edge = edgify(chain[foo], chain[foo + 1], is_hedge, is_forward);
            //check for intersection against all the rectangle edges
            SegmentRelation segrel;
            std::array<Eigen::Vector2d, 2> isect;
			for (int rectsideii = 0; rectsideii < 4; rectsideii++) {
				auto rectedge = rect_sides[rectsideii];
				bool is_rect_horizontal = (rectsideii == 1) | (rectsideii == 3);
				if (is_hedge == is_rect_horizontal) {
					segrel = edgeIntersection(edge, rectedge, is_rect_horizontal, isect);
//                    std::cout << rectsideii << " rect edge " << _debug_edge_print(rectedge) << " horizontal " << is_rect_horizontal << std::endl;
//                    std::cout << "edge " << _debug_edge_print(edge) << " horizontal " << is_hedge << std::endl;
//                    std::cout << "isect " << _debug_seg_rel_print(segrel) << " " << _debug_edge_print(isect) << std::endl;
                    if (segrel != SegmentRelation::kDisjoint){
						hits[rectsideii].push_back(isect);
                        break;
                    }
                }
            }
			//
            if (segrel == SegmentRelation::kDisjoint) {
				chain_forge.addLink(edge);
            } else {
				if ((segrel == SegmentRelation::kSupersegment) | (segrel == SegmentRelation::kDirectStagger)) {
					chain_forge.addLink({edge[0],isect[0]});
				}
                if ((segrel == SegmentRelation::kSupersegment) | (segrel == SegmentRelation::kReverseStagger)) {
					chain_forge.addLink({isect[1], edge[1]});
                }
            }
        }
    }
	
    std::cout << "Trimmed chains:" << std::endl;
    for (auto chain : chain_forge.chains_){
        std::cout << " new chain " << std::endl;
        for (auto p : chain){
            std::cout << p.transpose() << std::endl;
        }
    }

    //
	//
	// PART 2: SHATTER THE RECTANGLE
	//
	//order hits by going around the rectangle clockwise from west
	std::sort(hits[0].begin(), hits[0].end(), [](const Vedge& a, const Vedge& b) {return a[0][1] < b[0][1];});
	std::sort(hits[1].begin(), hits[1].end(), [](const Hedge& a, const Hedge& b) {return a[0][0] < b[0][0];});
	std::sort(hits[2].begin(), hits[2].end(), [](const Vedge& a, const Vedge& b) {return a[0][1] > b[0][1];});
	std::sort(hits[3].begin(), hits[3].end(), [](const Hedge& a, const Hedge& b) {return a[0][0] > b[0][0];});

	std::cout << "Rectangle hits are:" << std::endl;
	for (int foo = 0; foo < 4; foo++) {
		std::cout << "Rectangle side " << foo << std::endl;
		for (auto hit : hits[foo]) {
			std::cout << _debug_edge_print(hit) << std::endl;
		}
	}
	//Rect_segs has the connected components of the rectangle boundary - intersecting curves 
	//Now we reorganize the rectangle boundary that hasn't been removed into segments
	PointList rect_remnant;
	////std::array<int, 2> remnant_end_id;
	////std::array<bool, 2> remnant_end_direct;
	//
	int start = -1;
	int start_lead_index = 0;
    for (int foo=0; foo<4; foo++){
        if ((hits[foo].size() > 0) & (start < 0)){
            //Find the first impact
            start = foo;
            foo=0;
			start_lead_index = 1 - start / 2;
            rect_remnant.push_back(hits[start][0][ start_lead_index ]);
			//list all the segments on start edge
			for (int bar=1; bar < hits[start].size(); bar++){
                rect_remnant.push_back(hits[start][bar][1-start_lead_index]);
				//
				chain_forge.addChain(rect_remnant);
				//
                rect_remnant = {hits[start][bar][start_lead_index]};
			}
			//See if the you need the corner
			if ( (rect_corners[start] - rect_remnant.back()).norm() > repsilon) {
				rect_remnant.push_back(rect_corners[start]);
			}
        } else {
            //traverse around the other edges and write down segments
            int at = (foo + start) % 4;
			int at_lead_index = 1 - at / 2;
			std::cout << "considered edge " << at << std::endl;
			for (int bar = 0; bar < hits[at].size(); bar++) {
				rect_remnant.push_back(hits[at][bar][1-at_lead_index]);
				//
				chain_forge.addChain(rect_remnant);
				//
				rect_remnant = { hits[at][bar][at_lead_index] };
			}
			//See if the you need the corner
			if ((rect_corners[at] - *(rect_remnant.end() - 1)).norm() > repsilon) {
				rect_remnant.push_back(rect_corners[at]);
			}
        }
    }
	rect_remnant.push_back(hits[start][0][1-start_lead_index]);
	//
	chain_forge.addChain(rect_remnant);

	std::cout << "After rectangle:" << std::endl;
	for (auto chain : chain_forge.chains_) {
		std::cout << " new chain " << std::endl;
		for (auto p : chain) {
			std::cout << p.transpose() << std::endl;
		}
	}
	//
	// PART 3: REFORGE THE CHAIN PIECES TOGETHER WITH THE RECTANGLE PIECE
	//
	chain_forge.relink();
	
	std::cout << "After relinking:" << std::endl;
	for (auto chain : chain_forge.chains_) {
		std::cout << " new chain " << std::endl;
		for (auto p : chain) {
			std::cout << p.transpose() << std::endl;
		}
	}

	return chain_forge.getPointList();
};


std::string _debug_edge_print(std::array<Eigen::Vector2d,2> x){
    std::stringstream ss;
    ss << "("<< x[0].transpose() << " -> " << x[1].transpose() <<")";
    return ss.str();
};
