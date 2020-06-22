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

bool hedgeIntersection(const Hedge& a, const Hedge& b, Hedge& isect){
    if ( std::abs(a[0][1] - b[0][1]) > repsilon ) return false;
    //So a and b have sufficiently close y.
    if (a[0][0] <= b[0][0]) {
        if (a[1][0] < b[0][0]) return false;
        //So b[0][0] <= a[1][0]
        if (b[1][0] <= a[1][0]){
            //SegRelation::kSupersegment;
            isect = b;
            return true;
        }
        //So a[1][0] < b[1][0]
        //SegRelation::kDirectStack;
        isect = {b[0],a[1]};
        return true;
    }
    // So b[0][0]<a[0][0]
    if (b[1][0] < a[0][0]){
        //SegRelation::kDisjoint;
        return false;
    }
    //
    if (a[1][0] <= b[1][0]){
        //SegRelation::kSubsegment;
        isect = a;
        return true;
    }
    //So a[1][0] < b[1][0]
    //SegRelation::kReverseStack
    isect = {a[0],b[1]};
    return true;
};

bool vedgeIntersection(const Vedge& a, const Vedge& b, Vedge& isect){
    if ( std::abs(a[0][0] - b[0][0]) > repsilon ) return false;
    //So a and b have sufficiently close y.
    if (a[0][1] <= b[0][1]) {
        if (a[1][1] < b[0][1]) return false;
        //So b[0][0] <= a[1][0]
        if (b[1][1] <= a[1][1]){
            //SegRelation::kSupersegment;
            isect = b;
            return true;
        }
        //So a[1][0] < b[1][0]
        //SegRelation::kDirectStack;
        isect = {b[0],a[1]};
        return true;
    }
    // So b[0][0]<a[0][0]
    if (b[1][1] < a[0][1]){
        //SegRelation::kDisjoint;
        return false;
    }
    //
    if (a[1][1] <= b[1][1]){
        //SegRelation::kSubsegment;
        isect = a;
        return true;
    }
    //So a[1][0] < b[1][0]
    //SegRelation::kReverseStack
    isect = {a[0],b[1]};
    return true;
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

std::vector<PointList> addRectangleModZ2(std::vector<PointList> &chains, const Eigen::Vector2d &position, const double &width, const double &height){
    std::vector<PointList> chainsout;
    //
    Eigen::Vector2d se = {position[0]+width, position[1]};
    Eigen::Vector2d ne = {position[0]+width, position[1]+height};
    Eigen::Vector2d nw = {position[0], position[1]+height};
    std::vector<Eigen::Vector2d> corners = {nw, ne, se, position};
    //
    Hedge rect_s = {position,se};
    Hedge rect_n = {nw,ne};
    Vedge rect_w = {position,nw};
    Vedge rect_e = {se,ne};
    //
    std::vector<Hedge> hits_w;
    std::vector<Hedge> hits_e;
    std::vector<Vedge> hits_n;
    std::vector<Vedge> hits_s;
    
    for(const auto& chain : chains){
        for (int foo=0; foo < chain.size() - 1 ; foo++){
            bool is_hedge;
            auto edge = edgify(chain[foo], chain[foo+1], is_hedge);
            if (is_hedge){
                Hedge isect;
                bool is_isect = hedgeIntersection(edge, rect_s, isect);
                if (is_isect) hits_s.push_back(isect);
                is_isect = hedgeIntersection(edge, rect_n, isect);
                if (is_isect) hits_n.push_back(isect);
            } else {
                Vedge isect;
                bool is_isect = vedgeIntersection(edge, rect_w, isect);
                if (is_isect) hits_w.push_back(isect);
                is_isect = vedgeIntersection(edge, rect_e, isect);
                if (is_isect) hits_e.push_back(isect);
            }
        }
    }
    
    std::sort(hits_n.begin(), hits_n.end(), [](const Hedge& a, const Hedge&b){return a[0][0] < b[0][0];});
    std::sort(hits_s.begin(), hits_s.end(), [](const Hedge& a, const Hedge&b){return a[0][0] > b[0][0];});
    std::sort(hits_w.begin(), hits_w.end(), [](const Hedge& a, const Hedge&b){return a[0][1] < b[0][1];});
    std::sort(hits_e.begin(), hits_e.end(), [](const Hedge& a, const Hedge&b){return a[0][1] > b[0][1];});
    
    std::vector<std::vector<std::array<Eigen::Vector2d,2>>&> hits = {&hits_w, &hits_n, &hits_e, &hits_s};
    
    std::vector<std::vector<Eigen::Vector2d>> rect_segs;
    std::vector<Eigen::Vector2d> current_seg;
    
    int start = -1;
    for (int foo=0; foo<4; foo++){
        if ((hits[foo].size() > 0) & (start < 0)){
            //Find the first impact
            start = foo;
            foo=0;
            current_seg.push_back(hits[start][0][1]);
            for (int bar=1; bar < hits[start].size(); bar++){
                current_seg.push_back(hits[start][bar][0]);
                rect_segs.push_back(current_seg);
                current_seg = {hits[start][bar][1]};
            }
        } else {
            //traverse around and find the
            int at = (foo + start) % 4;
            
        }
    }
    
    
    
    
    std::vector<Eigen::Vector2d> remnants;
    
    if (hits_w.size()>0){
        
    }
    
};
