#include "cabby_curve_collection.h"

bool rayTraceNorth(const Eigen::Vector2d &origin, const hedge &segment, Eigen::Vector2d &impact){
    //Returns true if there is an impact, otherwise false.
    //If true, impact_location is populated with the impact point on the line segment.
    //Assumes segment is horizontal AND ordered with segment[0][0] < segment[1][0] and segment[0][1] == semgent[1][1]
    if ((origin[1] > segment[0][1]) | (segment[0][0] > origin[0]) | (origin[0] > segment[1][0])) return false;
    impact[0] = origin[0];
    impact[1] = segment[0][1];
    return true;
}


bool rayTraceEast(const Eigen::Vector2d &origin, const vedge &segment, Eigen::Vector2d &impact){
    //Returns true if there is an impact, otherwise false.
    //If true, impact_location is populated with the impact point on the line segment.
    //Assumes segment is vertical AND ordered with segment[0][1] < segment[1][1] and segment[0][0] == semgent[1][0]
    if ((origin[0] > segment[0][0]) | (segment[0][1] > origin[1]) | (origin[1] > segment[1][1])) return false;
    impact[1] = origin[1];
    impact[0] = segment[0][0];
    return true;
};

vedge Cabbie::getWall(const int &east_index) const {
    auto a = points_[2*east_index+1];
    auto b = points_[2*east_index+2];
    if (a[1] < b[1]) return {a,b};
    return {b,a};
};

hedge Cabbie::getPlateau(const int &east_index) const {
    auto a = points_[2*east_index];
    auto b = points_[2*east_index+1];
    if (a[0] < b[0]) return {a,b};
    return {b,a};
};

void Cabbie::copyPointList(const std::list<Eigen::Vector2d> &x){
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

void Cabbie::push_back(const Eigen::Vector2d &x){
    points_.push_back(x);
    num_walls_ = (points_.size()-1)/2;
    num_plateaus_ = points_.size()/2;
};

void Cabbie::resize(const std::vector<Eigen::Vector2d>::size_type &x){
    points_.resize(x);
    num_walls_ = (points_.size()-1)/2;
    num_plateaus_ = points_.size()/2;
};
