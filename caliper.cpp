//
//  caliper.cpp
//  ack
//
//  Created by Shane Scott on 6/10/20.
//

#include "caliper.h"

//vedge sortVedge(const Eigen::Vector2d &a, const Eigen::Vector2d &b){
//    //Remove once stable:
//    if (std::abs(a[0]-b[0])>repsilon){
//        std::cout << "warning! bad vedgies!" << a.transpose() << " and " << b.transpose();
//    }
//    if (a[1] < b[1]) return {a,b};
//    return {b,a};
//};
//
//CaliperHiker::CaliperHiker(double width, Trail trail){
//    width_ = width;
//    //Construct the valley walls
//    if (trail.landmarks_valley.size()>1){
//        landmarks_valley_.resize(trail.landmarks_valley.size());
//        num_walls_ = (landmarks_valley_.size()-1)/2;
//        std::list<Eigen::Vector2d>::iterator trit = trail.landmarks_valley.begin();
//        for (int foo=0; foo<landmarks_valley_.size(); foo++){
//            auto p0 = *trit;
//            landmarks_valley_[foo] = p0;
//            trit++;
//        }
//    }
//    //Copy the mountain walls
//        //!!!!
//};
//
//void CaliperHiker::eraseBadCamps(){};
//
//void CaliperHiker::hikeTrailValley(){
//    position_ = landmarks_valley_[0] - Eigen::Vector2d({width_,0});
//    vedge obs = getWall(0);
//    rayTraceEast(position_, obs, obstruction_);
//    camps_valley_.push_back(position_);
//    headEastValley(0);
//    eraseBadCamps();
//}
//
//void CaliperHiker::headEastValley(int start_east){
//    int index_east = start_east;
//    while(start_east < num_walls_-1){
//        vedge wall = getWall(index_east);
//        double east_reach = wall[0][0];
//        double north_reach = wall[1][1];
//        while(east_reach < obstruction_[0] + width_){
//            // Try to slide the caliper straight east along the obstruction
//            // But head north if a wall gets in the way.
//            if (index_east == num_walls_-1){
//                break;
//            }else if (north_reach > position_[1]){
//                //There is a valley wall in the way of east
//                position_ = {east_reach - width_, obstruction_[1]};
//                camps_valley_.push_back(position_);
//                position_ = {east_reach - width_, north_reach};
//                camps_valley_.push_back(position_);
//                obstruct_que_.clear();
//                rayTraceEast(position_, getWall(index_east+1), obstruction_);
//                headEastValley(index_east+1);
//                return;
//            } else {
//                index_east++;
//                vedge wall = getWall(index_east);
//                east_reach = wall[0][0];
//                north_reach = wall[1][1];
//            }
//        }
//        // Head south to a new obstruction
//        position_ = obstruction_;
//        camps_valley_.push_back(position_);
//        std::deque<hedge> obstructions_in_view = scanValleyFloor(start_east, index_east);
//        mergeValleyQue(obstruct_que_, obstructions_in_view);
//        vedge topfloor = obstruct_que_[0];
//        obstruct_que_.pop_front();
//        position_ = {position_[0], topfloor[0][1]};
//        camps_valley_.push_back(position_);
//        start_east = index_east;
//        obstruction_ = topfloor[1];
//    }
//};
//
//std::deque<hedge> CaliperHiker::scanValleyFloor(int east_start, int east_end){
//    std::deque<hedge> obs_up;
//    hedge obs = getPlateau(east_end);
//    obs_up.push_front(obs);
//    for (int foo = east_end - 1; foo > east_start; foo--){
//        obs = getPlateau(foo);
//        if (obs[0][1] >= obs_up[0][0][1]) obs_up.push_front(obs);
//    }
//    return obs_up;
//};
//
//void CaliperHiker::mergeValleyQue(std::deque<hedge> &q0, std::deque<hedge> &q1){
//    bool is_an_empty = (q0.size()==0) || (q1.size()==0);
//    if (!is_an_empty){
//        double t1y = q1[0][0][1];
//        while(q0.back()[0][1] <= t1y){
//            q0.pop_back();
//            if (q0.size()==0) break;
//        }
//    }
//    for (int foo=0; foo<q1.size(); foo++){
//        q0.push_back(q1[foo]);
//    }
//};
//
//
//void CaliperHiker::hikeTrail(){
//    
//};


PlankHiker::PlankHiker(const Cabbie& landmarks, const double plank_width, const bool is_gravity_south) : landmarks_(landmarks), width_(plank_width), is_gravity_south_(is_gravity_south)
{
    vert_index_ = is_gravity_south_? 1 : 0;
};

void PlankHiker::hike(){
    position_ = landmarks_.points_[0] - Eigen::Vector2d({width_,0});
    vedge obs = landmarks_.getWall(0);
    rayTraceEast(position_, obs, obstruction_);
    camps_.push_back(position_);
    headEast(0);
//    eraseBadCamps();
}

bool PlankHiker::compareVertical(const double& x, const double &y) const {
    if (is_gravity_south_) return (x<y);
    return (x>y);
}

void PlankHiker::headEast(int start_east){
    int index_east = start_east;
    while(start_east < landmarks_.num_walls_-1){
        vedge wall = landmarks_.getWall(index_east);
        double east_reach = wall[0][0];
        double north_reach = wall[vert_index_][1];
        while(east_reach < obstruction_[0] + width_){
            // Try to slide the caliper straight east along the obstruction
            // But head north if a wall gets in the way.
            if (index_east == landmarks_.num_walls_-1){
                break;
            }else if (compareVertical(position_[1], north_reach)){
                //There is a valley wall in the way of east
                position_ = {east_reach - width_, obstruction_[1]};
                camps_.push_back(position_);
                position_ = {east_reach - width_, north_reach};
                camps_.push_back(position_);
                obstruct_que_.clear();
                rayTraceEast(position_, landmarks_.getWall(index_east+1), obstruction_);
                headEast(index_east+1);
                return;
            } else {
                index_east++;
                vedge wall = landmarks_.getWall(index_east);
                east_reach = wall[0][0];
                north_reach = wall[vert_index_][1];
            }
        }
        // Head south to a new obstruction
        position_ = obstruction_;
        camps_.push_back(position_);
        std::deque<hedge> obstructions_in_view = scanPlateaus(start_east, index_east);
        mergeQue(obstruct_que_, obstructions_in_view);
        vedge topfloor = obstruct_que_[0];
        obstruct_que_.pop_front();
        position_ = {position_[0], topfloor[vert_index_][1]};
        camps_.push_back(position_);
        start_east = index_east;
        obstruction_ = topfloor[1];
    }
};
//
std::deque<hedge> PlankHiker::scanPlateaus(const int east_start, const int east_end){
    std::deque<hedge> plateau_que;
    hedge a_plateau = landmarks_.getPlateau(east_end);
    plateau_que.push_front(a_plateau);
    for (int foo = east_end - 1; foo > east_start; foo--){
        a_plateau = landmarks_.getPlateau(foo);
        if (!compareVertical(a_plateau[0][1],plateau_que[0][0][1])) plateau_que.push_front(a_plateau);
    }
    return plateau_que;
};
//
void PlankHiker::mergeQue(std::deque<hedge> &q0, std::deque<hedge> &q1){
    bool is_an_empty = (q0.size()==0) || (q1.size()==0);
    if (!is_an_empty){
        double t1y = q1[0][0][1];
        while(!compareVertical(t1y, q0.back()[0][1])){
            q0.pop_back();
            if (q0.size()==0) break;
        }
    }
    for (int foo=0; foo<q1.size(); foo++){
        q0.push_back(q1[foo]);
    }
};
//
//
//void CaliperHiker::hikeTrail(){
//
//};
