//
//  caliper.cpp
//  ack
//
//  Created by Shane Scott on 6/10/20.
//

#include "caliper.h"

vedge sortVedge(const Eigen::Vector2d &a, const Eigen::Vector2d &b){
    //Remove once stable:
    if (std::abs(a[0]-b[0])>repsilon){
        std::cout << "warning! bad vedgies!" << a.transpose() << " and " << b.transpose();
    }
    if (a[1] < b[1]) return {a,b};
    return {b,a};
};

vedge CaliperHiker::get_wall_valley_(int east_index){
    auto a = landmarks_valley_[2*east_index+1];
    auto b = landmarks_valley_[2*east_index+2];
    if (a[1] < b[1]) return {a,b};
    return {b,a};
}

hedge CaliperHiker::get_floor_valley_(int east_index){
    auto a = landmarks_valley_[2*east_index];
    auto b = landmarks_valley_[2*east_index+1];
    if (a[0] < b[0]) return {a,b};
    return {b,a};
}


CaliperHiker::CaliperHiker(double width, Trail trail){
    width_ = width;
    //Construct the valley walls
    if (trail.landmarks_valley.size()>1){
        landmarks_valley_.resize(trail.landmarks_valley.size());
        num_walls_ = (landmarks_valley_.size()-1)/2;
        std::list<Eigen::Vector2d>::iterator trit = trail.landmarks_valley.begin();
        for (int foo=0; foo<landmarks_valley_.size(); foo++){
            auto p0 = *trit;
            landmarks_valley_[foo] = p0;
            trit++;
        }
    }
    //Copy the mountain walls
        //!!!!
};

void CaliperHiker::eraseBadCamps(){};

void CaliperHiker::hikeTrail(){
    position_ = landmarks_valley_[0] - Eigen::Vector2d({width_,0});
    vedge obs = get_wall_valley_(0);
    rayTraceEast(position_, obs, obstruction_);
    camps_valley_.push_back(position_);
    std::cout << "camp at " << position_.transpose() << std::endl;
    headEast(0);
    eraseBadCamps();
}

void CaliperHiker::headEast(int start_east){
    int index_east = start_east;
    while(start_east < num_walls_){
        double east_reach = get_wall_valley_(index_east)[0][0];
        double north_reach = get_wall_valley_(index_east)[1][1];
        while(east_reach < obstruction_[0] + width_){
            // Try to slide the caliper straight east along the obstruction
            // But head north if a wall gets in the way.
            if (north_reach > position_[1]){
                //There is a valley wall in the way of east
                position_ = {east_reach - width_, obstruction_[1]};
                camps_valley_.push_back(position_);
                std::cout << "camp at " << position_.transpose() << std::endl;
                position_ = {east_reach - width_, north_reach};
                camps_valley_.push_back(position_);
                std::cout << "camp at " << position_.transpose() << std::endl;
                obstruct_que_.clear();
                rayTraceEast(position_, get_wall_valley_(index_east+1), obstruction_);
                headEast(index_east+1);
                return;
            } else if (index_east == num_walls_-1){
                break;
            } else {
                index_east++;
                vedge wall = get_wall_valley_(index_east);
                east_reach = wall[0][0];
                north_reach = wall[1][1];
            }
        }
        // Head south to a new obstruction
        position_ = obstruction_;
        camps_valley_.push_back(position_);
        std::cout << "camp at " << position_.transpose() << std::endl;
        std::deque<hedge> obstructions_in_view = scanObstructions(start_east, index_east);
        mergeObsQue(obstruct_que_, obstructions_in_view);
        vedge topfloor = obstruct_que_[0];
        obstruct_que_.pop_front();
        position_ = {position_[0], topfloor[0][1]};
        camps_valley_.push_back(position_);
        std::cout << "camp at " << position_.transpose() << std::endl;
        start_east = index_east;
        obstruction_ = topfloor[1];
    }
};

std::deque<hedge> CaliperHiker::scanObstructions(int east_start, int east_end){
    std::deque<hedge> obs_up;
    hedge obs = get_floor_valley_(east_end);
    obs_up.push_front(obs);
    for (int foo = east_end - 1; foo > east_start; foo--){
        obs = get_floor_valley_(foo);
        if (obs[0][1] >= obs_up[0][0][1]) obs_up.push_front(obs);
    }
    return obs_up;
};

void CaliperHiker::mergeObsQue(std::deque<hedge> &q0, std::deque<hedge> &q1){
    bool is_an_empty = (q0.size()==0) || (q1.size()==0);
    if (!is_an_empty){
        double t1y = q1[0][0][1];
        while(q0.back()[0][1] <= t1y){
            q0.pop_back();
            if (q0.size()==0) break;
        }
    }
    for (int foo=0; foo<q1.size(); foo++){
        q0.push_back(q1[foo]);
    }
};
