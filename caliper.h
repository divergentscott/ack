//
//  caliper.hpp
//  ack
//
//  Created by Shane Scott on 6/10/20.
//

#ifndef caliper_hpp_asdfpoij
#define caliper_hpp_asdfpoij

#include <deque>
#include <list>
#include <stdio.h>

#include <Eigen/Dense>

#include "cabby_curve_collection.h"
#include "vacancy.h"

using vedge = std::array<Eigen::Vector2d,2>;
using hedge = std::array<Eigen::Vector2d,2>;

struct CaliperHiker{
    double width_;
    int num_walls_;
    std::vector<Eigen::Vector2d> landmarks_valley_;
//    std::vector<vedge> walls_valley_; //all the places the caliper stops to measure
//    Eigen::Vector2d valley_start_;
//    Eigen::Vector2d valley_end_;
//    std::vector<vedge> walls_mountain_; //all the places the caliper stops to measure
    
    std::vector<Eigen::Vector2d> camps_valley_;
    std::vector<Eigen::Vector2d> camps_mountain_;

    Eigen::Vector2d obstruction_; //point currently preventing the caliper from expanding south (or maybe north)
    std::deque<hedge> obstruct_que_; //points that could potentially catch the caliper
    Eigen::Vector2d position_;
    //
    CaliperHiker(){};
    //
    CaliperHiker(double width, Trail trail);
    //
    vedge get_wall_valley_(int east_index);
    //
    hedge get_floor_valley_(int east_index);
    //
    void hikeTrail();
    //compute all the lower positions for the caliper
    
    void headEast(int start_east);
    //move the caliper east with
    
    std::deque<hedge> scanObstructions(int east_start, int east_end);
    
    void mergeObsQue(std::deque<hedge> &q0, std::deque<hedge> &q1);
    
    void eraseBadCamps();
};


#endif /* caliper_hpp */
