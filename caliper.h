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

struct CaliperHiker{
    double width_;
    int num_walls_;

    Cabbie landmarks_valley_;

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
    void hikeTrailValley();
    //compute all the lower positions for the caliper
};

struct PlankHiker{
    /*
     The PlankHiker moves a fixed width interval along a cabby path and notes the positions the interval can fit.
     
     */
    const Cabbie landmarks_;
    const double width_;
    const bool is_gravity_south_;
    int vert_index_;

    Cabbie camps_;
    Eigen::Vector2d obstruction_; //point currently preventing the caliper from expanding south (or maybe north)
    std::deque<hedge> obstruct_que_; //points that could potentially catch the caliper
    Eigen::Vector2d position_;
    //
    PlankHiker()=delete;
    //
    PlankHiker(const Cabbie& landmarks, const double width, const bool is_grav_south);
    //
    bool compareVertical(const double& x, const double &y) const;;
    //
    void hike();
    //
    void headEast(int start_east);
    //move the plank east with
    
    std::deque<hedge> scanPlateaus(const int east_start, const int east_end);
    
    void mergeQue(std::deque<hedge> &q0, std::deque<hedge> &q1);
};

#endif /* caliper_hpp */
