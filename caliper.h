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

#include "trail.h"

struct CaliperHiker{
    //Class to hike a single trail searching for positions.
    const double width_;
    const double height_;
    const CardinalPath& landmarks_valley_;
    const CardinalPath& landmarks_mountain_;
    CardinalPath camps_valley_;
    CardinalPath camps_mountain_;
    std::vector<Eigen::Vector2d> valid_;
    //
    CaliperHiker(const Trail& trail, const double& width, const double &height);
    //
    void hike();
    void unifyCamps();
    void measureCamps();
    void expandCamps();
    Eigen::Vector2d getMostValid() const;
};

struct PlankHiker{
    /*
     The PlankHiker moves a fixed width interval along a cabby path and notes the positions the interval can fit.
     
     */
    const CardinalPath landmarks_;
    const double width_;
    const bool is_gravity_south_;
    int vert_index_;

    CardinalPath camps_;
    Eigen::Vector2d obstruction_; //point currently preventing the caliper from expanding south (or maybe north)
    std::deque<Hedge> obstruct_que_; //points that could potentially catch the caliper
    Eigen::Vector2d position_;
    //
    PlankHiker()=delete;
    //
    PlankHiker(const CardinalPath& landmarks, const double width, const bool is_grav_south);
    //
    bool compareVertical(const double& x, const double &y) const;;
    //
    void hike();
    //
    void headEast(int start_east);
    //move the plank east with
    
    std::deque<Hedge> scanPlateaus(const int east_start, const int east_end);
    
    void mergeQue(std::deque<Hedge> &q0, std::deque<Hedge> &q1);
    //
    void nudgeFrontier();
};

#endif /* caliper_hpp */
