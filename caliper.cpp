//
//  caliper.cpp
//  ack
//
//  Created by Shane Scott on 6/10/20.
//

#include "caliper.h"

CaliperHiker::CaliperHiker(const Trail& trail, const double& width, const double& height) :
    width_(width),
    height_(height),
    landmarks_valley_(trail.landmarks_valley_),
    landmarks_mountain_(trail.landmarks_mountain_)
{};

void CaliperHiker::hike(){
    PlankHiker valley_trail(landmarks_valley_, width_, true);
    valley_trail.hike();
    camps_valley_ = valley_trail.camps_;
    PlankHiker mountain_trail(landmarks_mountain_, width_, false);
    mountain_trail.hike();
    camps_mountain_ = mountain_trail.camps_;
    expandCamps();
};

void CaliperHiker::expandCamps(){
    //Move through the camps and measure the vertical space, saving the ones which are sufficiently high
    if ((camps_valley_.num_plateaus_==0) | (camps_mountain_.num_plateaus_==0)) return;
    int ii = 0;
    int jj = 0;
//    std::cout << "ii " << ii << " jj " << jj << std::endl;
    Hedge low_ii = camps_valley_.getPlateau(ii);
    Hedge high_jj = camps_mountain_.getPlateau(jj);
    if (high_jj[0][1] - low_ii[0][1] >= height_){
        valid_.push_back(low_ii[0]);
//        std::cout << (valid_.end()-1)->transpose() << std::endl;
    }
    while((ii < camps_valley_.num_plateaus_-1) | (jj < camps_mountain_.num_plateaus_-1)){
        //Measure height east to west and keep any positions that are valid.
        while( (high_jj[1][0] <= low_ii[1][0]) & (jj < camps_mountain_.num_plateaus_-1) ){
            jj++;
//            std::cout << "ii " << ii << " jj " << jj << std::endl;
            high_jj = camps_mountain_.getPlateau(jj);
            if ( high_jj[0][1] - low_ii[0][1] >= height_){
                Eigen::Vector2d position = {high_jj[0][0], low_ii[0][1]};
                valid_.push_back(position);
//                std::cout << (valid_.end()-1)->transpose() << std::endl;
            }
        };
        while( (low_ii[1][0] <= high_jj[1][0]) & (ii < camps_valley_.num_plateaus_-1) ){
            ii++;
//            std::cout << "ii " << ii << " jj " << jj << std::endl;
            Eigen::Vector2d lastright = camps_valley_.getPlateau(ii-1)[1];
            if (high_jj[0][1] - lastright[1] >= height_){
                valid_.push_back(lastright);
//                std::cout << (valid_.end()-1)->transpose() << std::endl;
            }
            low_ii = camps_valley_.getPlateau(ii);
            if ( high_jj[0][1] - low_ii[0][1] >= height_){
                valid_.push_back(low_ii[0]);
//                std::cout << (valid_.end()-1)->transpose() << std::endl;

            }
        };
    }
    double lastx = std::min(low_ii[1][0], high_jj[1][0]);
    if ( high_jj[1][1] - low_ii[1][1] >= height_){
        Eigen::Vector2d position = {lastx, low_ii[1][1]};
        valid_.push_back(position);
//        std::cout << (valid_.end()-1)->transpose() << std::endl;
    }
};


PlankHiker::PlankHiker(const CardinalPath& landmarks, const double plank_width, const bool is_gravity_south) : landmarks_(landmarks), width_(plank_width), is_gravity_south_(is_gravity_south)
{
    vert_index_ = is_gravity_south_? 1 : 0;
};

void PlankHiker::hike(){
    position_ = landmarks_.points_[0] - Eigen::Vector2d({width_,0});
    Vedge obs = landmarks_.getWall(0);
    rayTraceEast(position_, obs, obstruction_);
    camps_.push_back(position_);
    headEast(0);
    nudgeFrontier();
}

bool PlankHiker::compareVertical(const double& x, const double &y) const {
    if (is_gravity_south_) return (x<y);
    return (x>y);
}

void PlankHiker::headEast(int start_east){
    int index_east = start_east;
    while(start_east < landmarks_.num_walls_){
        Vedge wall = landmarks_.getWall(index_east);
        double east_reach = wall[0][0];
        double north_reach = wall[vert_index_][1];
		//Move over or rise vertically
        while(east_reach < obstruction_[0] + width_ - repsilon){
            // Try to slide the caliper straight east along the obstruction
            // But head north if a wall gets in the way.
            if (compareVertical(position_[1], north_reach)){
                //There is a valley wall in the way of east
                position_ = {east_reach - width_, obstruction_[1]};
                camps_.push_back(position_);
                position_ = {east_reach - width_, north_reach};
                camps_.push_back(position_);
                obstruct_que_.clear();
				if (index_east+1 < landmarks_.num_walls_) rayTraceEast(position_, landmarks_.getWall(index_east+1), obstruction_);
				else rayTraceEast(position_, landmarks_.getWall(landmarks_.num_walls_-1), obstruction_);
                //Elevate, then head east.
                headEast(index_east+1);
                return;
            } else if (index_east == landmarks_.num_walls_){
                break;
            } else{
                index_east++;
				if (index_east == landmarks_.num_walls_) {
					break;
				}
                Vedge wall = landmarks_.getWall(index_east);
                east_reach = wall[0][0];
                north_reach = wall[vert_index_][1];
            }
        }
        // Head south to a new obstruction
        position_ = obstruction_;
        camps_.push_back(position_);
        std::deque<Hedge> obstructions_in_view = scanPlateaus(start_east, index_east);
        mergeQue(obstruct_que_, obstructions_in_view);
        Vedge topfloor = obstruct_que_[0];
        obstruct_que_.pop_front();
        position_ = {position_[0], topfloor[vert_index_][1]};
        camps_.push_back(position_);
        start_east = index_east;
        obstruction_ = topfloor[1];
    }
};
//
std::deque<Hedge> PlankHiker::scanPlateaus(const int east_start, const int east_end){
    std::deque<Hedge> plateau_que;
	int east_check = east_end < landmarks_.num_plateaus_ ? east_end : landmarks_.num_plateaus_ - 1;
    Hedge a_plateau = landmarks_.getPlateau(east_check);
    plateau_que.push_front(a_plateau);
    for (int foo = east_check - 1; foo > east_start; foo--){
        a_plateau = landmarks_.getPlateau(foo);
        if (!compareVertical(a_plateau[0][1],plateau_que[0][0][1])) plateau_que.push_front(a_plateau);
    }
    return plateau_que;
};
//
void PlankHiker::mergeQue(std::deque<Hedge> &q0, std::deque<Hedge> &q1){
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
void PlankHiker::nudgeFrontier(){
    //Clip any camps that are west of the western frontier.
    //The first camp is always placed -(width,0) of the western frontier's south corner.
    int east_index = 0;
    double western_frontier = landmarks_.points_[0][0];
    while( camps_.points_[east_index][0] < western_frontier ){
        east_index++;
		if (east_index == camps_.points_.size()) break;
    }
    camps_.points_[east_index-1][0] = western_frontier;
    if (east_index>1){
        camps_.points_.erase(camps_.points_.begin(), camps_.points_.begin()+(east_index-1));
        camps_.num_plateaus_ = (camps_.points_.size())/2;
        camps_.num_walls_ = (camps_.points_.size()-1)/2;
    }
    //On the eastern frontier, ensure that you get the last possible point.
    landmarks_.getPlateau(landmarks_.num_plateaus_-1);
}

Eigen::Vector2d CaliperHiker::getMostValid() const{
    Eigen::Vector2d best= valid_[0];
    for (int foo=1; foo < valid_.size(); foo++){
        if (valid_[foo][1] < best[1]) best = valid_[foo];
        if (std::abs(valid_[foo][1]-best[1]) < repsilon ){
            if (valid_[foo][0] < best[0]) best = valid_[foo];
        }
    }
    return best;
}
