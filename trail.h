#ifndef trail_hpp_anicetrail
#define trail_hpp_anicetrail

#include "cabbie_curve_collection.h"


struct Trail{
    //Trails cover the Wilderness.
    //Trails have a path of lower and upper points in R2
    //Trails do not contain east or west notches.
    CabbiePath landmarks_mountain_;
    CabbiePath landmarks_valley_;
    Vedge getWesternFront() const;
};

void removeRectangle(std::vector<Trail> &trails, const Eigen::Vector2d& position, const double& width, const double& height);

#endif /* patrol_hpp */
