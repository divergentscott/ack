#include <unordered_map>

#include "trail.h"

vedge Trail::getWesternFront() const {
    Eigen::Vector2d a = landmarks_valley_.points_[0];
    Eigen::Vector2d b = landmarks_mountain_.points_[0];
    return {b,a};
};

enum class segmentRelation{
    kDisjoint,
    kSupersegment,
    kSubsegment,
    kDirectStack,
    kReverseStack
};

segmentRelation hedgeIntersection(hedge a, hedge b){
    if ( std::abs(a[0][1] - b[0][1]) < repsilon ) return segmentRelation::kDisjoint;
    //So a and b have sufficiently close y.
    if (a[0][0] <= b[0][0]) {
        if (a[1][0] < b[0][0]) return segmentRelation::kDisjoint;
        //So b[0][0] <= a[1][0]
        if (b[1][0] <= a[1][0]) return segmentRelation::kSupersegment;
        //So a[1][0] < b[1][0]
        return segmentRelation::kDirectStack;
    }
    // So b[0][0]<a[0][0]
    if (b[1][0] < a[0][0]) return segmentRelation::kDisjoint;
    //
    if (a[1][0] <= b[1][0]) return segmentRelation::kSubsegment;
    //So a[1][0] < b[1][0]
    return segmentRelation::kReverseStack;
};


void Trail::removeRectangle(const Eigen::Vector2d& position, const double& width, const double& height){
    //Specifically assuming bottom left placement style.
    //First find all the bottom edges that are in contact with the rectangle placement.
    std::unordered_map<int,segmentRelation> contacts;
    hedge rect_bot ={position, {position[0]+width, position[1]}};
    for (int foo=0; foo < landmarks_valley_.num_plateaus_; foo++){
        hedge platfoo = landmarks_valley_.getPlateau(foo);
        if ( position[0] + width < platfoo[0][0] ) break;
        if ( platfoo[1][0] < position[0] ) continue;
        segmentRelation contact_type = hedgeIntersection(rect_bot, platfoo);
        if (contact_type != segmentRelation::kDisjoint) contacts[foo] = contact_type;
    }
    //if there is a single contact
}
