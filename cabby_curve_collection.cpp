#include "cabby_curve_collection.h"

bool CabbyCurveCollection::rayTraceNorth(const Eigen::Vector2d &origin, const std::array<Eigen::Vector2d, 2> &segment, Eigen::Vector2d &impact){
    //Returns true if there is an impact, otherwise false.
    //If true, impact_location is populated with the impact point on the line segment.
    //Assumes segment is horizontal AND ordered with segment[0][0] < segment[1][0] and segment[0][1] == semgent[1][1]
    if ((origin[1] > segment[0][1]) | (segment[0][0] > origin[0]) | (origin[0] > segment[1][0])) return false;
    impact[0] = origin[0];
    impact[1] = segment[0][1];
    return true;
}
