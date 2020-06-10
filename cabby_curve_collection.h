//
//  cabby_curve_collection.hpp
//  ack
//
//  Created by Shane Scott on 5/29/20.
//

#ifndef cabby_curve_collection_hpp
#define cabby_curve_collection_hpp

#include "curve_collection.h"

class CabbyCurveCollection : public CurveCollection {
public:
    bool rayTraceNorth(const Eigen::Vector2d &origin, const std::array<Eigen::Vector2d,2> &segment,  Eigen::Vector2d &impact);
    //Returns true if there is an impact, otherwise false.
    //If true, impact_location is populated with the impact point on the line segment.
};

#endif /* cabby_curve_collection_hpp */
