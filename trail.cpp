#include <unordered_map>

#include "trail.h"

Vedge Trail::getWesternFront() const {
    Eigen::Vector2d a = landmarks_valley_.points_[0];
    Eigen::Vector2d b = landmarks_mountain_.points_[0];
    return {b,a};
};

//struct RectContact{
//    int trail_id = -1;
//    int wall_id = -1;
//    int plateau_id = -1;
//    SegRelation segment_relation;
//};

/*
 
 
void removeRectangle(std::vector<Trail> &trails, const Eigen::Vector2d& position, const double& width, const double& height){
    //Removes the specified rectangle.
    //Specifically assuming bottom left placement style.
    //First find all the bottom edges that are in contact with the rectangle placement.
    std::vector<std::vector<RectContact>> contacts;
    
    std::vector<int> shadow_extents;
    shadow_extents.resize(trails.size());
    
    //
    Eigen::Vector2d se = {position[0]+width, position[1]};
    Eigen::Vector2d ne = {position[0]+width, position[1]+height};
    Eigen::Vector2d nw = {position[0], position[1]+height};
    Hedge rect_s = {position,se};
    Hedge rect_n = {nw,ne};
    Vedge rect_w = {position,nw};
    Vedge rect_e = {se,ne};
    
    //First find all the contacts
    for (int tt=0; tt<trails.size(); tt++){
        const Trail& trail_tt = trails[tt];
        for (int plat_ii=0; plat_ii < trail_tt.landmarks_valley_.num_plateaus_; plat_ii++){
            //
            Hedge plat = trail_tt.landmarks_valley_.getPlateau(plat_ii);
            if ( se[0] < plat[0][0] ) break;
            if ( plat[1][0] < position[0] ) continue;
            // so plateau foo is between
            SegRelation contact_type = hedgeIntersection(rect_s, plat);
            if (contact_type != SegRelation::kDisjoint){
                RectContact contact;
                contact.trail_id = tt;
                contact.plateau_id = plat_ii;
                contact.segment_relation = contact_type;
                contacts[tt].push_back(contact);
            }
            if ( (std::abs(plat[1][0] - position[0]) < repsilon) & (plat[1][1]<position[1]) & (plat[1][1]<nw[1]) ){
                RectContact contact;
                contact.trail_id = tt;
                contact.plateau_id = plat_ii;
                contact.segment_relation = SegRelation::kPerpindicularPoint;
            }
            
            shadow_extents[tt] = plat_ii;
        }
    //  THIS IS FOR FINDING WALL COLLISIONS
        
//        for (int wall_ii=0; wall_ii < trail_tt.landmarks_valley_.num_walls_; wall_ii++){
//            //
//            vedge wall = trail_tt.landmarks_valley_.getWall(wall_ii);
//            if ( nw[1] < wall[0][1] ) break;
//            if ( wall[1][1] < position[1] ) continue;
//            // so plateau foo is between
//            segmentRelation contact_type = hedgeIntersection(rect_w, wall);
//            if (contact_type != segmentRelation::kDisjoint){
//                RectContact contact;
//                contact.trail_id = tt;
//                contact.plateau_id = wall_ii;
//                contact.segment_relation = contact_type;
//                contacts[tt].push_back(contact);
//            }
//            shadow_extents[tt] = wall_ii;
//        }

    };
    
    if ((contacts.size() > 2) | (contacts.size() == 0)){
        std::cout << "Something bad happened to contacts!" << std::endl;
        return;
    }
    
    //We need to order the contacts  acording to their sequence around the rectangle
    
    //edit the trails
    
    for (int tt=0; tt< contacts.size(); tt++){
        Trail &trl = trails[tt];
        std::vector<Eigen::Vector2d> newvalley;
        int platoo = 0;
        for (const RectContact& contact : contacts[tt]){
            while(platoo<contact.plateau_id){
                Hedge plat = trl.landmarks_valley_.getPlateau(platoo);
                newvalley.push_back(plat[0]);
                newvalley.push_back(plat[1]);
                if ((plat[0] - position).norm() < repsilon){
                    // So the rectangle is placed at the left edge of the plateau. This is by far the most likely case.
                    int rm_start_id = 2 * contact.plateau_id;
                    int rm_end_id = 2 * shadow_extents[tt];
                    auto lwb = trl.landmarks_valley_.points_.begin();
                    double fallto = trl.landmarks_valley_.points_[rm_end_id][1];
                    trl.landmarks_valley_.points_.erase(lwb + rm_start_id, lwb + rm_end_id + 1);
                    std::vector<Eigen::Vector2d> getin = {
                        {position[0], position[1]+height},
                        {position[0]+width, position[1]+height},
                        {position[0]+width, fallto}
                    };
                    trl.landmarks_valley_.points_.insert(lwb + rm_start_id, getin.begin(), getin.end());
                    trl.landmarks_valley_.num_walls_ = (trl.landmarks_valley_.points_.size()-1)/2;
                    trl.landmarks_valley_.num_plateaus_ = (trl.landmarks_valley_.points_.size())/2;
                    return;
                }
            }
        };
        trl.landmarks_valley_.points_ = newvalley;
        trl.landmarks_valley_.num_plateaus_ = (trl.landmarks_valley_.points_.size()-1)/2;
        trl.landmarks_valley_.num_walls_ = (trl.landmarks_valley_.points_.size())/2;
    };
    
    //!!!! This insertion procedure should not be on vectors, refactor to lists? may not be worth it
    
        //This forces an interaction with another trail. Probably we need to redraw the whole thing to avoid topology mistakes.
};

*/
