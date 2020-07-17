//
//  zoning_board.cpp
//  ack
//
//  Created by Shane Scott on 7/16/20.
//

#include "zoning_board.h"

#include "zoning_commisioner.h"

Applicant::Applicant(const double& width, const double& height, const int multiplicity) : width(width), height(height), multiplicity(multiplicity)
{};

void ZoningBoard::setRectangles(const std::vector<Eigen::Vector2d>& rectangles){
    applicants_.resize(rectangles.size());
    for (const auto& x : rectangles){
        Applicant app(x[0],x[1],1);
        applicants_.push_back(app);
    };
    placements_.resize(applicants_.size());
    for (auto foo = 0; foo < placements_.size(); foo++){
        placements_[foo].reserve(applicants_[foo].multiplicity);
    }
};


void ZoningBoard::setRectangles(const std::vector<Eigen::Vector2d>& rectangles, const std::vector<int>& multiplicites){
    applicants_.resize(rectangles.size());
    for (auto foo=0; foo < rectangles.size(); foo++){
        Applicant app(rectangles[foo][0],rectangles[foo][1], multiplicites[foo]);
        applicants_.push_back(app);
    };
    placements_.resize(applicants_.size());
    for (auto foo = 0; foo < placements_.size(); foo++){
        placements_[foo].reserve(applicants_[foo].multiplicity);
    }
};

void ZoningBoard::sortApplicants(){
    std::sort(applicants_.begin(),
              applicants_.end(),
              [](Applicant a, Applicant b){
                    if (a.width == b.width){
                        return a.height > b.height;
                    }
                    return a.width > b.width;
              }
              );
};

void ZoningBoard::zone(){
    sortApplicants();
    std::vector<ZoningCommisioner> zcs;
    for (const Vacancy& vc: vacancies_){
        ZoningCommisioner zc;
        zc.insertCurves(vc.grid_points_, vc.lines_);
        zc.populateNeighbors();
        zc.trailblaze();
    }
    for (auto foo=0; foo<applicants_.size(); foo++){
        Applicant& app = applicants_[foo];
        for (auto mult = 0; mult < app.multiplicity; mult++){
            Placement placement;
            bool is_placed = false;
            for (auto zc_foo = 0; zc_foo < zcs.size(); zc_foo++){
                ZoningCommisioner& zc = zcs[zc_foo];
                bool is_placable = zc.findPlacement(app.width, app.height, placement.position);
                if (!is_placable & allow_rotations){
                    is_placable = zc.findPlacement(app.height, app.width, placement.position);
                    if (is_placable) placement.rotated = true;
                }
                if (is_placable){
                    is_placed = true;
                    placement.vacancy_id = zc_foo;
                    placements_[foo].push_back(placement);
                    zc.zoneOff(placement.position, app.width, app.height);
                    break; // placed in a
                }
            }
            if (!is_placed){
                break; //No need to place more, break the loop on mult
            }
        }
    }
};
