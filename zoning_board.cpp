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

void ZoningBoard::annexVacancy(const std::vector<Eigen::Vector2d>& grid_points,
	const int multiplicity) {
	auto n_pts = grid_points.size();
	std::vector<std::vector<int>> lines(n_pts);
	auto foo = 0;
	while (foo + 1 < n_pts) {
		lines[foo] = {foo, foo+1};
		foo++;
	}
	lines[foo] = {foo, 0};
	annexVacancy(grid_points, lines, multiplicity);
};

void ZoningBoard::annexVacancy(const std::vector<Eigen::Vector2d>& grid_points,
	const std::vector<std::vector<int>> &lines,
	const int multiplicity) {
	Vacancy vac;
	vac.grid_points_ = grid_points;
	vac.lines_ = lines;
	vac.multiplicity_ = multiplicity;
	//!!!! check that this is a valid cardinal curve before continuing any further.
	// Shouls also check for nested boundary components
	// too easy to give garbage input here.
	vacancies_.push_back(vac);
};

void ZoningBoard::setApplicantRectangles(const std::vector<Eigen::Vector2d>& rectangles){
	std::vector<int> multiplicities(rectangles.size());
	std::fill(multiplicities.begin(), multiplicities.end(), 1);
	setApplicantRectangles(rectangles, multiplicities);
};


void ZoningBoard::setApplicantRectangles(const std::vector<Eigen::Vector2d>& rectangles, const std::vector<int>& multiplicites){
    applicants_.resize(rectangles.size());
    for (auto foo=0; foo < rectangles.size(); foo++){
		if (rectangles[foo][0] >= rectangles[foo][1]) {
			Applicant app(rectangles[foo][0], rectangles[foo][1], multiplicites[foo]);
			applicants_[foo] = app;
		}
		else {
			Applicant app(rectangles[foo][1], rectangles[foo][0], multiplicites[foo]);
			applicants_[foo] = app;
		}
	};
    placements_.resize(rectangles.size());
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

	if (false) {
		std::cout << "RECTANGLES:" << std::endl;
		for (const auto& app : applicants_) {
			std::cout << "{" << app.width << ", " << app.height << "}," << std::endl;
		}
	}

	std::vector<ZoningCommisioner> zcs;
	zcs.reserve(vacancies_.size());
    for (const Vacancy& vc: vacancies_){
        ZoningCommisioner zc;
        zc.insertCurves(vc.grid_points_, vc.lines_);
        zc.populateNeighbors();
        zc.constructZoneCovering();
		zcs.push_back(zc);
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
                    break; //Already placed. No more commisioners need to try.
                }
            }
            if (!is_placed){
				denials_.push_back({app.width, app.height});
                break; //No need to place more, break the loop on mult
            }
        }
    }
};
