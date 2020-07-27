//
//  zoning_board.hpp
//  ack
//
//  Created by Shane Scott on 7/16/20.
//

#ifndef zoning_board_hpp
#define zoning_board_hpp

#include <vector>

#include <Eigen/Dense>

enum class ZoningStratgey{
    kUnknown,
    kBottomLeft
};

struct Applicant{
	int id;
    double width;
    double height;
    int multiplicity;
    Applicant(){};
    Applicant(const int id, const double& width, const double& height, const int multiplicity);
};

struct Vacancy{
    std::vector<Eigen::Vector2d> grid_points_;
    std::vector<std::vector<int>> lines_;
	int multiplicity_;
	int num_copies_ = 0;
};

struct Placement{
    Eigen::Vector2d position;
    int vacancy_clone_id;
    bool rotated = false;
};

class ZoningBoard{
//private:
public:
    ZoningStratgey strategy_ = ZoningStratgey::kBottomLeft;
    bool allow_rotations = false;
    std::vector<Applicant> applicants_; // rectangles to pack width x height
	std::vector<int> vacancy_clone_parent_ids_; // Points to the vacancy that a clone came from.
	std::vector<std::vector<Placement>> placements_; //indexed by applicant id
    std::vector<Vacancy> vacancies_;
	std::vector<Eigen::Vector2d> denials_;
    void sortApplicants();
    
public:
    ZoningBoard(){};
	// Negative multiplicities are used to indicate no limit.
	void annexVacancy(const std::vector<Eigen::Vector2d>& grid_points,
		const int multiplicity = 1);
    void annexVacancy(const std::vector<Eigen::Vector2d>& grid_points,
		const std::vector<std::vector<int>> &lines,
		const int multiplicity = 1);

	void setApplicantRectangles(const std::vector<Eigen::Vector2d>& rectangles);
    void setApplicantRectangles(const std::vector<Eigen::Vector2d>& rectangles, const std::vector<int>& multiplicites);
    void zone();
};

#endif /* zoning_board_hpp */
