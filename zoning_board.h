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
    double width;
    double height;
    int multiplicity;
    Applicant(){};
    Applicant(const double& width, const double& height, const int multiplicity);
};

struct Vacancy{
    std::vector<Eigen::Vector2d> grid_points_;
    std::vector<std::vector<int>> lines_;
};

struct Placement{
    Eigen::Vector2d position;
    int vacancy_id;
    bool rotated = false;
};

class ZoningBoard{
private:
    ZoningStratgey strategy_ = ZoningStratgey::kBottomLeft;
    bool allow_rotations = false;
    std::vector<Applicant> applicants_; // rectangles to pack width x height
    std::vector<std::vector<Placement>> placements_;
    std::vector<Vacancy> vacancies_;
    void sortApplicants();
    
public:
    ZoningBoard(){};
    void addVacancy(const Eigen::Vector2d&grid_points, const std::vector<std::vector<int>> &lines, const int multiplicity = 1);
    void setRectangles(const std::vector<Eigen::Vector2d>& rectangles);
    void setRectangles(const std::vector<Eigen::Vector2d>& rectangles, const std::vector<int>& multiplicites);
    void zone();
};

#endif /* zoning_board_hpp */
