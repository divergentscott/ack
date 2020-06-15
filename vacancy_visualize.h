//
//  vacancy.h
//  ack
//
//  Created by Shane Scott on 6/9/20.
//

#ifndef vacancy_visualize_h_0220
#define vacancy_visualize_h_0220

#include "vacancy.h"

struct VacancyVisualize{
    std::vector<std::string> topstack = {};
    std::vector<std::string> botstack = {};
    double min_x_;
    double min_y_;
    double max_x_;
    double max_y_;
    double stroke_width_ = 0.1;
    //
    VacancyVisualize();
    void setStroke(const double&);
    void setViewBox(const Eigen::Vector2d& position, const double& width, const double& height);
    void addRectangle(const Eigen::Vector2d& position, const double& width, const double& height);
    void addRectangles(const std::vector<Eigen::Vector2d>& positions, const double& width, const double& height);
    void addCabbieCurveCollection(const CabbieCurveCollection&);
    void addVacancy(const Vacancy&);
    void addTrail(const Trail&);
    void writeScalableVectorGraphics(const std::string& outfilepath);
};

#endif
