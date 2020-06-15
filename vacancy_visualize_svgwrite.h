//
//  vacancy.h
//  ack
//
//  Created by Shane Scott on 6/9/20.
//

#ifndef vacancy_visualize_svgwrite_h_0220
#define vacancy_visualize_svgwrite_h_0220

#include "simple_svg_1.0.0.hpp"

#include "vacancy.h"

struct VacancyVisualizer{
    svg::Document svgdoc_ ;
    double stroke_ = 0.1;
    //
    VacancyVisualizer()=delete;
    VacancyVisualizer(const Eigen::Vector2d& position, const double width, const double height, const std::string& outfilepath);
    void setStroke(const double&);
    void setViewBox(const Eigen::Vector2d& position, const double& width, const double& height);
    void addRectangle(const Eigen::Vector2d& position, const double& width, const double& height, const svg::Color& color = svg::Color::Red);
    void addRectangles(const std::vector<Eigen::Vector2d>& positions, const double& width, const double& height, const svg::Color& color = svg::Color::Red);
    void addCabbieCurveCollection(const CabbieCurveCollection&, const svg::Color& color = svg::Color::Blue);
    void addCabbiePath(const CabbiePath&,  const svg::Color& color = svg::Color::Green);
    void save();
};

#endif
