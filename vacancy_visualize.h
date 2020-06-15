//
//  vacancy.h
//  ack
//
//  Created by Shane Scott on 6/9/20.
//

#ifndef vacancy_visualize_h_0220
#define vacancy_visualize_h_0220

#include "vacancy.h"

namespace svgvis{

struct Rectangle{
    Eigen::Vector2d position;
    double width;
    double height;
    std::string fill_color = "none";
    double stroke_width;
    std::string stroke_color;
    Rectangle(){};
    std::string getEntry() const;
};

struct Polyline{
    std::vector<Eigen::Vector2d> points;
    std::string fill_color = "none";
    double stroke_width;
    std::string stroke_color;
    Polyline(){};
    std::string getEntry() const;
};

}

struct VacancyVisualize{
    double min_x_ = 1e8;
    double min_y_ = 1e8;
    double max_x_ = -1e8;
    double max_y_ = -1e8;
    double default_stroke_width_ = 0.1;
    std::vector<svgvis::Rectangle> rectangles = {};
    std::vector<svgvis::Polyline> polylines = {};
    //
    VacancyVisualize();
    void setStroke(const double&);
    void addRectangle(const Eigen::Vector2d& position, const double& width, const double& height, const std::string& color = "#3EC300", const double& stroke = -1);
    void addRectangles(const std::vector<Eigen::Vector2d>& positions, const double& width, const double& height, const std::string& color = "#3EC300", const double& stroke = -1);
    void addCabbieCurveCollection(const CabbieCurveCollection&, const std::string& color = "#337CA0", const double& stroke = -1);
    void addCabbiePath(const CabbiePath&, const std::string& color = "#FFFC31", const double& stroke = -1);
    void writeScalableVectorGraphics(const std::string& outfilepath);
};

#endif
