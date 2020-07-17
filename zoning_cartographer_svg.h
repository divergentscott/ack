//
//  vacancy.h
//  ack
//
//  Created by Shane Scott on 6/9/20.
//

#ifndef vacancy_visualize_h_0220
#define vacancy_visualize_h_0220

#include "zoning_commisioner.h"

namespace svgvis{

std::string chaosHex();

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

struct PolyLine{
    std::vector<Eigen::Vector2d> points;
    std::string fill_color = "none";
    double stroke_width;
    std::string stroke_color;
    PolyLine(){};
    std::string getEntry() const;
};


struct ZoningCartographerSVG{
    double min_x_;
    double min_y_;
    double max_x_;
    double max_y_;
    double default_stroke_width_ = 0.1;
    std::vector<svgvis::Rectangle> rectangles_ = {};
    std::vector<svgvis::PolyLine> polylines_ = {};
    //
	/*
	"#3EC300"
	"#5F5AA2"
	"#30292F"
	"#355691"
	"#355691"
	*/
    ZoningCartographerSVG();
    void setStroke(const double&);
    void addRectangle(const Eigen::Vector2d& position, const double& width, const double& height, std::string color = "random", const double& stroke = -1);
    void addRectangles(const std::vector<Eigen::Vector2d>& positions, const double& width, const double& height, std::string color = "random", const double& stroke = -1);
    void addCardinalCurveCollection(const CardinalCurveCollection&, std::string color = "random", const double& stroke = -1);
    void addCardinalPath(const CardinalPath&, std::string color = "random", const double& stroke = -1);
	void addPointList(const PointList&, std::string color = "random", const double& stroke = -1);
    void writeScalableVectorGraphics(const std::string& outfilepath);
};

}; //namespace svg


#endif
