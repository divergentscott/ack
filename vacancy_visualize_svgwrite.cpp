#include <fstream>

#include "vacancy_visualize_svgwrite.h"


VacancyVisualizer::VacancyVisualizer(const Eigen::Vector2d& position, const double width, const double height, const std::string& outfilepath) :
    svgdoc_(outfilepath, svg::Layout(svg::Dimensions(width,height),
                    svg::Layout::Origin::BottomLeft,
                    1, //scale
                    svg::Point(position[0], position[1])))
{};

void VacancyVisualizer::addRectangle(const Eigen::Vector2d& position, const double& width, const double& height, const svg::Color& color){
    svgdoc_ << svg::Rectangle(svg::Point(position[0], position[1]), width, height, color);
};

void VacancyVisualizer::addRectangles(const std::vector<Eigen::Vector2d>& positions, const double& width, const double& height, const svg::Color& color){
    for (const auto& p : positions){
        addRectangle(p, width, height, color);
    }
};

void VacancyVisualizer::setStroke(const double &x){
    stroke_ = x;
};

void VacancyVisualizer::addCabbieCurveCollection(const CabbieCurveCollection& ccc, const svg::Color& color){
    int n_components = ccc.get_number_of_components();
    for (int foo=0; foo < n_components; foo++){
        svg::Polyline polyline_a(svg::Stroke(stroke_, color));
        int basepoint = ccc.get_basepoint(foo);
        int bar = basepoint;
        Eigen::Vector2d pbar = ccc.get_point(bar);
        polyline_a << svg::Point(pbar[0], pbar[1]);
        do {
            bar = ccc.get_next_point(bar);
            pbar = ccc.get_point(bar);
            polyline_a << svg::Point(pbar[0], pbar[1]);
        } while (bar != basepoint);
        svgdoc_ << polyline_a;
    }
};

void VacancyVisualizer::addCabbiePath(const CabbiePath &cpath, const svg::Color& color){
    svg::Polyline polyline_a(svg::Stroke(stroke_, color));
    for (auto &pp : cpath.points_){
        polyline_a << svg::Point(pp[0], pp[1]);
    }
    svgdoc_ << polyline_a;
};



void VacancyVisualizer::save(){
    svgdoc_.save();
};


