#include <fstream>
#include <random>

#include "wilderness_cartographer_svg.h"

namespace svgvis {

std::string chaosHex(){
    std::random_device rand_dev;
    std::mt19937 rng(rand_dev());
    std::uniform_int_distribution<std::mt19937::result_type> dist16(0,15);
    std::string hexanymns = "0123456789ABCDEF";
    std::string hex = "#";
    for (int foo=0; foo<6; foo++){
        hex += hexanymns[dist16(rng)];
    }
    return hex;
}

std::string Rectangle::getEntry() const{
    std::stringstream ss;
    ss << "<rect";
    ss << " x=\"" << position[0] << "\"";
    ss << " y=\"" << position[1] << "\"";
    ss << " width=\"" << width << "\"";
    ss << " height=\"" << height << "\"";
//    ss << " stroke-width=\"" << stroke_width << "\"";
//    ss << " stroke=\"" << stroke_color << "\"";
    ss << " fill=\"" << fill_color << "\"";
    ss << " fill-opacity=\"0.7\" />";
    return ss.str();
};

std::string Polyline::getEntry() const{
    std::stringstream ss;
    ss << "<polyline";
    ss << " points=\"";
    for (const auto &p : points){
        ss << " " << p[0] << "," << p[1];
    }
    ss << "\"";
    ss << " stroke-width=\"" << stroke_width << "\"";
    ss << " stroke=\"" << stroke_color << "\"";
    ss << " fill=\"" << fill_color << "\"";
    ss << "/>";
    return ss.str();
};

const std::string topline = "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>";

};//svgvis

WildernessCartographerSVG::WildernessCartographerSVG(){
};

void WildernessCartographerSVG::setStroke(const double &x){
    default_stroke_width_ = x;
}

void WildernessCartographerSVG::addRectangle(const Eigen::Vector2d& position, const double& width, const double& height, std::string color, const double& stroke){
	if (color.compare("random") == 0) {
		color = svgvis::chaosHex();
	}
	min_x_ = std::min(position[0], min_x_);
    min_y_ = std::min(position[1], min_y_);
    max_x_ = std::max(max_x_, position[0] + width);
    max_y_ = std::max(max_y_, position[1] + height);
    svgvis::Rectangle rec;
    rec.position = position;
    rec.width = width;
    rec.height = height;
    rec.stroke_color = color;
    rec.fill_color = color;
    rec.stroke_width = stroke;
    if (stroke < 0) rec.stroke_width = default_stroke_width_;
    rectangles_.push_back(rec);
};

void WildernessCartographerSVG::addRectangles(const std::vector<Eigen::Vector2d>& positions, const double& width, const double& height, std::string color, const double& stroke){
	if (color.compare("random") == 0) {
		color = svgvis::chaosHex();
	}
	for (const auto& p : positions){
        addRectangle(p,width,height,color,stroke);
    }
};

void WildernessCartographerSVG::addCardinalCurveCollection(const CardinalCurveCollection& ccc, std::string color, const double& stroke){
	if (color.compare("random") == 0) {
		color = svgvis::chaosHex();
	}
	int n_components = ccc.get_number_of_components();
    for (int foo=0; foo < n_components; foo++){
        svgvis::Polyline plyl;
        plyl.stroke_color = color;
        plyl.stroke_width = stroke;
        if (stroke < 0) plyl.stroke_width = default_stroke_width_;
        //
        int basepoint = ccc.get_basepoint(foo);
        int bar = basepoint;
        Eigen::Vector2d pbar = ccc.get_point(bar);
        plyl.points.push_back(pbar);
        do {
            bar = ccc.get_next_point(bar);
            pbar = ccc.get_point(bar);
            plyl.points.push_back(pbar);
            min_x_ = std::min(pbar[0], min_x_);
            min_y_ = std::min(pbar[1], min_y_);
            max_x_ = std::max(pbar[0], max_x_);
            max_y_ = std::max(pbar[1], max_y_);
        } while (bar != basepoint);
        polylines_.push_back(plyl);
    };
}

void WildernessCartographerSVG::addCardinalPath(const CardinalPath &cp, std::string color, const double& stroke){
	if (color.compare("random") == 0) {
		color = svgvis::chaosHex();
	}
	addPointList(cp.points_, color, stroke);
};

void WildernessCartographerSVG::addPointList(const PointList& pl, std::string color, const double& stroke){
	if (color.compare("random") == 0) {
		color = svgvis::chaosHex();
	}
	svgvis::Polyline plyl;
	plyl.stroke_color = color;
	plyl.stroke_width = stroke;
	if (stroke < 0) plyl.stroke_width = default_stroke_width_;
	for (const auto & pp : pl) {
		min_x_ = std::min(pp[0], min_x_);
		min_y_ = std::min(pp[1], min_y_);
		max_x_ = std::max(pp[0], max_x_);
		max_y_ = std::max(pp[1], max_y_);
	}
	plyl.points = pl;
	polylines_.push_back(plyl);
};

void WildernessCartographerSVG::writeScalableVectorGraphics(const std::string& outfilepath){
    std::ofstream file;
    file.open(outfilepath.c_str());
    file << svgvis::topline << std::endl;
    double margin = 0.1 * std::max(max_x_ - min_x_, max_y_ - min_y_);
    double viewx = min_x_ - 0.5 * margin;
    double viewy = min_y_ - 0.5 * margin;
    double view_width = max_x_ - min_x_ + margin;
    double view_height = max_y_ - min_y_ + margin;
    file << "<svg width=\"100%\" height=\"100%\" viewBox=\"" << 0 << " " << 0 << " " << view_width << " " << view_height  << " \" xmlns=\"http://www.w3.org/2000/svg\">" << std::endl;
    file << "<g transform=\"translate("<< -viewx << "," << view_height + viewy <<") scale(1,-1)\">" << std::endl;
    for (const svgvis::Polyline &q : polylines_){
        file << q.getEntry() << std::endl;
    }
    for (const svgvis::Rectangle &r : rectangles_){
        file << r.getEntry() << std::endl;
    }
    file <<"</g>" << std::endl;
    file << "</svg>" << std::endl;
    file.close();
};


