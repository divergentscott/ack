#include <fstream>
#include <random>

#include "zoning_cartographer_svg.h"

#include <algorithm>
#include <limits>

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

std::string PolyLine::getEntry() const{
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


ZoningCartographerSVG::ZoningCartographerSVG(){
    min_x_ = std::numeric_limits<double>::max();
    min_y_ = std::numeric_limits<double>::max();
    max_x_ = std::numeric_limits<double>::lowest();
    max_y_ = std::numeric_limits<double>::lowest();
};

void ZoningCartographerSVG::setStroke(const double &x){
    default_stroke_width_ = x;
}

void ZoningCartographerSVG::addRectangle(const Eigen::Vector2d& position, const double& width, const double& height, std::string color, const double& stroke){
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

void ZoningCartographerSVG::addRectangles(const std::vector<Eigen::Vector2d>& positions, const double& width, const double& height, std::string color, const double& stroke){
	if (color.compare("random") == 0) {
		color = svgvis::chaosHex();
	}
	for (const auto& p : positions){
        addRectangle(p,width,height,color,stroke);
    }
};

void ZoningCartographerSVG::addCardinalCurveCollection(const CardinalCurveCollection& ccc, std::string color, const double& stroke){
	if (color.compare("random") == 0) {
		color = svgvis::chaosHex();
	}
	int n_components = ccc.get_number_of_components();
    for (int foo=0; foo < n_components; foo++){
        svgvis::PolyLine plyl;
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

void ZoningCartographerSVG::addCardinalPath(const CardinalPath &cp, std::string color, const double& stroke){
	if (color.compare("random") == 0) {
		color = svgvis::chaosHex();
	}
	addPointList(cp.points_, color, stroke);
};

void ZoningCartographerSVG::addPointList(const PointList& pl, std::string color, const double& stroke){
	if (color.compare("random") == 0) {
		color = svgvis::chaosHex();
	}
	svgvis::PolyLine plyl;
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

void ZoningCartographerSVG::writeScalableVectorGraphics(const std::string& outfilepath){
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
    for (const svgvis::PolyLine &q : polylines_){
        file << q.getEntry() << std::endl;
    }
    for (const svgvis::Rectangle &r : rectangles_){
        file << r.getEntry() << std::endl;
    }
    file <<"</g>" << std::endl;
    file << "</svg>" << std::endl;
    file.close();
};

void ZoningCartographerSVG::addZoningBoardReport(const ZoningBoard zb) {
	//!!!! This is broken for vacancy multiplicity.
	std::vector<Eigen::Vector2d> vis_offsets(zb.vacancies_.size() + 1);
	std::vector<double> west(zb.vacancies_.size() + 1);
	std::vector<double> east(zb.vacancies_.size() + 1);
	vis_offsets[0] = { 0,0 };
	//find offsets for each vacancy to visualize together
	for (auto foo = 0; foo < zb.vacancies_.size(); foo++) {
		const Vacancy& vac = zb.vacancies_[foo];
		auto max_foo = std::max_element(
			vac.grid_points_.begin(),
			vac.grid_points_.end(),
			[](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {return a[0] < b[0];}
		);
		auto min_foo = std::min_element(
			vac.grid_points_.begin(),
			vac.grid_points_.end(),
			[](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {return a[0] < b[0];}
		);
		west[foo] = (*min_foo)[0];
		east[foo] = (*max_foo)[0];
	};
	//
	for (auto foo = 0; foo < zb.vacancies_.size() + 1; foo++) {
		vis_offsets[foo] = {-west[foo], 0};
		if (foo > 0) {
			vis_offsets[foo][0] += vis_offsets[foo - 1][0] + (east[foo - 1] - west[foo - 1]) * 1.1;
		}
	}
	//
	for (auto foo = 0; foo < zb.vacancies_.size(); foo++) {
		const Vacancy& vac = zb.vacancies_[foo];
		PointList offsetpoints;
		offsetpoints.reserve(vac.grid_points_.size());
		for (const auto &pp : vac.grid_points_) offsetpoints.push_back(pp + vis_offsets[foo]);
		CardinalCurveCollection ccc;
		ccc.setGridPointsAndCells(offsetpoints, vac.lines_);
		addCardinalCurveCollection(ccc, "black");
	}
	//
	for (auto vac_foo = 0; vac_foo < zb.vacancies_.size(); vac_foo++) {
		const Vacancy& vac = zb.vacancies_[vac_foo];
		//Need to do some vacancy multiplying here.
		for (auto bar = 0; bar < zb.applicants_.size(); bar++) {
			const Applicant& appli = zb.applicants_[bar];
			for (const Placement& pp : zb.placements_[bar]) {
				if (pp.vacancy_id == vac_foo) {
					if (!pp.rotated) {
						addRectangle(pp.position + vis_offsets[vac_foo], appli.width, appli.height);
					}
					else {
						addRectangle(pp.position + vis_offsets[vac_foo], appli.height, appli.width);
					}
				}
			}
		}
	}
	for (const auto &denied : zb.denials_) {
		addRectangle(vis_offsets.back(), denied[0], denied[1]);
	}
};


};//svgvis
