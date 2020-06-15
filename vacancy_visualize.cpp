#include <fstream>

#include "vacancy_visualize.h"



const std::string topline = "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>";

VacancyVisualize::VacancyVisualize(){
    topstack.push_back("<g opacity=\"0.8\">");
    botstack.push_back("</g>");
};

void VacancyVisualize::setStroke(const double &x){
    stroke_width_ = x;
}

void VacancyVisualize::setViewBox(const Eigen::Vector2d& position, const double& width0, const double& height0){
    min_x_ = position[0];
    min_y_ = -position[1];
    max_x_ = position[0] + width0;
    max_y_ = -position[1] + height0;
};

void VacancyVisualize::addRectangle(const Eigen::Vector2d& position, const double& width, const double& height){
    min_x_ = std::min(position[0], min_x_);
    min_y_ = std::min(-position[1], min_y_);
    max_x_ = std::max(max_x_, position[0] + width);
    max_y_ = std::max(max_y_, position[1] + height);
    std::stringstream ss;
    ss << "<rect x=\"" << position[0] << "\" y=\"" << -position[1]-height << "\" width=\"" << width << "\" height=\"" << height << "\" stroke-width=\""<< stroke_width_ <<"\" stroke=\"red\" fill=\"none\" />";
    topstack.push_back(ss.str());
};

void VacancyVisualize::addRectangles(const std::vector<Eigen::Vector2d>& positions, const double& width, const double& height){
    for (const auto& p : positions){
        addRectangle(p,width,height);
    }
};

void VacancyVisualize::addCabbieCurveCollection(const CabbieCurveCollection& ccc){
    int n_components = ccc.get_number_of_components();
    for (int foo=0; foo < n_components; foo++){
        std::stringstream ss;
        ss << "<polyline points=\"";
        int basepoint = ccc.get_basepoint(foo);
        int bar = basepoint;
        Eigen::Vector2d pbar = ccc.get_point(bar);
        ss << pbar[0] << "," << -pbar[1] << " ";
        do {
            bar = ccc.get_next_point(bar);
            pbar = ccc.get_point(bar);
            ss << pbar[0] << "," << -pbar[1] << " ";
            min_x_ = std::min(pbar[0], min_x_);
            min_y_ = std::min(-pbar[1], min_y_);
        } while (bar != basepoint);
        ss << "\" stroke=\"black\" stroke-width=\""<< stroke_width_ << "\" fill=\"none\" />";
        topstack.push_back(ss.str());
    };
}

//<polyline points="50,150 50,200 200,200 200,100" stroke="black" stroke-width="4" fill="none" />

void VacancyVisualize::addVacancy(const Vacancy&){};
void VacancyVisualize::addTrail(const Trail&){};

void VacancyVisualize::writeScalableVectorGraphics(const std::string& outfilepath){
    std::ofstream file;
    file.open(outfilepath.c_str());
    file << topline << "\n";
    double margin = 0.1 * std::max(max_x_ - min_x_, max_y_ - min_y_);
    double viewx = min_x_ - 0.5 * margin;
    double viewy = min_y_ - 0.5 * margin;
    double view_width = max_x_ - min_x_ + margin;
    double view_height = max_y_ - min_y_ + margin;
    file << "<svg width=\"100%\" height=\"100%\" viewBox=\"" << viewx << " " << viewy << " " << view_width << " " << view_height << " \" xmlns=\"http://www.w3.org/2000/svg\">" << "\n";
    for (const std::string &astr : topstack){
        file << astr << "\n";
    }
    for (int foo = botstack.size()-1; foo>=0; foo--){
        file << botstack[foo] << "\n";
    }
    file << "</svg>" << "\n";
    file.close();
};


