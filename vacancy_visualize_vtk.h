//
//  vacancy.h
//  ack
//
//  Created by Shane Scott on 6/9/20.
//

#ifndef vacancy_visualizevtk_h_20
#define vacancy_visualizevtk_h_20

#include <vtkContext2D.h>

#include "vacancy.h"

struct VacancyVisualizeVTK{
//    vtkSmartPointer<vtkContext2D> context_ = vtkSmartPointer<vtkContext2D>::New();
    
    //
    VacancyVisualizeVTK();
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
