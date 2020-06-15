#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkSVGExporter.h>

#include "vacancy_visualize_vtk.h"


#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkNamedColors.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolygon.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>


VacancyVisualizeVTK::VacancyVisualizeVTK(){
    vtkSmartPointer<vtkNamedColors> colors =
      vtkSmartPointer<vtkNamedColors>::New();

    // Setup four points
    vtkSmartPointer<vtkPoints> points =
      vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(0.0, 0.0, 0.0);
    points->InsertNextPoint(1.0, 0.0, 0.0);
    points->InsertNextPoint(1.0, 1.0, 0.0);
    points->InsertNextPoint(0.0, 1.0, 0.0);

    // Create the polygon
    vtkSmartPointer<vtkPolygon> polygon =
      vtkSmartPointer<vtkPolygon>::New();
    polygon->GetPointIds()->SetNumberOfIds(4); //make a quad
    polygon->GetPointIds()->SetId(0, 0);
    polygon->GetPointIds()->SetId(1, 1);
    polygon->GetPointIds()->SetId(2, 2);
    polygon->GetPointIds()->SetId(3, 3);

    // Add the polygon to a list of polygons
    vtkSmartPointer<vtkCellArray> polygons =
      vtkSmartPointer<vtkCellArray>::New();
    polygons->InsertNextCell(polygon);

    // Create a PolyData
    vtkSmartPointer<vtkPolyData> polygonPolyData =
      vtkSmartPointer<vtkPolyData>::New();
    polygonPolyData->SetPoints(points);
    polygonPolyData->SetPolys(polygons);

    // Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> mapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polygonPolyData);

    vtkSmartPointer<vtkActor> actor =
      vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(
      colors->GetColor3d("Silver").GetData());

    // Visualize
    vtkSmartPointer<vtkRenderer> renderer =
      vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->SetWindowName("Polygon");
    renderWindow->AddRenderer(renderer);
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    renderer->AddActor(actor);
    renderer->SetBackground(colors->GetColor3d("Salmon").GetData());
    renderWindow->Render();
    renderWindowInteractor->Start();
};

//void VacancyVisualizeVTK::setStroke(const double &x){
//    stroke_width_ = x;
//}
//
//void VacancyVisualizeVTK::setViewBox(const Eigen::Vector2d& position, const double& width0, const double& height0){
//    min_x_ = position[0];
//    min_y_ = -position[1];
//    max_x_ = position[0] + width0;
//    max_y_ = -position[1] + height0;
//};
//
void VacancyVisualizeVTK::addRectangle(const Eigen::Vector2d& position, const double& width, const double& height){
//    context_->GetPen()->SetColor(0,0,0);
//    context_->DrawRect(position[0], position[1], width, height);
};
//
//void VacancyVisualizeVTK::addRectangles(const std::vector<Eigen::Vector2d>& positions, const double& width, const double& height){
//    for (const auto& p : positions){
//        addRectangle(p,width,height);
//    }
//};
//
//void VacancyVisualizeVTK::addCabbieCurveCollection(const CabbieCurveCollection& ccc){
//    int n_components = ccc.get_number_of_components();
//    for (int foo=0; foo < n_components; foo++){
//        std::stringstream ss;
//        ss << "<polyline points=\"";
//        int basepoint = ccc.get_basepoint(foo);
//        int bar = basepoint;
//        Eigen::Vector2d pbar = ccc.get_point(bar);
//        ss << pbar[0] << "," << -pbar[1] << " ";
//        do {
//            bar = ccc.get_next_point(bar);
//            pbar = ccc.get_point(bar);
//            ss << pbar[0] << "," << -pbar[1] << " ";
//            min_x_ = std::min(pbar[0], min_x_);
//            min_y_ = std::min(-pbar[1], min_y_);
//        } while (bar != basepoint);
//        ss << "\" stroke=\"black\" stroke-width=\""<< stroke_width_ << "\" fill=\"none\" />";
//        topstack.push_back(ss.str());
//    };
//}

//<polyline points="50,150 50,200 200,200 200,100" stroke="black" stroke-width="4" fill="none" />


//void VacancyVisualizeVTK::writeScalableVectorGraphics(const std::string& outfilepath){
//    auto svger = vtkSmartPointer<vtkSVGExporter>::New();
//    auto rendy = vtkSmartPointer<vtkRenderer>::New();
//    rendy->AddActor2D();
//    svger->SetInput(context_);
//    svger->SetFileName(outfilepath.c_str());
//    svger->WriteSVG();
//};
