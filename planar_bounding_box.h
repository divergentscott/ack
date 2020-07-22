#ifndef planar_bounding_box_hpp_0110
#define planar_bounding_box_hpp_0110

#include <Eigen/Dense>

#include <vtkPolyData.h>

struct PlanarBoxBounder{
    vtkSmartPointer<vtkPolyData> surface_;
    //Probably want this to be a ugrid not a polydata
    Eigen::Vector2d z_span_;
    vtkSmartPointer<vtkPolyData> hull_;
    vtkSmartPointer<vtkPolyData> rectangle_;
    Eigen::Vector3d projection_normal_ = {0,0,1};
    Eigen::Vector3d corner_;
    Eigen::Vector3d box_x_dir_;
    Eigen::Vector3d box_y_dir_;
    Eigen::Matrix4d transform_;
    double width;
    double height;
    
    void projectToHull();
    vtkSmartPointer<vtkPolyData> makePolydataRectangle(Eigen::Vector3d origin, Eigen::Vector3d basis0, Eigen::Vector3d basis1);
    Eigen::Matrix4d computeMeshFrameTransform();
    vtkSmartPointer<vtkPolyData> getTransformedSurface();
};

#endif
