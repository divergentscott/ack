#ifndef planar_bounding_box_hpp_0110
#define planar_bounding_box_hpp_0110

#include <Eigen/Dense>

#include <vtkPolyData.h>

vtkSmartPointer<vtkPolyData> applyAffineTransform(vtkSmartPointer<vtkPolyData> mesh, const Eigen::Matrix4d & transform);

vtkSmartPointer<vtkPolyData> makePolydataRectangle(const Eigen::Vector3d& origin, const Eigen::Vector3d& basis0, const Eigen::Vector3d& basis1);

struct PlanarBoxBounder{
    vtkSmartPointer<vtkPolyData> surface_;
    // Should this be a unstructured grid?
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
    
	void setMesh(vtkSmartPointer<vtkPolyData> mesh);
    void projectToHull();
    Eigen::Matrix4d computeMeshFrameTransform();
    vtkSmartPointer<vtkPolyData> getTransformedSurface();
};

#endif
