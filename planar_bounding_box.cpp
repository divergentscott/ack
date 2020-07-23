#include "planar_bounding_box.h"

#include <vtkCenterOfMass.h>
#include <vtkConvexHull2D.h>
#include <vtkIdList.h>
#include <vtkOBBTree.h>
#include <vtkOutlineFilter.h>
#include <vtkPoints.h>
#include <vtkQuad.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkUnstructuredGrid.h>

static const double repsilon = std::sqrt(std::numeric_limits<double>::epsilon());


vtkSmartPointer<vtkPolyData> makePolydataRectangle(const Eigen::Vector3d& origin, const Eigen::Vector3d& basis0, const Eigen::Vector3d& basis1){
    auto box_pts = vtkSmartPointer<vtkPoints>::New();
    box_pts ->InsertNextPoint(origin.data());
    Eigen::Vector3d c1 = (origin + basis0);
    box_pts->InsertNextPoint(c1.data());
    Eigen::Vector3d c2 = (origin + basis0 + basis1);
    box_pts->InsertNextPoint(c2.data());
    Eigen::Vector3d c3 = (origin + basis1);
    box_pts->InsertNextPoint(c3.data());
    //

    auto quad = vtkSmartPointer<vtkQuad>::New();
    quad->GetPointIds()->SetId(0,0);
    quad->GetPointIds()->SetId(1,1);
    quad->GetPointIds()->SetId(2,2);
    quad->GetPointIds()->SetId(3,3);
    auto cellarray = vtkSmartPointer<vtkCellArray>::New();
    cellarray->InsertNextCell(quad);
    //
    auto square_polydata = vtkSmartPointer<vtkPolyData>::New();
    square_polydata->SetPoints(box_pts);
    square_polydata->SetPolys(cellarray);
    //
    return square_polydata;
}

Eigen::Matrix4d PlanarBoxBounder::computeMeshFrameTransform(){
    Eigen::Matrix4d rotT = Eigen::Matrix4d::Identity();
    Eigen::Vector3d zz = {0.0,0.0,1.0};
    if ((projection_normal_ - zz).norm() > repsilon){
        Eigen::Vector3d xx = (projection_normal_.cross(zz)).normalized();
        Eigen::Vector3d yy = (projection_normal_.cross(xx)).normalized();
        rotT.block(0,0,3,1) = xx;
        rotT.block(0,1,3,1) = yy;
        rotT.block(0,2,3,1) = projection_normal_;
    }
    Eigen::Vector3d orig;
    vtkCenterOfMass::ComputeCenterOfMass(surface_->GetPoints(), nullptr,
                                         orig.data());
    
    //compute the lowest
    double lowest = std::numeric_limits<double>::max();
    double highest = std::numeric_limits<double>::lowest();
    for (auto foo=0; foo< surface_->GetNumberOfPoints(); foo++){
        Eigen::Vector3d pnt;
        surface_->GetPoint(foo, pnt.data());
        double height = projection_normal_.dot(pnt - orig);
        lowest = std::min(height,lowest);
        highest = std::max(height, highest);
    }
    z_span_ = {lowest, highest};

    Eigen::Matrix4d mesh_frame = Eigen::Matrix4d::Identity();
    mesh_frame.block(0,3,3,1) = -orig;
    mesh_frame = rotT.transpose() * mesh_frame;
    return mesh_frame;
};

void PlanarBoxBounder::projectToHull(){
    projection_normal_.normalize();
    transform_ = computeMeshFrameTransform();
    Eigen::Matrix4d  proj_mat = Eigen::Matrix4d::Identity();
    proj_mat(2,2) = 0.0;
    proj_mat = proj_mat * transform_;
    
    auto planar_pts = vtkSmartPointer<vtkPoints>::New();
    for (auto foo = 0; foo < surface_->GetNumberOfPoints(); foo++) {
        Eigen::Vector4d non_planar_pt = {0.0,0.0,0.0,1.0};
        surface_->GetPoints()->GetPoint(foo, non_planar_pt.data());
        Eigen::Vector4d planar_pt = proj_mat * non_planar_pt;
        planar_pts->InsertNextPoint(planar_pt.data());
    }
    auto planar_polydata = vtkSmartPointer<vtkPolyData>::New();
    planar_polydata->SetPoints(planar_pts);
    //Extract the convext hull of the planar projection
    auto huller = vtkSmartPointer<vtkConvexHull2D>::New();
    huller->SetInputData(planar_polydata);
    huller->Update();
    hull_ = huller->GetOutput();
    Eigen::Vector3d corner, longest_side, middlest_side, _lilside, _size;
    auto obbtree = vtkSmartPointer<vtkOBBTree>::New();
    obbtree->ComputeOBB(hull_, corner.data(), longest_side.data(), middlest_side.data(), _lilside.data(), _size.data());
    //
    corner_ = corner;
    box_x_dir_ = longest_side;
    box_y_dir_ = middlest_side;
    // w by h //
    width = longest_side.norm();
    height = middlest_side.norm();
    //shift rectangle center to origin
    Eigen::Vector3d rect_center = corner + 0.5 * longest_side + 0.5 * middlest_side;
    Eigen::Matrix4d shift_rc = Eigen::Matrix4d::Identity();
    shift_rc.block(0,3,3,1) = -rect_center;
    transform_ = shift_rc * transform_;
    //rotation to align longest direction to x
    Eigen::Matrix4d rot_rc = Eigen::Matrix4d::Identity();
    Eigen::Vector3d lhat = longest_side.normalized();
    rot_rc(0,0) = lhat[0];
    rot_rc(0,1) = lhat[1];
    rot_rc(1,0) = -lhat[1];
    rot_rc(1,1) = lhat[0];
    transform_ = rot_rc * transform_;
    //Fix the z translation
    Eigen::Matrix4d shift_z = Eigen::Matrix4d::Identity();
    Eigen::Vector3d tpn = transform_.block(0,0,3,3) * projection_normal_;
    if (tpn[2] > 0){
        //The projection normal is sent to positive z.
        shift_z(2,3) = -z_span_[0];
    } else {
        //The projection normal is sent to negative z.
        shift_z(1,1) = -1;
        shift_z(2,2) = -1;
        shift_z(2,3) = -z_span_[1];
    }
    transform_ = shift_z * transform_;
    // shift bottom left corner to origin
    // result should be in strictly positive octant
    Eigen::Vector3d bottom_left = { - 0.5 * width,  - 0.5 * height, 0};
    Eigen::Matrix4d shift_bl = Eigen::Matrix4d::Identity();
    shift_bl.block(0,3,3,1) = -bottom_left;
    transform_ = shift_bl * transform_;
};

vtkSmartPointer<vtkPolyData> applyAffineTransform(vtkSmartPointer<vtkPolyData> mesh, const Eigen::Matrix4d & transform) {
	auto vtk_transform = vtkSmartPointer<vtkTransform>::New();
	Eigen::Matrix4d transposed = transform.transpose();

	vtk_transform->SetMatrix(transposed.data());

	auto transformer = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformer->SetTransform(vtk_transform);
	transformer->SetInputData(mesh);
	transformer->Update();
	return transformer->GetOutput();
};

vtkSmartPointer<vtkPolyData> PlanarBoxBounder::getTransformedSurface(){
	return applyAffineTransform(surface_, transform_);
};

void PlanarBoxBounder::setMesh(vtkSmartPointer<vtkPolyData> mesh) {
	surface_ = mesh;
};