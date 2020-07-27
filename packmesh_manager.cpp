#include "packmesh_manager.h"

#include <vtkAppendPolyData.h>

#include "planar_bounding_box.h"


void PackMeshManager::insertPackspace(const std::vector<Eigen::Vector2d>& grid_points, const int multiplicity) {
	zoning_board_.annexVacancy(grid_points, multiplicity);
};

void PackMeshManager::insertMesh(vtkSmartPointer<vtkPolyData> mesh, Eigen::Vector3d z_dir, int multiplicity) {
	meshes.push_back(mesh);
	z_directions.push_back(z_dir);
	multiplicities.push_back(multiplicity);
	PlanarBoxBounder pbb;
	pbb.setMesh(mesh);
	pbb.projection_normal_ = z_dir;
	pbb.projectToHull();
	rectangles.push_back({pbb.width, pbb.height});
	mesh_transforms.push_back(pbb.transform_);
	//rectangles.push_back()
};

void PackMeshManager::generatePlacementTransforms() {
	placement_transforms.resize(zoning_board_.placements_.size());
	for (auto foo = 0; foo < meshes.size(); foo++) {
		const std::vector<Placement>& places_foo = zoning_board_.placements_[foo];
		const Eigen::Vector2d& rect_foo = rectangles[foo];
		const Eigen::Matrix4d& m_base_transform = mesh_transforms[foo];
		std::vector<Eigen::Matrix4d>& placement_transforms_foo = placement_transforms[foo];
		placement_transforms_foo.reserve(places_foo.size());
		for (const auto & place : places_foo) {
			Eigen::Matrix4d shift_place = Eigen::Matrix4d::Identity();
			if (place.rotated) {
				shift_place.block(0, 0, 2, 2) << 0, -1, 1, 0;
				shift_place.block(0, 3, 2, 1) = place.position + Eigen::Vector2d({ rect_foo[1],0 });
			}
			else {
				shift_place.block(0, 3, 2, 1) = place.position;
			}
			placement_transforms_foo.push_back(shift_place * m_base_transform);
		}
	}
}


void PackMeshManager::pack() {
	//!!!! Add multi vacancy support
	zoning_board_.setApplicantRectangles(rectangles, multiplicities);
	zoning_board_.allow_rotations = true;
	zoning_board_.zone();
	generatePlacementTransforms();
};

vtkSmartPointer<vtkPolyData> PackMeshManager::getPackMesh() {
	vtkSmartPointer<vtkPolyData> packspace;
	auto appender = vtkSmartPointer<vtkAppendPolyData>::New();
	for (auto foo = 0; foo < meshes.size(); foo++) {
		//
		vtkSmartPointer<vtkPolyData> mesh_foo = meshes[foo];
		const Eigen::Vector2d& rect_foo = rectangles[foo];
		const std::vector<Placement>& places_foo = zoning_board_.placements_[foo];
		const std::vector<Eigen::Matrix4d> & placements_transforms_foo = placement_transforms[foo];
		//
		for (auto bar = 0; bar < places_foo.size(); bar++) {
			//transform the mesh
			const Eigen::Matrix4d& tt = placements_transforms_foo[bar];
			auto mesh_bar = applyAffineTransform( mesh_foo, tt);
			appender->AddInputData(mesh_bar);
			//slap in the bounding box
			const Eigen::Vector2d& place_bar = places_foo[bar].position;
			Eigen::Vector3d place = { place_bar[0], place_bar[1], 0 };
			Eigen::Vector3d xax;
			Eigen::Vector3d yax;
			if (places_foo[bar].rotated) {
				xax = { rect_foo[1], 0, 0};
				yax = { 0, rect_foo[0], 0};
			}
			else {
				xax = { rect_foo[0], 0, 0 };
				yax = { 0, rect_foo[1], 0 };
			}
			auto rect_bar = makePolydataRectangle(place, xax, yax);
			appender->AddInputData(rect_bar);
		}
	}
	appender->Update();
	packspace = appender->GetOutput();
	return packspace;
};