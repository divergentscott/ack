#ifndef packmesh_manager_hpp
#define packmesh_manager_hpp

#include <Eigen/Dense>

#include <vtkPolyData.h>

#include "zoning_board.h"


struct PackMeshManager{
	ZoningBoard zoning_board_;
	std::vector<vtkSmartPointer<vtkPolyData>> meshes;
	std::vector<Eigen::Vector3d> z_directions;
	std::vector<int> multiplicities;
	std::vector<Eigen::Vector2d> rectangles;
	std::vector<Eigen::Matrix4d> mesh_transforms;
	std::vector<std::vector<Eigen::Matrix4d>> placement_transforms;
	PackMeshManager(){};
	void generatePlacementTransforms();
	void insertMesh(vtkSmartPointer<vtkPolyData> mesh, Eigen::Vector3d z_dir = {0,0,1}, int multiplicity = 1);
	void insertPackspace(const std::vector<Eigen::Vector2d>& grid_points, const int multiplicity = 1);
	void pack();
	vtkSmartPointer<vtkPolyData> getPackMesh();

	//private:
//public:
//	ZoningStratgey strategy_ = ZoningStratgey::kBottomLeft;
//	bool allow_rotations = false;
//	std::vector<Applicant> applicants_; // rectangles to pack width x height
//	std::vector<int> vacancy_clone_parent_ids_; // Points to the vacancy that a clone came from.
//	std::vector<std::vector<Placement>> placements_; //indexed by applicant number
//	std::vector<Vacancy> vacancies_;
//	std::vector<Eigen::Vector2d> denials_;
//	void sortApplicants();
//
//public:
//	ZoningBoard() {};
//	// Negative multiplicities are used to indicate no limit.
//	void annexVacancy(const std::vector<Eigen::Vector2d>& grid_points,
//		const int multiplicity = 1);
//	void annexVacancy(const std::vector<Eigen::Vector2d>& grid_points,
//		const std::vector<std::vector<int>> &lines,
//		const int multiplicity = 1);
//
//	void setApplicantRectangles(const std::vector<Eigen::Vector2d>& rectangles);
//	void setApplicantRectangles(const std::vector<Eigen::Vector2d>& rectangles, const std::vector<int>& multiplicites);
//	void zone();
//
};

#endif
