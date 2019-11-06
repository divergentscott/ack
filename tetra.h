#ifndef D3DAPI_H_BREEZIN_CPP_D3TOOLS_TETRA
#define D3DAPI_H_BREEZIN_CPP_D3TOOLS_TETRA

//#include "d3dapi.h"
#include "vector"
#include <boost/filesystem.hpp>
#include "iostream"

enum D3D_status {
	SUCCESS=0,
	CANNOT_OPEN_FILE,
	ERROR_TETGEN_SELF_INTERSECT,
	ERROR_TETGEN_CLOSE_FACETS,
	ERROR_TETGEN_UNKNOWN_FAILURE,
	ERROR_TETGEN_OUT_OF_MEMORY,
	ERROR_TETGEN_SMALL_FEATURE
};

using pMesh = double[];

namespace d3d {
	namespace tetra {
		struct tetgenio_data_list {
			// Each point is 3 doubles
			std::vector<double> points;
			// Every 2 point ids is one edge
			std::vector<int> edges;
			// Every 3 point ids is one triangle
			std::vector<int> triangles;
			// Every 4 point ids is a tetrahedron
			std::vector<int> tetrahedra;
			// tetrahedronregion[cell_id]=region id
			std::vector<int> tetrahedronregion;
		};

		class TetGeneral {
		public:
			double getVolume() {};
			double getEulerCharacteristic() {};
			double getInputMeshPointer() {};
		};

		TetGeneral appointTetGeneral(pMesh mesh_in);

		TetGeneral appointTetGeneral(tetgenio_data_list mesh_in);



		double volume(tetgenio_data_list x);

		int euler_characteristic(tetgenio_data_list x);

		tetgenio_data_list tetgen_tetrahedralize(std::string switches,
			tetgenio_data_list mesh_in);

		tetgenio_data_list tetrahedralize_with_hollows(tetgenio_data_list mesh_in,
			double volume_constraint,
			double quality_constraint);

		D3D_status tet_and_write_mesh(pMesh me, boost::filesystem::path filename,
			double volume_constraint,
			double quality_constraint);

	}  // namespace tetra
}  // namespace d3d
#endif
