
#include "meshData.h"

namespace d3d {

bool isIntInVector(int x, std::vector<int> vec) {
    auto it = std::find(vec.begin(), vec.end(), x);
    return (it != vec.end()) ? true : false;
}

D3D_status extractMeshDataByPID(const CommonMeshData &mesh_in,
                                std::vector<int> pids,
                                CommonMeshData &mesh_out) {
    // mesh_out.gridIds = mesh_in.gridIds;
    mesh_out.gridPoints = mesh_in.gridPoints;

    bool pid_found = false;

    try {
        for (auto dim = 0; dim < mesh_in.connectivity.size(); ++dim) {
            for (auto ii = 0; ii < mesh_in.connectivity[dim].size(); ++ii) {
                if (mesh_in.cellPIDs[dim].size() > ii)
                    if (isIntInVector(mesh_in.cellPIDs[dim][ii], pids)) {
                        pid_found = true;
                        if (mesh_in.cellIds[dim].size() > ii)
                            mesh_out.cellIds[dim].push_back(
                                mesh_in.cellIds[dim][ii]);
                        mesh_out.cellPIDs[dim].push_back(
                            mesh_in.cellPIDs[dim][ii]);
                        mesh_out.connectivity[dim].push_back(
                            mesh_in.connectivity[dim][ii]);
                    }
            }
        }
    } catch (...) {
        return D3D_status::FAIL;
    }

    if (!pid_found) return D3D_status::PID_NOT_FOUND;

    return D3D_status::SUCCESS;
}

D3D_status overwriteMeshDataPIDs(CommonMeshData &mesh_to_modify,
                                 const CommonMeshData &source_mesh,
                                 const std::vector<int> &pids) {
    bool pid_found = false;

    try {
        auto it_to_max_grid_id = std::max_element(
            mesh_to_modify.gridIds.begin(), mesh_to_modify.gridIds.end());

        auto new_grid_id = it_to_max_grid_id != mesh_to_modify.gridIds.end()
                               ? *(it_to_max_grid_id) + 1
                               : 0;

        CommonMeshData new_mesh;
        new_mesh.gridIds = mesh_to_modify.gridIds;
        new_mesh.gridPoints = mesh_to_modify.gridPoints;

        std::vector<int> localPointIndex;

        // copy grid points from the source mesh and make a mapping between the
        // new grid ids and the original ones
        for (auto ii = 0; ii < source_mesh.gridPoints.size(); ++ii) {
            localPointIndex.push_back(new_mesh.gridPoints.size());
            new_mesh.gridPoints.push_back(source_mesh.gridPoints[ii]);
            new_mesh.gridIds.push_back(new_grid_id);
            new_grid_id++;
        }

        // copy all cells from mesh_to_modify that have a different PID than the
        // input PIDs
        for (auto dim = 0; dim < mesh_to_modify.connectivity.size(); ++dim) {
            for (auto ii = 0; ii < mesh_to_modify.connectivity[dim].size();
                 ++ii) {
                if (mesh_to_modify.cellPIDs[dim].size() > ii)
                    if (!isIntInVector(mesh_to_modify.cellPIDs[dim][ii],
                                       pids)) {
                        new_mesh.cellPIDs[dim].push_back(
                            mesh_to_modify.cellPIDs[dim][ii]);

                        new_mesh.connectivity[dim].push_back(
                            mesh_to_modify.connectivity[dim][ii]);
                    }
            }
        }

        // copy all cells from source_mesh that have a PID included in the input
        // PIDs.

        for (auto dim = 0; dim < source_mesh.connectivity.size(); ++dim) {
            for (auto ii = 0; ii < source_mesh.connectivity[dim].size(); ++ii) {
                if (source_mesh.cellPIDs[dim].size() > ii)
                    if (isIntInVector(source_mesh.cellPIDs[dim][ii], pids)) {
                        pid_found = true;

                        new_mesh.cellPIDs[dim].push_back(
                            source_mesh.cellPIDs[dim][ii]);

                        std::vector<int> cell;
                        for (auto id : source_mesh.connectivity[dim][ii])
                            cell.push_back(localPointIndex[id]);
                        new_mesh.connectivity[dim].push_back(cell);
                    }
            }
        }

        mesh_to_modify = new_mesh;

    } catch (...) {
        return D3D_status::FAIL;
    }

    if (!pid_found) return D3D_status::PID_NOT_FOUND;

    return D3D_status::SUCCESS;
}

}  // namespace d3d
