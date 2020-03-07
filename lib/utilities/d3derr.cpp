#include "d3derr.h"
#include <iostream>

namespace {
class D3D_status_category_t : public std::error_category {
   public:
    const char* name() const noexcept override;
    std::string message(int ev) const override;
};

const D3D_status_category_t D3D_status_category{};

const char* D3D_status_category_t::name() const noexcept {
    return "D3D Error code";
}

std::string D3D_status_category_t::message(int ev) const {
    switch (static_cast<D3D_status>(ev)) {
        case D3D_status::SUCCESS:
            return "Operation successful";
        case D3D_status::FAIL:
            return "Operation failed";
        case D3D_status::UNKNOWN_OPTIONS:
            return "Unknown input options";
        case D3D_status::MISSING_INPUT:
            return "Missing input file";
        case D3D_status::MISSING_FILE_EXTENSION:
            return "Please include a valid file extension (fem, stl, dat, bdf)";
        case D3D_status::CANNOT_READ_MESH:
            return "Cannot read input mesh";
        case D3D_status::CANNOT_OPEN_FILE:
            return "Error while trying to open file - file might not exist";
        case D3D_status::CANNOT_WRITE_FILE:
            return "Error while trying to write file";
        case D3D_status::ERROR_INVALID_MESH_INPUT:
            return "Error while reading mesh, mesh does not seem to be valid.";
        case D3D_status::ERROR_COLINEAR_TRIANGLE:
            return "Error - encountered colinear triangle";
        case D3D_status::ERROR_TETGEN_OUT_OF_MEMORY:
            return "Error - Tetgen failed because it ran out of memory";
        case D3D_status::ERROR_TETGEN_UNKNOWN_FAILURE:
            return "Error - Tetgen failed for an unknown reason";
        case D3D_status::ERROR_TETGEN_SELF_INTERSECT:
            return "Error - Tetgen failed because of self-intersections";
        case D3D_status::ERROR_TETGEN_SMALL_FEATURE:
            return "Error - Tetgen failed because of a small feature";
        case D3D_status::ERROR_TETGEN_CLOSE_FACETS:
            return "Error - Tetgen failed because of close facets";
        case D3D_status::ERROR_IN_COMPARISON:
            return "Error - Comparison showed differences between meshes";
        case D3D_status::FAIL_CLPS_DUP_EDGE:
            return "Failure of collapse operation because of a duplicate edge "
                   "(?)";
        case D3D_status::FAIL_CLPS_FLIP_NORMAL:
            return "Failure of collapse operation because of a flipped normal";
        case D3D_status::FAIL_CLSP_SHAPE_DECREASE:
            return "Failure of collapse operation because of a shape decrease "
                   "(?)";
        case D3D_status::FAIL_NON_MANIFOLD:
            return "Failure because of non manifold mesh";
        case D3D_status::FAIL_PATCH_HOLES:
            return "Failure - there are 2d holes in the mesh";
        case D3D_status::FAIL_SMALL_DIMENSION:
            return "Failed because of small dimensions, please check units";
        case D3D_status::FAIL_COTWEIGHT:
            return "Failed because of mesh quality";
        case D3D_status::FAIL_CLPS_BRIDGE_GEDGE:
            return "Failure of collapse operation because of bridge g edge (?)";
        case D3D_status::FAIL_SWAP_DUP_EDGE:
            return "Failure of swap operation because of a duplicate edge";
        case D3D_status::FAIL_NORM_TOO_SMALL:
            return "Failure because a calculated norm was smaller than the "
                   "tolerance";
        case D3D_status::ERROR_NUMERICAL_TOLERANCE:
            return "Failure because numerical calculation outside of "
                   "tolerance";
        case D3D_status::PID_NOT_FOUND:
            return "Specified PID was not found in file";
        default:
            return "(unrecognized error)";
    }
}

}  // namespace
std::error_code make_error_code(D3D_status e) {
    return {static_cast<int>(e), D3D_status_category};
}

int D3D_return(D3D_status status_code) {
    if (status_code == D3D_status::SUCCESS_AND_DONE)
        status_code = D3D_status::SUCCESS;
    auto ec = std::error_code(status_code);
    std::cout << ec.message().c_str() << "\n";
    return ec.value();
}
