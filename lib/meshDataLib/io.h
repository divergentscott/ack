#ifndef MESHDATALIB_IO_H
#define MESHDATALIB_IO_H

#include <boost/filesystem.hpp>
#include <vector>
#include "utilities/d3derr.h"
#include "meshData.h"

namespace d3d {
namespace io {
// 2D Data readers and writers
D3D_status readOBJToCommonMeshData(const boost::filesystem::path& meshPath,
                                     CommonMeshData& mesh);
D3D_status writeOBJFromCommonMeshData(CommonMeshData& mesh,
                                      const boost::filesystem::path& meshPath);

D3D_status readSTLToCommonMeshData(const boost::filesystem::path& meshPath,
                                     CommonMeshData& mesh);
D3D_status writeSTLFromCommonMeshData(CommonMeshData& mesh,
                                      const boost::filesystem::path& meshPath);

D3D_status readVTPToCommonMeshData(const boost::filesystem::path& meshPath,
                                     CommonMeshData& mesh);

D3D_status writeVTPFromCommonMeshData(CommonMeshData& mesh,
                                      const boost::filesystem::path& meshPath);

// 3D Data readers and writers
D3D_status readVTUToCommonMeshData(const boost::filesystem::path& meshPath,
                                     CommonMeshData& mesh);
D3D_status writeVTUFromCommonMeshData(CommonMeshData& mesh,
                                      const boost::filesystem::path& meshPath);

}  // namespace io

}  // namespace d3d

#endif
