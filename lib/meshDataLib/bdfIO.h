#ifndef MESHDATALIB_BDFIO_H
#define MESHDATALIB_BDFIO_H

#include <boost/filesystem.hpp>
#include <vector>
#include "utilities/d3derr.h"
#include "meshData.h"

namespace d3d {
namespace io {

struct BulkDataFileContents {
	std::vector<std::string> bonusSections;
	CommonMeshData mesh;
	int designDomainPID;
	int meshSectionPosition;
	std::vector<RigidBodyElement> rigidBodyElements;
	int rigidBodySectionPosition;
};

D3D_status writeBDFFromCommonMeshData(CommonMeshData& mesh,
                        const boost::filesystem::path& path);

D3D_status readBDFToCommonMeshData(const boost::filesystem::path& meshPath,
                         CommonMeshData& mesh, int& designDomainPID);

D3D_status readBDFToCommonMeshData(const boost::filesystem::path& meshPath,
                         CommonMeshData& mesh);

D3D_status readBDF(const boost::filesystem::path& filePath, BulkDataFileContents contents);

}  // namespace io

}  // namespace d3d

#endif