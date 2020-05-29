#ifndef MESHDATALIB_BDFIO_H
#define MESHDATALIB_BDFIO_H

#include <vector>

#include <boost/filesystem.hpp>

#include "utilities/d3derr.h"
#include "meshData.h"


namespace d3d {
namespace io {

struct BulkDataFileContents {
	std::vector<std::string> bonusSections; //Unparsed text storage. Does not include the parsed sections.
	CommonMeshData mesh; 
	int designDomainPID;
	int pidSectionPosition = -1; //place of the cell section in the text section list
	int cellSectionPosition = -1; //place of the cell section in the text section list
	int pointSectionPosition = -1; //place of the gridpoint section in the text section list
	std::vector<RigidBodyElement> rigidBodyElements;
	int rigidBodySectionPosition = -1;
};

D3D_status writeBDFFromCommonMeshData(CommonMeshData& mesh,
                        const boost::filesystem::path& path);

D3D_status readBDFToCommonMeshData(const boost::filesystem::path& meshPath,
                         CommonMeshData& mesh, int& designDomainPID);

D3D_status readBDFToCommonMeshData(const boost::filesystem::path& meshPath,
                         CommonMeshData& mesh);

D3D_status readBDF(const boost::filesystem::path& filePath, BulkDataFileContents& contents);

D3D_status writeBDF(const boost::filesystem::path& filePath, BulkDataFileContents& bdf_contents);
}  // namespace io

}  // namespace d3d

#endif