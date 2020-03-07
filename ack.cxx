#include "iostream"
#include "string"

#include <boost/filesystem.hpp>

#include "lib\meshDataLib\bdfIO.h"

int main() {
	std::cout << "\n\nSALUTON MUNDO.\n\n";
	using namespace d3d;
	boost::filesystem::path filePath = "C:\\Users\\sscott\\Documents\\model_01_carm.fem";
	CommonMeshData mesh;
	std::vector<RigidBodyElement> rbes;
	std::vector<std::string> bonus_texts;
	int designDomainPid;
	io::BulkDataFileContents bdf_contents;
	auto status = io::readBDF(filePath, bdf_contents);
}