#include "iostream"
#include "string"

#include <boost/filesystem.hpp>

#include "lib\meshDataLib\bdfIO.h"

bool startsWithString(std::string line, std::string token) {
	return (line.substr(0, token.size()) == token);
}

int main() {
	std::cout << "\n\nSALUTON MUNDO.\n\n";
	using namespace d3d;
	//boost::filesystem::path filePath = "C:\\Users\\sscott\\Documents\\rbe_transfer\\simplify.fem";
	boost::filesystem::path filePath = "C:\\Users\\sscott\\Documents\\rbe_transfer\\model_01_carm.fem";
	std::ifstream filestream_(filePath.c_str());
	std::string line;
	int designDomainPid;
	io::BulkDataFileContents bdf_contents;
	auto status = io::readBDF(filePath, bdf_contents);
	for (auto s : bdf_contents.bonusSections) {
		std::cout << "@@@SeCtIoN cHaNgE@@@" << std::endl << std::endl << s << std::endl << std::endl;
	}
	boost::filesystem::path outFilePath = "C:\\Users\\sscott\\Documents\\rbe_transfer\\try_out.fem";
	auto status2 = io::writeBDF(outFilePath, bdf_contents);
}
