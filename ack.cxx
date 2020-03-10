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
	boost::filesystem::path filePath = "C:\\Users\\sscott\\Documents\\model_01_carm.fem";
	std::ifstream filestream_(filePath.c_str());
	std::string line;

	//while (std::getline(filestream_, line)) {
	//	int foo = 0;
	//	if (startsWithString(line,"G")) {
	//		do {
	//			if (!startsWithString(line, "G")) break;
	//		} while (std::getline(filestream_, line));
	//		foo++;
	//		std::cout << std::endl << foo << std::endl;
	//	}
	//	if (startsWithString(line, "C")) {
	//		do {
	//			if (!(startsWithString(line, "C")|| startsWithString(line, "+"))) break;
	//		} while (std::getline(filestream_, line));
	//		foo++;
	//	std::cout << std::endl << foo << std::endl;
	//	}
	//}


	CommonMeshData mesh;
	std::vector<RigidBodyElement> rbes;
	std::vector<std::string> bonus_texts;
	int designDomainPid;
	io::BulkDataFileContents bdf_contents;
	auto status = io::readBDF(filePath, bdf_contents);
	for (auto s : bdf_contents.bonusSections) {
		std::cout << "@@@SeCtIoN cHaNgE@@@" << std::endl << std::endl << s << std::endl << std::endl;
	}
	//fil
}
