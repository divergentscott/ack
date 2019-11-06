#include <boost/filesystem.hpp>
#include "iostream"
#include "tetra.h"

int main() {
	boost::filesystem::path spherepath = "C:\Users\sscott\Pictures\unitsphere_meshlab";
	std::cout << spherepath.extension();
	if (spherepath.extension() == ".stl") std::cout << "I Saw The Sign.";
}