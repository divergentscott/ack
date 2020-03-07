#include "bdfIO.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <vtkCellType.h>

namespace d3d {

namespace io {
namespace {
const int defaultPID = 1;

const auto triName = "CTRIA3  ";
const auto quadName = "CQUAD4  ";
const auto tetraName = "CTETRA  ";
const auto hexaName = "CHEXA   ";
const auto pyraName = "CPYRA   ";

const CellElement hexaElem =
    CellElement("CHEXA", VTK_HEXAHEDRON, 8, 3, CellType::CHEXA);
const CellElement tetraElem =
    CellElement("CTETRA", VTK_TETRA, 4, 3, CellType::CTETRA);
const CellElement quadElem =
    CellElement("CQUAD4", VTK_QUAD, 4, 2, CellType::CQUAD4);
const CellElement triElem =
    CellElement("CTRIA3", VTK_TRIANGLE, 3, 2, CellType::CTRIA3);
const CellElement pyraElem =
    CellElement("CPYRA", VTK_PYRAMID, 5, 3, CellType::CPYRA);

const std::vector<CellElement> allCellElements{hexaElem, tetraElem, quadElem,
                                               triElem, pyraElem};

const int standardCharSpace = 8;
const int extCharSpace = 16;

bool startsWithString(std::string line, std::string token) {
    return (line.substr(0, token.size()) == token);
}

}  // namespace

const char* getCellName(int nbDimension, int nbPoints) {
    if (nbDimension == 2) {
        if (nbPoints == 3) return triName;
        if (nbPoints == 4) return quadName;
    }
    if (nbDimension == 3) {
        if (nbPoints == 4) return tetraName;
        if (nbPoints == 5) return pyraName;
        if (nbPoints == 8) return hexaName;
    }

    return "UNDEFINED";
}

std::vector<int> findUniquePIDs(std::array<std::vector<int>, 4> array) {
    std::vector<int> vec;
    for (auto&& v : array) {
        vec.insert(vec.end(), v.begin(), v.end());
    }

    if (vec.size() == 0)
        vec = {defaultPID};
    else {
        std::sort(vec.begin(), vec.end());
        auto it = std::unique(vec.begin(), vec.end());
        vec.resize(std::distance(vec.begin(), it));
    }

    return vec;
}

D3D_status writeBDFFromCommonMeshData(CommonMeshData& mesh,
                                      const boost::filesystem::path& path) {
    if (mesh.gridPoints.size() == 0) {
        std::cout << "No vertices to write\n";
        return D3D_status::FAIL;
    }
    auto out = std::ofstream(path.string());
    if (!out.is_open()) return D3D_status::FAIL;
    int nMaterialId = 1;

    out << "TITLE = written by Divergent 3D system\n";
    out << "BEGIN BULK\n";

    out << "MAT1    1       3.0e+07         0.3300006.5e-06 5.4e+02\n";

    // if this is not empty, we need to make the connectivity refer to these ids
    if (mesh.gridIds.size() == 0) {
        std::cout << "Creating new indexing for the mesh grid points\n";
        for (auto ii = 0; ii < mesh.gridPoints.size(); ++ii) {
            mesh.gridIds.push_back(ii + 1);
        }
    }

    auto pids = findUniquePIDs(mesh.cellPIDs);
    std::for_each(pids.begin(), pids.end(), [&](int pid) {
        out << "PSHELL  " << std::left << std::setw(8) << pid << std::left
            << std::setw(8) << nMaterialId << std::left << std::setw(8)
            << 0.01 * pid << "\n";
    });

    // !! The bdf file format starts indexing at 1, so we need to add 1 to each
    // grid index
    for (auto dim = 0; dim < mesh.connectivity.size(); ++dim) {
        if (mesh.cellIds[dim].size() == 0) {
            for (auto ii = 0; ii < mesh.connectivity[dim].size(); ++ii) {
                mesh.cellIds[dim].push_back(ii + 1);
            }
        }

        if (mesh.connectivity[dim].size() > 0) {
            for (auto ii = 0; ii < mesh.connectivity[dim].size(); ++ii) {
                auto cellPID = mesh.cellPIDs[dim].size() > 0
                                   ? mesh.cellPIDs[dim][ii]
                                   : defaultPID;

                auto cell = mesh.connectivity[dim][ii];
                auto elemName = getCellName(dim, cell.size());

                out << elemName << std::left << std::setw(8)
                    << mesh.cellIds[dim][ii] << std::left << std::setw(8)
                    << cellPID;
                for (auto jj = 0; jj < cell.size(); ++jj) {
                    if (jj + 4 % 10 == 0) out << "\n        ";
                    out << std::left << std::setw(8) << mesh.gridIds[cell[jj]];
                }
                out << "\n";
            }
        }
    }
    out << "PMASS   4       0.100000\n";

    for (auto ii = 0; ii < mesh.gridPoints.size(); ++ii) {
        auto coords = mesh.gridPoints[ii];

        out.precision(9);
        out << std::scientific;
        out << "GRID*   " << std::left << std::setw(32) << mesh.gridIds[ii]
            << std::setw(16) << coords[0] << std::setw(16) << coords[1]
            << "*\n*       " << std::setw(16) << coords[2] << "\n";
    }
    out << "ENDDATA\n";
    out.close();
    return D3D_status::SUCCESS;
}

std::string trimWhitespace(const std::string& str) {
    size_t first = str.find_first_not_of(' ');
    if (std::string::npos == first) {
        return str;
    }
    size_t last = str.find_last_not_of(' ');
    return str.substr(first, (last - first + 1));
}

double parseDouble(std::string line) {
    // this parsing is specific to the short notation in bdf format (without e
    // for the exponent)
    line = trimWhitespace(line);
    auto it = std::find(line.begin() + 1, line.end(), '-');
    if (it == line.end()) it = std::find(line.begin() + 1, line.end(), '+');

    if (it != line.end()) {
        auto subLength = std::distance(line.begin(), it);
        auto ret =
            atof(line.substr(0, subLength).c_str()) *
            pow(10,
                atoi(
                    line.substr(subLength, line.length() - subLength).c_str()));
        return ret;
    }

    return atof(line.c_str());
}

D3D_status readGridPoints(std::ifstream& femFile, std::string line,
                          CommonMeshData& mesh) {
    std::string _;
    int countPoints = 0;
    const int nCoords = 3;
    std::array<double, nCoords> coords;
    int gridId;
    bool continueReading = true;
    try {
        while (continueReading) {
            if (startsWithString(line, "GRID ")) {
                gridId = atoi(line.substr(8, standardCharSpace).c_str());
                coords[0] = parseDouble(line.substr(24, standardCharSpace));
                coords[1] = parseDouble(line.substr(32, standardCharSpace));
                coords[2] = parseDouble(line.substr(40, standardCharSpace));
                mesh.gridPoints.push_back(coords);
                mesh.gridIds.push_back(gridId);
                countPoints++;
            } else if (startsWithString(line, "GRID*")) {
                gridId = atoi(line.substr(8, extCharSpace).c_str());
                coords[0] = atof(line.substr(40, extCharSpace).c_str());
                coords[1] = atof(line.substr(56, extCharSpace).c_str());
                getline(femFile, line);
                coords[2] = atof(line.substr(8, extCharSpace).c_str());
                mesh.gridPoints.push_back(coords);
                mesh.gridIds.push_back(gridId);
                countPoints++;
            } else if (startsWithString(line, "GRID")) {
                std::string field;
                std::istringstream iss(line);
                getline(iss, field, ',');  // GRID
                getline(iss, field, ',');  // point ID
                gridId = atoi(field.c_str());
                getline(iss, field, ',');  // other ID

                for (auto ii = 0; ii < nCoords; ++ii) {
                    getline(iss, field, ',');
                    coords[ii] = atof(field.c_str());
                }

                mesh.gridPoints.push_back(coords);
                mesh.gridIds.push_back(gridId);
                countPoints++;
            } else {
                break;
            }
            continueReading = getline(femFile, line).good();
        }
    } catch (...) {
        return D3D_status::CANNOT_READ_MESH;
    }

    return D3D_status::SUCCESS;
}

D3D_status readGridCells(
    std::ifstream& femFile, std::string line, d3d::CommonMeshData& mesh,
    std::map<int, int>& pointMap,
    std::array<std::vector<CommonMeshData::Cell>, 4>& tempConnectivity) {
    std::array<int, 4> cellNumbers;
    for (auto ii = 0; ii < cellNumbers.size(); ++ii) {
        cellNumbers[ii] = mesh.cellIds[ii].size();
    }

    bool continueReading = true;

    try {
        while (continueReading) {
            int charId = 8;
            bool ext;
            CellElement thisElemType;
            if (line[0] == '+') {
                continueReading = getline(femFile, line).good();
                continue;
            }
            if (line[0] != 'C') {
                break;
            }
            for (auto elemType : allCellElements) {
                if (startsWithString(line, elemType.name)) {
                    thisElemType = elemType;
                    ext = line[elemType.size] == '*';
                    break;
                }
            }

            if (thisElemType.defined) {
                int charSpace;
                charSpace = ext ? extCharSpace : standardCharSpace;

                auto cellId = atoi(line.substr(charId, charSpace).c_str());
                charId += charSpace;
                auto cellTag = atoi(line.substr(charId, charSpace).c_str());
                charId += charSpace;

                auto tempCellConnectivity =
                    CommonMeshData::Cell(thisElemType.numPoints);

                for (int ii = 0; ii < thisElemType.numPoints; ++ii) {
                    if (charId >= 72) {
                        getline(femFile,
                                line);  // skip to second line of coordinates
                        charId = 8;
                    }

                    auto gridId = atoi(line.substr(charId, charSpace).c_str());
                    tempCellConnectivity[ii] = gridId;
                    charId += charSpace;
                }

                tempConnectivity[thisElemType.dim].push_back(
                    tempCellConnectivity);

                mesh.cellIds[thisElemType.dim].push_back(cellId);
                mesh.cellPIDs[thisElemType.dim].push_back(cellTag);
                mesh.cellTypes[thisElemType.dim].push_back(thisElemType.type);

                ++cellNumbers[thisElemType.dim];
            } else {
                break;
            }

            continueReading = getline(femFile, line).good();
        }
    } catch (...) {
        return D3D_status::CANNOT_READ_MESH;
    }

    return D3D_status::SUCCESS;
}
D3D_status readBDFToCommonMeshData(const boost::filesystem::path& meshPath,
                                   CommonMeshData& mesh) {
    int _;
    return readBDFToCommonMeshData(meshPath, mesh, _);
}

D3D_status readBDFToCommonMeshData(const boost::filesystem::path& filePath,
	CommonMeshData& mesh, int& designDomainPID) {
	std::ifstream femFile(filePath.string());

	std::cout << "Read input file." << std::endl;

	std::map<int, int> pointMap;
	std::array<std::vector<CommonMeshData::Cell>, 4> tempConnectivity;

	auto ret_code = D3D_status::SUCCESS;

	if (femFile.is_open()) {
		std::string line;
		std::string _;

		while (getline(femFile, line)) {
			if (startsWithString(line, "DTPL")) {
				std::istringstream iss(line);
				iss >> _ >> _ >> _ >> designDomainPID;
				if (iss.fail()) {
					iss.clear();
					std::cout << "Design domain key could not be read."
						<< std::endl;
					designDomainPID = -1;
				}
			}
			if (startsWithString(line, "GRID")) {
				ret_code = readGridPoints(femFile, line, mesh);
				if (ret_code != D3D_status::SUCCESS) {
					return ret_code;
				}
				for (auto ii = 0; ii < mesh.gridIds.size(); ++ii) {
					pointMap[mesh.gridIds[ii]] = ii;
				}
			}

			if (std::any_of(allCellElements.begin(), allCellElements.end(),
				[line](CellElement e) {
				return startsWithString(line, e.name);
			})) {
				// we have no guarantee that the grid Ids have been read before.
				// Thats why we create a temp connectivity with the grid Ids,
				// and we create the real connectivity after having read
				// everything with the point map and the temp connectivity
				ret_code = readGridCells(femFile, line, mesh, pointMap,
					tempConnectivity);
				if (ret_code != D3D_status::SUCCESS) {
					return ret_code;
				}
			}
		}

		auto numCells = 0;
		std::for_each(
			tempConnectivity.begin(), tempConnectivity.end(),
			[&](std::vector<std::vector<int>> vec) { numCells += vec.size(); });

		std::cout << "Finished reading " << mesh.gridPoints.size()
			<< " points and " << numCells << " cells." << std::endl;
	}

	// the tempConnectivity data structure refers to the vertices by their fem
	// grid ids. We use the point map to instead make them refer to their ids in
	// the new vectors

	for (auto ndim = 0; ndim < tempConnectivity.size(); ++ndim) {
		mesh.connectivity[ndim] =
			std::vector<std::vector<int>>(tempConnectivity[ndim].size());
		for (auto ii = 0; ii < tempConnectivity[ndim].size(); ++ii) {
			auto newCell = std::vector<int>(tempConnectivity[ndim][ii].size());
			for (auto jj = 0; jj < tempConnectivity[ndim][ii].size(); ++jj) {
				newCell[jj] = pointMap[tempConnectivity[ndim][ii][jj]];
			}
			mesh.connectivity[ndim][ii] = newCell;
		}
	}

	return D3D_status::SUCCESS;
}


D3D_status readBDF(const boost::filesystem::path& filePath, BulkDataFileContents& bdf_contents) {
	//using mesh = bdf_contents.mesh;
    std::ifstream femFile(filePath.string());

    std::cout << "Read input file." << std::endl;

    std::map<int, int> pointMap;
    std::array<std::vector<CommonMeshData::Cell>, 4> tempConnectivity;

    auto ret_code = D3D_status::SUCCESS;

    if (femFile.is_open()) {
        std::string line;
        std::string _;
		std::string section_txt = "";
		int section_cnt = 0;

        while (getline(femFile, line)) {
			section_txt.append(line);

            if (startsWithString(line, "DTPL")) {
                std::istringstream iss(line);
                iss >> _ >> _ >> _ >> bdf_contents.designDomainPID;
                if (iss.fail()) {
                    iss.clear();
                    std::cout << "Design domain key could not be read."
                              << std::endl;
					bdf_contents.designDomainPID = -1;
                }
            }
            if (startsWithString(line, "GRID")) {
                ret_code = readGridPoints(femFile, line, bdf_contents.mesh);
                if (ret_code != D3D_status::SUCCESS) {
                    return ret_code;
                }
                for (auto ii = 0; ii < bdf_contents.mesh.gridIds.size(); ++ii) {
                    pointMap[bdf_contents.mesh.gridIds[ii]] = ii;
                }
            }

            if (std::any_of(allCellElements.begin(), allCellElements.end(),
                            [line](CellElement e) {
                                return startsWithString(line, e.name);
                            })) {
                // we have no guarantee that the grid Ids have been read before.
                // Thats why we create a temp connectivity with the grid Ids,
                // and we create the real connectivity after having read
                // everything with the point map and the temp connectivity
                ret_code = readGridCells(femFile, line, bdf_contents.mesh, pointMap,
                                         tempConnectivity);
                if (ret_code != D3D_status::SUCCESS) {
                    return ret_code;
                }
            }
        }

        auto numCells = 0;
        std::for_each(
            tempConnectivity.begin(), tempConnectivity.end(),
            [&](std::vector<std::vector<int>> vec) { numCells += vec.size(); });

        std::cout << "Finished reading " << bdf_contents.mesh.gridPoints.size()
                  << " points and " << numCells << " cells." << std::endl;
    }

    // the tempConnectivity data structure refers to the vertices by their fem
    // grid ids. We use the point map to instead make them refer to their ids in
    // the new vectors

    for (auto ndim = 0; ndim < tempConnectivity.size(); ++ndim) {
		bdf_contents.mesh.connectivity[ndim] =
            std::vector<std::vector<int>>(tempConnectivity[ndim].size());
        for (auto ii = 0; ii < tempConnectivity[ndim].size(); ++ii) {
            auto newCell = std::vector<int>(tempConnectivity[ndim][ii].size());
            for (auto jj = 0; jj < tempConnectivity[ndim][ii].size(); ++jj) {
                newCell[jj] = pointMap[tempConnectivity[ndim][ii][jj]];
            }
			bdf_contents.mesh.connectivity[ndim][ii] = newCell;
        }
    }

    return D3D_status::SUCCESS;
}



}  // namespace io

}  // namespace d3d
