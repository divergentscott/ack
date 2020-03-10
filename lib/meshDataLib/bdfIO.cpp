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

class BulkDataFileParser {
public:
	enum class BulkDataFileCard {
		UNKNOWN,
		GRIDPOINTS,
		CELLS,
		RIGIDBODYELEMENTS
	};

	private:
		std::string line_;
		std::ifstream filestream_;

	public:
		const int defaultPID = 1;
		//
		const std::string triName = "CTRIA3  ";
		const std::string quadName = "CQUAD4  ";
		const std::string tetraName = "CTETRA  ";
		const std::string hexaName = "CHEXA   ";
		const std::string pyraName = "CPYRA   ";
		//
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
		//
		const std::vector<CellElement> allCellElements{ hexaElem, tetraElem, quadElem,
													   triElem, pyraElem };
		//
		const int standardCharSpace = 8;
		const int extCharSpace = 16;
		const int CONST_COL_WIDTH = 72; //I hope this column width doesn't change.

		//
		bool startsWithString(std::string line, std::string token) {
			return (line.substr(0, token.size()) == token);
		}

		bool startsWithCell(std::string line) {
			return std::any_of(allCellElements.begin(),
				               allCellElements.end(),
							   [&](CellElement e) {
				return startsWithString(line, e.name);
			});
		};
		//
		const std::string getCellName(int nbDimension, int nbPoints) {
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
		//
		std::vector<int> findUniquePIDs(std::array<std::vector<int>, 4> array) {
			std::vector<int> vec;
			for (auto&& v : array) {
				vec.insert(vec.end(), v.begin(), v.end());
			}

			if (vec.size() == 0)
				vec = { defaultPID };
			else {
				std::sort(vec.begin(), vec.end());
				auto it = std::unique(vec.begin(), vec.end());
				vec.resize(std::distance(vec.begin(), it));
			}

			return vec;
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

		D3D_status readGridPoints(CommonMeshData& mesh) {
			std::string _;
			int countPoints = 0;
			const int nCoords = 3;
			std::array<double, nCoords> coords;
			int gridId;
			bool continueReading = true;
			try {
				do {
					if (startsWithString(line_, "GRID ")) {
						gridId = atoi(line_.substr(8, standardCharSpace).c_str());
						coords[0] = parseDouble(line_.substr(24, standardCharSpace));
						coords[1] = parseDouble(line_.substr(32, standardCharSpace));
						coords[2] = parseDouble(line_.substr(40, standardCharSpace));
						mesh.gridPoints.push_back(coords);
						mesh.gridIds.push_back(gridId);
						countPoints++;
					}
					else if (startsWithString(line_, "GRID*")) {
						gridId = atoi(line_.substr(8, extCharSpace).c_str());
						coords[0] = atof(line_.substr(40, extCharSpace).c_str());
						coords[1] = atof(line_.substr(56, extCharSpace).c_str());
						getline(filestream_, line_);
						coords[2] = atof(line_.substr(8, extCharSpace).c_str());
						mesh.gridPoints.push_back(coords);
						mesh.gridIds.push_back(gridId);
						countPoints++;
					}
					else if (startsWithString(line_, "GRID")) {
						std::string field;
						std::istringstream iss(line_);
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
					}
					else {
						break;
					}
				} while (getline(filestream_, line_));
			}
			catch (...) {
				return D3D_status::CANNOT_READ_MESH;
			}

			return D3D_status::SUCCESS;
		}

		D3D_status readGridCells(d3d::CommonMeshData& mesh,
			std::map<int, int>& pointMap,
			std::array<std::vector<CommonMeshData::Cell>, 4>& tempConnectivity) {
			std::array<int, 4> cellNumbers;
			for (auto ii = 0; ii < cellNumbers.size(); ++ii) {
				cellNumbers[ii] = mesh.cellIds[ii].size();
			}
			try {
				do {
					if (!(startsWithCell(line_) || startsWithString(line_, "+"))) break;
					int charId = standardCharSpace;
					bool is_extended;
					CellElement thisElemType;
					for (auto elemType : allCellElements) {
						if (startsWithString(line_, elemType.name)) {
							thisElemType = elemType;
							is_extended = line_[elemType.size] == '*';
							break;
						}
					}
					if (thisElemType.defined) {
						int charSpace;
						charSpace = is_extended ? extCharSpace : standardCharSpace;

						auto cellId = atoi(line_.substr(charId, charSpace).c_str());
						charId += charSpace;
						auto cellTag = atoi(line_.substr(charId, charSpace).c_str());
						charId += charSpace;

						auto tempCellConnectivity =
							CommonMeshData::Cell(thisElemType.numPoints);

						for (int ii = 0; ii < thisElemType.numPoints; ++ii) {
							if (charId >= CONST_COL_WIDTH) {
								getline(filestream_,
									line_);  // skip to second line of coordinates
								charId = standardCharSpace;
							}

							auto gridId = atoi(line_.substr(charId, charSpace).c_str());
							tempCellConnectivity[ii] = gridId;
							charId += charSpace;
						}

						tempConnectivity[thisElemType.dim].push_back(
							tempCellConnectivity);

						mesh.cellIds[thisElemType.dim].push_back(cellId);
						mesh.cellPIDs[thisElemType.dim].push_back(cellTag);
						mesh.cellTypes[thisElemType.dim].push_back(thisElemType.type);

						++cellNumbers[thisElemType.dim];
					}
				} while (getline(filestream_, line_));
			}
			catch (...) {
				return D3D_status::CANNOT_READ_MESH;
			}

			return D3D_status::SUCCESS;
		}


		D3D_status readRigidBodyElements(
			std::ifstream& femFile, std::string line, std::vector<RigidBodyElement> &rbes) {
			// Parse the rbe section
			bool continueReading = true;
			int rbe_count = 0;
			try {
				RigidBodyElement rbe;
				while (continueReading) {
					continueReading = false;
					int charLoc = standardCharSpace; //If the character space can go to 16 for RBEs, this will fail. Don't know if that exists or not in usage.
					if (startsWithString(line, "RBE2")) {
						rbe = RigidBodyElement();
						rbe.type = RigidBodyElementType(2);
						//rbe's identity number
						rbe.id = atoi(line.substr(charLoc, standardCharSpace).c_str());
						charLoc += standardCharSpace;
						// location grid point id number
						rbe.virtualPointId = atoi(line.substr(charLoc, standardCharSpace).c_str());
						charLoc += standardCharSpace;
						// component numbers of dependent degress of freedom
						rbe.degreesOfFreedom = line.substr(charLoc, standardCharSpace).c_str();
						charLoc += standardCharSpace;
						while (charLoc < CONST_COL_WIDTH) {
							// collect gridpoints
							rbe.gridIds.insert(atoi(line.substr(charLoc, standardCharSpace).c_str()));
							charLoc += standardCharSpace;
						}
						continueReading = true;
					}
					if (startsWithString(line, "+")) {
						while (charLoc < CONST_COL_WIDTH) {
							// collect gridpoints
							rbe.gridIds.insert(atoi(line.substr(charLoc, standardCharSpace).c_str()));
							charLoc += standardCharSpace;
						}
						continueReading = true;
					}
					continueReading &= getline(femFile, line).good();
				}

			}
			catch (...) {
				return D3D_status::CANNOT_OPEN_FILE;
			}

			return D3D_status::SUCCESS;
		}


		D3D_status read(const boost::filesystem::path& filePath, BulkDataFileContents& bdf_contents) {
			//Read the Nastran BDF
			std::ifstream filestream_(filePath.string());

			std::map<int, int> pointMap;
			std::array<std::vector<CommonMeshData::Cell>, 4> tempConnectivity;

			auto ret_code = D3D_status::SUCCESS;

			if (filestream_.is_open()) {
				std::string _;
				std::string card_txt = "";
				int section_cnt = 0;
				bool is_different_section = false;
				bool is_unread = getline(filestream_, line_).good();
				while (is_unread) {
					if (startsWithString(line_, "DTPL")) {
						std::istringstream iss(line_);
						iss >> _ >> _ >> _ >> bdf_contents.designDomainPID;
						if (iss.fail()) {
							iss.clear();
							std::cout << "Design domain key could not be read."
								<< std::endl;
							bdf_contents.designDomainPID = -1;
						}
					}

					//Read all gridpoints
					if (startsWithString(line_, "GRID")) {
						//This block should only hit once.
						bdf_contents.bonusSections.push_back(card_txt);
						card_txt = "";
						section_cnt++;
						bdf_contents.pointSectionPosition = section_cnt;
						//
						ret_code = readGridPoints(bdf_contents.mesh);
						if (ret_code != D3D_status::SUCCESS) {
							return ret_code;
						}
						for (auto ii = 0; ii < bdf_contents.mesh.gridIds.size(); ++ii) {
							pointMap[bdf_contents.mesh.gridIds[ii]] = ii;
						}
						continue;
					}

					//Read all RBEs.
					if (startsWithString(line_, "RBE")) {
						//This block should only hit once.
						bdf_contents.bonusSections.push_back(card_txt);
						card_txt = "";
						section_cnt++;
						bdf_contents.rigidBodySectionPosition = section_cnt;
						//
						ret_code = readRigidBodyElements(filestream_, line_, bdf_contents.rigidBodyElements);
						if (ret_code != D3D_status::SUCCESS) {
							return ret_code;
						}
						for (auto ii = 0; ii < bdf_contents.mesh.gridIds.size(); ++ii) {
							pointMap[bdf_contents.mesh.gridIds[ii]] = ii;
						}
						continue;
					}

					//Read all cells
					if (startsWithCell(line_))
					 {	
						//This block should only hit once.
						bdf_contents.bonusSections.push_back(card_txt);
						card_txt = "";
						section_cnt++;
						bdf_contents.cellSectionPosition = section_cnt;
						//

						// we have no guarantee that the grid Ids have been read before.
						// Thats why we create a temp connectivity with the grid Ids,
						// and we create the real connectivity after having read
						// everything with the point map and the temp connectivity
						ret_code = readGridCells(bdf_contents.mesh, pointMap,
							tempConnectivity);
						if (ret_code != D3D_status::SUCCESS) {
							return ret_code;
						}
						continue;
					}

					//Save any unrecognized sections
					card_txt.append(line_ + "\n");
					is_unread = getline(filestream_, line_).good();
				}
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

}; //class BulkDataFileParser

D3D_status readBDF(const boost::filesystem::path& filePath, BulkDataFileContents& bdf_contents) {
	BulkDataFileParser bdfer;
	auto status =  bdfer.read(filePath, bdf_contents);
	return status;
}

//
//D3D_status writeBDFFromCommonMeshData(CommonMeshData& mesh,
//	const boost::filesystem::path& path) {
//	if (mesh.gridPoints.size() == 0) {
//		std::cout << "No vertices to write\n";
//		return D3D_status::FAIL;
//	}
//	auto out = std::ofstream(path.string());
//	if (!out.is_open()) return D3D_status::FAIL;
//	int nMaterialId = 1;

//	out << "TITLE = written by Divergent 3D system\n";
//	out << "BEGIN BULK\n";

//	out << "MAT1    1       3.0e+07         0.3300006.5e-06 5.4e+02\n";

//	// if this is not empty, we need to make the connectivity refer to these ids
//	if (mesh.gridIds.size() == 0) {
//		std::cout << "Creating new indexing for the mesh grid points\n";
//		for (auto ii = 0; ii < mesh.gridPoints.size(); ++ii) {
//			mesh.gridIds.push_back(ii + 1);
//		}
//	}

//	auto pids = findUniquePIDs(mesh.cellPIDs);
//	std::for_each(pids.begin(), pids.end(), [&](int pid) {
//		out << "PSHELL  " << std::left << std::setw(8) << pid << std::left
//			<< std::setw(8) << nMaterialId << std::left << std::setw(8)
//			<< 0.01 * pid << "\n";
//	});

//	// !! The bdf file format starts indexing at 1, so we need to add 1 to each
//	// grid index
//	for (auto dim = 0; dim < mesh.connectivity.size(); ++dim) {
//		if (mesh.cellIds[dim].size() == 0) {
//			for (auto ii = 0; ii < mesh.connectivity[dim].size(); ++ii) {
//				mesh.cellIds[dim].push_back(ii + 1);
//			}
//		}

//		if (mesh.connectivity[dim].size() > 0) {
//			for (auto ii = 0; ii < mesh.connectivity[dim].size(); ++ii) {
//				auto cellPID = mesh.cellPIDs[dim].size() > 0
//					? mesh.cellPIDs[dim][ii]
//					: defaultPID;

//				auto cell = mesh.connectivity[dim][ii];
//				auto elemName = getCellName(dim, cell.size());

//				out << elemName << std::left << std::setw(8)
//					<< mesh.cellIds[dim][ii] << std::left << std::setw(8)
//					<< cellPID;
//				for (auto jj = 0; jj < cell.size(); ++jj) {
//					if (jj + 4 % 10 == 0) out << "\n        ";
//					out << std::left << std::setw(8) << mesh.gridIds[cell[jj]];
//				}
//				out << "\n";
//			}
//		}
//	}
//	out << "PMASS   4       0.100000\n";

//	for (auto ii = 0; ii < mesh.gridPoints.size(); ++ii) {
//		auto coords = mesh.gridPoints[ii];

//		out.precision(9);
//		out << std::scientific;
//		out << "GRID*   " << std::left << std::setw(32) << mesh.gridIds[ii]
//			<< std::setw(16) << coords[0] << std::setw(16) << coords[1]
//			<< "*\n*       " << std::setw(16) << coords[2] << "\n";
//	}
//	out << "ENDDATA\n";
//	out.close();
//	return D3D_status::SUCCESS;
//}

//D3D_status readBDFToCommonMeshData(const boost::filesystem::path& meshPath,
//	CommonMeshData& mesh) {
//	int _;
//	return readBDFToCommonMeshData(meshPath, mesh, _);
//}

//D3D_status readBDFToCommonMeshData(const boost::filesystem::path& filePath,
//	CommonMeshData& mesh, int& designDomainPID) {
//	std::ifstream femFile(filePath.string());

//	std::cout << "Read input file." << std::endl;

//	std::map<int, int> pointMap;
//	std::array<std::vector<CommonMeshData::Cell>, 4> tempConnectivity;

//	auto ret_code = D3D_status::SUCCESS;

//	if (femFile.is_open()) {
//		std::string line;
//		std::string _;

//		while (getline(femFile, line)) {
//			if (startsWithString(line, "DTPL")) {
//				std::istringstream iss(line);
//				iss >> _ >> _ >> _ >> designDomainPID;
//				if (iss.fail()) {
//					iss.clear();
//					std::cout << "Design domain key could not be read."
//						<< std::endl;
//					designDomainPID = -1;
//				}
//			}
//			if (startsWithString(line, "GRID")) {
//				ret_code = readGridPoints(femFile, line, mesh);
//				if (ret_code != D3D_status::SUCCESS) {
//					return ret_code;
//				}
//				for (auto ii = 0; ii < mesh.gridIds.size(); ++ii) {
//					pointMap[mesh.gridIds[ii]] = ii;
//				}
//			}

//			if (std::any_of(allCellElements.begin(), allCellElements.end(),
//				[line](CellElement e) {
//				return startsWithString(line, e.name);
//			})) {
//				// we have no guarantee that the grid Ids have been read before.
//				// Thats why we create a temp connectivity with the grid Ids,
//				// and we create the real connectivity after having read
//				// everything with the point map and the temp connectivity
//				ret_code = readGridCells(femFile, line, mesh, pointMap,
//					tempConnectivity);
//				if (ret_code != D3D_status::SUCCESS) {
//					return ret_code;
//				}
//			}
//		}

//		auto numCells = 0;
//		std::for_each(
//			tempConnectivity.begin(), tempConnectivity.end(),
//			[&](std::vector<std::vector<int>> vec) { numCells += vec.size(); });

//		std::cout << "Finished reading " << mesh.gridPoints.size()
//			<< " points and " << numCells << " cells." << std::endl;
//	}

//	// the tempConnectivity data structure refers to the vertices by their fem
//	// grid ids. We use the point map to instead make them refer to their ids in
//	// the new vectors

//	for (auto ndim = 0; ndim < tempConnectivity.size(); ++ndim) {
//		mesh.connectivity[ndim] =
//			std::vector<std::vector<int>>(tempConnectivity[ndim].size());
//		for (auto ii = 0; ii < tempConnectivity[ndim].size(); ++ii) {
//			auto newCell = std::vector<int>(tempConnectivity[ndim][ii].size());
//			for (auto jj = 0; jj < tempConnectivity[ndim][ii].size(); ++jj) {
//				newCell[jj] = pointMap[tempConnectivity[ndim][ii][jj]];
//			}
//			mesh.connectivity[ndim][ii] = newCell;
//		}
//	}

//	return D3D_status::SUCCESS;
//}

}  // namespace io

}  // namespace d3d
