#include "bdfIO.h"

#include <fstream>
#include <iostream>
#include <sstream>

#include <vtkCellArray.h>
#include <vtkCellType.h>
#include <vtkIdTypeArray.h>
#include <vtkOBJReader.h>
#include <vtkOBJWriter.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSTLReader.h>
#include <vtkSTLWriter.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLUnstructuredGridReader.h>
#include <vtkXMLUnstructuredGridWriter.h>

#include "utils.h"

namespace d3d {

namespace io {

D3D_status readOBJToCommonMeshData(const boost::filesystem::path& meshPath,
                                     CommonMeshData& mesh) {
    auto return_code = D3D_status::SUCCESS;
    auto reader = vtkSmartPointer<vtkOBJReader>::New();
    reader->SetFileName(meshPath.string().c_str());
    reader->Update();
    auto polydata = reader->GetOutput();

    mesh = makeCommonMeshData(polydata);

    return return_code;
}

D3D_status writeOBJFromCommonMeshData(CommonMeshData& mesh,
                                      const boost::filesystem::path& meshPath) {
    auto return_code = D3D_status::SUCCESS;
    auto writer = vtkSmartPointer<vtkOBJWriter>::New();
    writer->SetFileName(meshPath.string().c_str());
    writer->SetInputData(makePolydata(mesh));
    writer->Update();
    writer->Write();
    return return_code;
}

D3D_status readSTLToCommonMeshData(const boost::filesystem::path& meshPath,
                                     CommonMeshData& mesh) {
    auto return_code = D3D_status::SUCCESS;
    auto reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(meshPath.string().c_str());
    reader->Update();
    auto polydata = reader->GetOutput();

    mesh = makeCommonMeshData(polydata);

    return return_code;
}

D3D_status writeSTLFromCommonMeshData(CommonMeshData& mesh,
                                      const boost::filesystem::path& meshPath) {
    auto return_code = D3D_status::SUCCESS;
    auto writer = vtkSmartPointer<vtkSTLWriter>::New();
    writer->SetFileName(meshPath.string().c_str());
    writer->SetInputData(makePolydata(mesh));
    writer->Update();
    writer->Write();
    return return_code;
}

D3D_status readVTPToCommonMeshData(const boost::filesystem::path& meshPath,
                                     CommonMeshData& mesh) {
    auto return_code = D3D_status::SUCCESS;
    auto reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
    reader->SetFileName(meshPath.string().c_str());
    reader->Update();
    auto polydata = reader->GetOutput();

    mesh = makeCommonMeshData(polydata);

    return return_code;
}

D3D_status writeVTPFromCommonMeshData(CommonMeshData& mesh,
                                      const boost::filesystem::path& meshPath) {
    auto return_code = D3D_status::SUCCESS;
    auto writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
    writer->SetFileName(meshPath.string().c_str());
    writer->SetInputData(makePolydata(mesh));
    writer->Update();
    writer->Write();
    return return_code;
}

D3D_status readVTUToCommonMeshData(const boost::filesystem::path& meshPath,
                                     CommonMeshData& mesh) {
    auto return_code = D3D_status::SUCCESS;
    auto reader = vtkSmartPointer<vtkXMLUnstructuredGridReader>::New();
    reader->SetFileName(meshPath.string().c_str());
    reader->Update();
    auto uGrid = reader->GetOutput();

    mesh = makeCommonMeshData(uGrid);

    return return_code;
}

D3D_status writeVTUFromCommonMeshData(CommonMeshData& mesh,
                                      const boost::filesystem::path& meshPath) {
    auto return_code = D3D_status::SUCCESS;
    auto writer = vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();
    writer->SetFileName(meshPath.string().c_str());
    writer->SetInputData(makeUGrid(mesh));
    writer->Update();
    writer->Write();
    return return_code;
}

}  // namespace io

}  // namespace d3d
