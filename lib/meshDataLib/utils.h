#ifndef MESHDATALIB_UTILS_H
#define MESHDATALIB_UTILS_H

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>

#include "meshData.h"

namespace d3d {
vtkSmartPointer<vtkPolyData> makePolydata(const CommonMeshData& mesh);
CommonMeshData makeCommonMeshData(vtkSmartPointer<vtkPolyData> polydata);

vtkSmartPointer<vtkUnstructuredGrid> makeUGrid(const d3d::CommonMeshData& mesh);
CommonMeshData makeCommonMeshData(vtkSmartPointer<vtkUnstructuredGrid> uGrid);
}  // namespace d3d

#endif