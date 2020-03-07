#ifndef MESHDATALIB_VERTEX_CLUSTER_H
#define MESHDATALIB_VERTEX_CLUSTER_H

#include <unordered_set>

#include "meshData.h"

namespace d3d {

// "Vertex clusters" are collections of 0-cells which represent a region given by the 2-cells they share. Cluster transferrence is meant to find equivalent areas 

// For each set of pointids, returns a list of points in the target mesh within dist_threshold of the source submesh induced by the point_ids.
std::unordered_set<int> transferVertexCluster(
    const CommonMeshData source,
	std::unordered_set<int> cluster_list,
    const double dist_threshold, const CommonMeshData target);


// Returns a list of points in the target mesh within dist_threshold of the source submesh induced by the point_ids.
std::vector<std::unordered_set<int>> transferVertexClusters(
    const CommonMeshData source,
    const std::vector<std::unordered_set<int>> cluster_list,
    const double dist_threshold, const CommonMeshData target);


} //end namespace d3d

#endif  // MESHDATALIB_VERTEX_CLUSTER_H