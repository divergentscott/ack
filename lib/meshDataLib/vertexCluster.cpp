#include "vertexCluster.h"
#include "vtkCellLocator.h"
#include "vtkExtractSelection.h"
#include "vtkKdTree.h"
#include "vtkSelectionNode.h"

#include "utils.h"

namespace d3d {

class VertexClusterer {
   private:
    vtkSmartPointer<vtkPolyData> source_ = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> target_ = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkKdTree> target_tree_ = vtkSmartPointer<vtkKdTree>::New();

   public:
    VertexClusterer() = delete;
    VertexClusterer(vtkSmartPointer<vtkPolyData> source_mesh,
                    vtkSmartPointer<vtkPolyData> target_mesh) {
        source_ = source_mesh;
        target_ = target_mesh;
        source_->BuildLinks();
        target_tree_->BuildLocatorFromPoints(target_->GetPoints());
	}

	std::unordered_set<int> find_cluster(
        const std::unordered_set<int> point_ids,
        const double dist_threshold) {
        // First find the cells whose vertices are all members of point_ids
        std::unordered_set<int> cids;
        for (int pid : point_ids) {
            auto cells_at_pid = vtkSmartPointer<vtkIdList>::New();
            source_->GetPointCells(pid, cells_at_pid);
            for (int foo = 0; foo < cells_at_pid->GetNumberOfIds(); foo++) {
                int cell_foo = cells_at_pid->GetId(foo);
                auto pts_foo = vtkSmartPointer<vtkIdList>::New();
                source_->GetCellPoints(cell_foo, pts_foo);
                bool keep_foo = true;
                int bar = 0;
                const int npts = (pts_foo->GetNumberOfIds());
                while (keep_foo & (bar < npts)) {
                    int pt_bar = pts_foo->GetId(bar);
                    keep_foo &= (point_ids.find(pt_bar) != point_ids.end());
                    bar++;
                }
                if (keep_foo) {
                    cids.insert(cell_foo);
                }
            }
        }
        // extract the cluster
        auto cluster = vtkSmartPointer<vtkPolyData>::New();
        auto cluster_cellarray = vtkSmartPointer<vtkCellArray>::New();
        cluster->SetPolys(cluster_cellarray);
        auto clustercellids = vtkSmartPointer<vtkIdList>::New();
        for (int cid : cids) clustercellids->InsertNextId(cid);
        cluster->CopyCells(source_, clustercellids);
        // poly->CopyCells(cluster, clustercellsids);
        // Get the points within a box around the cluster
        double clusterbounds[6];
        cluster->GetBounds(clusterbounds);
        clusterbounds[0] -= dist_threshold;
        clusterbounds[2] -= dist_threshold;
        clusterbounds[4] -= dist_threshold;
        clusterbounds[1] += dist_threshold;
        clusterbounds[3] += dist_threshold;
        clusterbounds[5] += dist_threshold;
        auto target_pts_in_area = vtkSmartPointer<vtkIdTypeArray>::New();
        target_tree_->FindPointsInArea(clusterbounds, target_pts_in_area);
        // For each point in the bounding box, check distance from source
        // cluster
        std::unordered_set<int> target_pts;
        auto locker = vtkSmartPointer<vtkCellLocator>::New();
        locker->SetDataSet(cluster);
        locker->BuildLocator();
        for (int foo = 0; foo < target_pts_in_area->GetNumberOfValues();
                foo++) {
            int pid = target_pts_in_area->GetValue(foo);
            double pt[3];
            target_->GetPoint(pid, pt);
            double _closestPoint[3];
            vtkIdType _cellId;
            int _subId;
            double _dist2;
            bool isclose = locker->FindClosestPointWithinRadius(
                pt, dist_threshold, _closestPoint, _cellId, _subId, _dist2);
            if (isclose) target_pts.insert(pid);
        }
        //
        return target_pts;
	}
};


std::vector<std::unordered_set<int>> transferVertexClusters(
    // For each set of pointids, returns a list of points in the target mesh within dist_threshold of the
    // source submesh induced by the point_ids.
    const CommonMeshData source,
    const std::vector<std::unordered_set<int>> cluster_list,
    const double dist_threshold, const CommonMeshData target) {
    auto source_poly = makePolydata(source);
    auto target_poly = makePolydata(target);
    auto vc = VertexClusterer(source_poly, target_poly);
    std::vector<std::unordered_set<int>> t_clusters;
    for (auto cluster : cluster_list) {
        auto t_cluster = vc.find_cluster(cluster, dist_threshold);
        t_clusters.push_back(t_cluster);
    }
    return t_clusters;
}

std::unordered_set<int> transferVertexCluster(
    const CommonMeshData source, const std::unordered_set<int> point_ids,
    const double dist_threshold, const CommonMeshData target) {
    // Returns a list of points in the target mesh within dist_threshold of the
    // source submesh induced by the point_ids.
    auto source_poly = makePolydata(source);
    auto target_poly = makePolydata(target);
    auto vc = VertexClusterer(source_poly, target_poly);
    return vc.find_cluster(point_ids, dist_threshold);
}

}  // namespace d3d
