#include <boost/filesystem.hpp>
#include "vector"
#include "iostream"
#include "fstream"
#include <tetgen.h>
#include "set"
#include "map"
#include "algorithm"
#include "array"
#include "queue"
#include "tetgen.h" // Defined tetgenio, tetrahedralize().

struct tetgenio_data_list {
	//Each point is 3 doubles
	std::vector<double> points;
	//Every 2 point ids is one edge
	std::vector<int> edges;
	//Every 3 point ids is one triangle
	std::vector<int> triangles;
	//Every 4 point ids is a tetrahedron
	std::vector<int> tetrahedra;
	//tetrahedronregion[cell_id]=region id
	std::vector<int> tetrahedronregion;
};

tetgenio_data_list tetgenio_to_datalist(tetgenio *mesh_in) {
	tetgenio_data_list mesh_out;
	for (int i = 0; i < 3*mesh_in->numberofpoints; i++) {
		mesh_out.points.push_back(mesh_in->pointlist[i]);
	}
	for (int i = 0; i < 2*mesh_in->numberofedges; i++) {
		mesh_out.edges.push_back(mesh_in->edgelist[i]);
	}
	for (int i = 0; i < 3 * mesh_in->numberoftrifaces; i++) {
		mesh_out.triangles.push_back(mesh_in->trifacelist[i]);
	}
	for (int i = 0; i < 4 * mesh_in->numberoftetrahedra; i++) {
		mesh_out.tetrahedra.push_back(mesh_in->tetrahedronlist[i]);
	}
	if (mesh_in->numberoftetrahedronattributes ==1 ) {
		for (int i = 0; i <  mesh_in->numberoftetrahedra; i++) {
			mesh_out.tetrahedronregion.push_back(mesh_in->tetrahedronattributelist[i]);
		}
	}
	return mesh_out;
}

//void datalist_to_tetgenio(tetgenio_data_list mesh_in,
//	tetgenio &mesh_out) {
//	// copy points
//	if (mesh_in.points.size() > 0) {
//		mesh_out.numberofpoints = mesh_in.points.size() / 3;
//		mesh_out.pointlist = new REAL[mesh_in.points.size()];
//		for (int i = 0; i < mesh_in.points.size(); i++) {
//			mesh_out.pointlist[i] = mesh_in.points[i];
//		}
//	}
//	// copy edges
//	if (mesh_in.edges.size() > 0) {
//		mesh_out.numberofedges = mesh_in.edges.size() / 2;
//		mesh_out.edgelist = new int[mesh_in.points.size()];
//		for (int i = 0; i < mesh_in.points.size(); i++) {
//			mesh_out.edgelist[i] = mesh_in.edges[i];
//		}
//	}
//	// copy triangles
//	if (mesh_in.triangles.size() > 0) {
//		mesh_out.numberoftrifaces = mesh_in.triangles.size() / 3;
//		mesh_out.trifacelist = new int[mesh_in.triangles.size()];
//		for (int i = 0; i < mesh_in.triangles.size(); i++) {
//			mesh_out.trifacelist[i] = mesh_in.triangles[i];
//		}
//	}
//	// copy tets
//	if (mesh_in.tetrahedra.size() > 0) {
//		mesh_out.numberoftetrahedra = mesh_in.tetrahedra.size() / 4;
//		mesh_out.tetrahedronlist = new int[mesh_in.tetrahedra.size()];
//		for (int i = 0; i < mesh_in.tetrahedra.size(); i++) {
//			mesh_out.tetrahedronlist[i] = mesh_in.tetrahedra[i];
//		}
//	}
//	// copy tet regions
//	if (mesh_in.tetrahedronregion.size() > 0) {
//		mesh_out.numberoftetrahedronattributes = 1;
//		mesh_out.tetrahedronattributelist =
//			new REAL[mesh_in.tetrahedronregion.size()];
//		for (int i = 0; i < mesh_in.tetrahedronregion.size(); i++) {
//			mesh_out.tetrahedronattributelist[i] =
//				mesh_in.tetrahedronregion[i];
//		}
//	}
//}

void datalist_to_tetgenio(tetgenio_data_list mesh_in,
	tetgenio &mesh_out) {
	// copy points
	if (mesh_in.points.size() > 0) {
		mesh_out.numberofpoints = mesh_in.points.size()/3;
		mesh_out.pointlist = new REAL[mesh_in.points.size()];
		for (int i = 0; i < mesh_in.points.size(); i++) {
			mesh_out.pointlist[i] = mesh_in.points[i];
		}
	}
	// copy edges
	if (mesh_in.edges.size() > 0) {
		mesh_out.numberofedges = mesh_in.edges.size() / 2;
		mesh_out.edgelist = new int[mesh_in.points.size()];
		for (int i = 0; i < mesh_in.points.size(); i++) {
			mesh_out.edgelist[i] = mesh_in.edges[i];
		}
	}
	// copy triangles
	if (mesh_in.triangles.size() > 0) {
		// facets
		int num_tris = mesh_in.triangles.size() / 3;
		mesh_out.numberoffacets = num_tris;
		mesh_out.facetlist = new tetgenio::facet[num_tris];
		mesh_out.facetmarkerlist = new int[num_tris];
		for (int i = 0; i < num_tris; i++) {
			mesh_out.facetmarkerlist[i] = 0;
			auto f = &mesh_out.facetlist[i];  // facet i
			f->numberofpolygons = 1;
			f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
			f->numberofholes = 0;
			f->holelist = NULL;
			auto p = &f->polygonlist[0];
			p->numberofvertices = 3;
			p->vertexlist = new int[p->numberofvertices];
			for (int j = 0; j < p->numberofvertices; j++) {
				p->vertexlist[j] = mesh_in.triangles[3 * i + j];
			}
		}
	}
	// copy tets
	if (mesh_in.tetrahedra.size() > 0) {
		mesh_out.tetrahedronlist = new int[mesh_in.tetrahedra.size()];
		for (int i = 0; i < mesh_in.tetrahedra.size(); i++) {
			mesh_out.tetrahedronlist[i] = mesh_in.tetrahedra[i];
		}
	}
	// copy tet regions
	if (mesh_in.tetrahedronregion.size() > 0) {
		mesh_out.tetrahedronattributelist =
			new REAL[mesh_in.tetrahedronregion.size()];
		for (int i = 0; i < mesh_in.tetrahedronregion.size(); i++) {
			mesh_out.tetrahedronattributelist[i] =
				mesh_in.tetrahedronregion[i];
		}
	}
}

void write_tetgenio_to_vtk(const tetgenio &mesh_in,
	const char *ofilename) {
	// Writes tetrahedra to file.
	// If there are no tetrahedra, writes all polygons to file, but assumes they are triangles.
	std::ofstream out_file;
	std::cout << "\nWriting to:\n" << ofilename << "\n";
	out_file.open(ofilename);
	if (out_file.is_open()) {
		if (mesh_in.mesh_dim != 3) {
			std::cout
				<< "VTK write only implemented for meshes embedded in 3-space.";
		}
		//

		// int NEL = tetrahedrons->items - hullsize;
		int num_points = mesh_in.numberofpoints;

		// always write big endian
		// bool ImALittleEndian = !testIsBigEndian();
		out_file << "# vtk DataFile Version 2.0\n";
		out_file << "Unstructured Grid\n";
		out_file << "ASCII\n";  // BINARY
		out_file << "DATASET UNSTRUCTURED_GRID\n";
		out_file << "POINTS " << num_points << " double\n";

		// Write points
		for (int id = 0; id < num_points; id++) {
			double pnt[3];
			for (int foo = 0; foo < 3; foo++) {
				pnt[foo] = mesh_in.pointlist[3 * id + foo];
			}
			out_file << pnt[0] << " " << pnt[1] << " " << pnt[2] << "\n";
		}
		out_file << "\n";

		// Write tets
		int num_tetrahedra = mesh_in.numberoftetrahedra;
		if (num_tetrahedra > 0) {
			out_file << "CELLS " << num_tetrahedra << " " << num_tetrahedra * 5
				<< "\n";
			for (int tid = 0; tid < num_tetrahedra; tid++) {
				int tet[4];
				for (int foo = 0; foo < 4; foo++) {
					tet[foo] = mesh_in.tetrahedronlist[4 * tid + foo] -
						mesh_in.firstnumber;
				}
				out_file << 4 << " " << tet[0] << " " << tet[1] << " " << tet[2]
					<< " " << tet[3] << "\n";
			}
			out_file << "\n";

			// Write tet cell type
			int celltype = 10;
			out_file << "CELL_TYPES " << num_tetrahedra << "\n";
			for (int tid = 0; tid < num_tetrahedra; tid++) {
				out_file << celltype << "\n";
			}
			out_file << "\n";

			// Write attributes
			if (mesh_in.numberoftetrahedronattributes > 0) {
				if (mesh_in.numberoftetrahedronattributes != 1)
					printf("Too many attributes.");
				// Output tetrahedra region attributes.
				out_file << "CELL_DATA " << num_tetrahedra << "\n";
				out_file << "SCALARS region int 1\n";
				out_file << "LOOKUP_TABLE default\n";
				for (int tid = 0; tid < num_tetrahedra; tid++) {
					out_file << mesh_in.tetrahedronattributelist[tid] << "\n";
				}
				out_file << "\n";
			}
		}
		else {
			//If there are no tetrahedra, then the simplicial complex may be 2 dimensional so write the triangles instead.
			//This may fail if there are polygons with >3 vertices.
			int num_tris = mesh_in.numberoffacets;
			out_file << "CELLS " << num_tris << " " << num_tris * 4
				<< "\n";
			for (int i = 0; i < num_tris; i++) {
				auto f = &mesh_in.facetlist[i];  // facet i
				int num_polys = f->numberofpolygons;
				for (int j = 0; j < num_polys; j++) {
					auto p = &f->polygonlist[j];
					out_file << p->numberofvertices << " ";
					for (int k = 0; k < p->numberofvertices; k++) {
						out_file << p->vertexlist[k] << " ";
					}
					out_file << "\n";
				}
			}
			out_file << "\n";
			// Write tet cell type
			int celltype = 5;
			out_file << "CELL_TYPES " << num_tris << "\n";
			for (int tid = 0; tid < num_tris; tid++) {
				out_file << celltype << "\n";
			}
			out_file << "\n";
		}
		out_file.close();
	} else {
		printf("File I/O Error:  Cannot create file %s.\n", ofilename);
	}
}

using graph_adjacency_list = std::set<std::array<int, 2>>;

std::vector<int> filter_tets_by_region(tetgenio *tetg, std::set<int> regions) {
	// Return a list of all tets in the regions listed
	std::vector<int> tets_found;
	for (int tet_i = 0; tet_i < tetg->numberoftetrahedra; tet_i++) {
		int reg_i =
			tetg->tetrahedronattributelist[tet_i *
			tetg->numberoftetrahedronattributes];
		if (regions.find(reg_i) != regions.end()) {
			tets_found.push_back(tet_i);
		}
	}
	return tets_found;
}

graph_adjacency_list get_region_adjacency_edges(tetgenio *mesh_in) {
	// Extract adjacency information between the regions labeling the tetrahedra
	graph_adjacency_list region_adjacency_edges;
	std::set<int> regions;
	for (int i = 0; i < mesh_in->numberoftetrahedra; i++) {
		int region_i = mesh_in->tetrahedronattributelist[i];
		for (int j = 0; j < 4; j++) {
			int ngb_id = mesh_in->neighborlist[i * 4 + j];
			int region_j;
			if (ngb_id >= 0) {
				region_j = mesh_in->tetrahedronattributelist[ngb_id];
			}
			else {
				region_j = 0;
			}
			if (region_i > region_j) {
				std::array<int, 2> region_ij{ region_j, region_i };
				region_adjacency_edges.insert(region_ij);
				regions.insert(region_i);
				regions.insert(region_j);
			}
		}
	}
	return region_adjacency_edges;
}

std::map<int, bool> compute_two_coloring(graph_adjacency_list edges,
	int root = 0) {
	// Computes a 2-coloring of a graph from the adjacency list.
	// The colors are True, False and the root is colored False by convention.
	// Input adjacency list must represent a tree.
	// This does not check connectivity or if the graph is acyclic.
	std::map<int, bool> coloring;
	coloring[root] = false;
	// Copy edges to a que
	std::queue<std::array<int, 2>> edge_que;
	bool is_root_in_edges = false;
	for (auto edge : edges) {
		edge_que.push(edge);
		// Ensure that the root is in the edgelist to prevent an infinite loop.
		if (!is_root_in_edges)
			is_root_in_edges = (edge[0] == root) | (edge[1] == root);
	}
	if (is_root_in_edges) {
		// For every edge, color the vertices opposite if either has a color.
		while (!edge_que.empty()) {
			auto edge = edge_que.front();
			edge_que.pop();
			// Scan coloring for a vertex of the edge. Color if one vertex
			// found. Reque if not found.
			if (coloring.find(edge[0]) != coloring.end())
				coloring[edge[1]] = !coloring[edge[0]];
			else if (coloring.find(edge[1]) != coloring.end())
				coloring[edge[0]] = !coloring[edge[1]];
			else
				edge_que.push(edge);
		}
	}
	return coloring;
}

void filter_tetrahedra(tetgenio *mesh_in, tetgenio *mesh_out,
	std::vector<int> tets_kept) {
	// Copy over all the kept tetrahedra together with their points.
	// Updates the tetrahedra and point ids.
	// Does not recompute the 1 (edges) or 2 (faces) skeleta of the simplicial
	// complex.
	std::set<int> pnts_kept;
	mesh_out->firstnumber = 0;
	// First traverse the tets of tetin and find the points of kept tets.
	for (int tet : tets_kept) {
		for (int j = 0; j < 4; j++) {
			pnts_kept.insert(mesh_in->tetrahedronlist[4 * tet + j]);
		}
	}
	// Relabel the points
	std::map<int, int> pnt_id_map;
	int pnt_counter = 0;
	for (int pnt : pnts_kept) {
		pnt_id_map[pnt] = pnt_counter;
		pnt_counter++;
	}
	// Copy the kept  points into a new tetgenio with updated ids.
	// tetgenio tetout;
	mesh_out->numberofpoints = pnts_kept.size();
	mesh_out->numberofcorners = 4;
	mesh_out->pointlist = new REAL[mesh_out->numberofpoints * 3];
	for (int pnt : pnts_kept) {
		int pnt_new_id = pnt_id_map[pnt];
		for (int j = 0; j < 3; j++) {
			mesh_out->pointlist[3 * pnt_new_id + j] =
				mesh_in->pointlist[3 * pnt + j];
		}
	}
	// Copy the kept tets into a new tetgenio with updated ids.
	mesh_out->numberoftetrahedra = tets_kept.size();
	mesh_out->tetrahedronlist = new int[mesh_out->numberoftetrahedra * 4];
	int tet_counter = 0;
	for (int tet : tets_kept) {
		for (int j = 0; j < 4; j++) {
			mesh_out->tetrahedronlist[4 * tet_counter + j] =
				pnt_id_map[mesh_in->tetrahedronlist[4 * tet + j]];
		}
		tet_counter++;
	}
}

void carve_regions(tetgenio *mesh_in, tetgenio *mesh_out) {
	// Mesh_in is assumed to represent a 3 dimensional simplicial complex
	// with spatial regions assigned to tetrahedra separated by compact
	// 2-manifold S. The surface S is presumed to bound a 3-manifold M subset of
	// R^3. Copy the tetrahedra in regions bounded by S into mesh_out.
	//
	// Compute a coloring of the adjacency tree of the components of R^3-S to
	// determine "inside S" and "outside S".
	auto region_adjacency_edges = get_region_adjacency_edges(mesh_in);
	auto coloring = compute_two_coloring(region_adjacency_edges);
	// Find the regions that are colored False so that we can delete them.
	std::set<int> regions_kept;
	std::map<int, bool>::iterator it = coloring.begin();
	while (it != coloring.end()) {
		if (it->second) {
			regions_kept.insert(it->first);
		}
		it++;
	}
	std::vector<int> tets_kept = filter_tets_by_region(mesh_in, regions_kept);
	filter_tetrahedra(mesh_in, mesh_out, tets_kept);
}


void tetgen_example(tetgenio &in) {
	tetgenio::facet *f;
	tetgenio::polygon *p;

	// All indices start from 1.
	in.firstnumber = 1;

	in.numberofpoints = 8;
	in.pointlist = new REAL[in.numberofpoints * 3];
	in.pointlist[0] = 0;  // node 1.
	in.pointlist[1] = 0;
	in.pointlist[2] = 0;
	in.pointlist[3] = 2;  // node 2.
	in.pointlist[4] = 0;
	in.pointlist[5] = 0;
	in.pointlist[6] = 2;  // node 3.
	in.pointlist[7] = 2;
	in.pointlist[8] = 0;
	in.pointlist[9] = 0;  // node 4.
	in.pointlist[10] = 2;
	in.pointlist[11] = 0;
	// Set node 5, 6, 7, 8.
	for (int i = 4; i < 8; i++) {
		in.pointlist[i * 3] = in.pointlist[(i - 4) * 3];
		in.pointlist[i * 3 + 1] = in.pointlist[(i - 4) * 3 + 1];
		in.pointlist[i * 3 + 2] = 12;
	}

	in.numberoffacets = 6;
	in.facetlist = new tetgenio::facet[in.numberoffacets];
	in.facetmarkerlist = new int[in.numberoffacets];

	// Facet 1. The leftmost facet.
	f = &in.facetlist[0];
	f->numberofpolygons = 1;
	f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
	f->numberofholes = 0;
	f->holelist = NULL;
	p = &f->polygonlist[0];
	p->numberofvertices = 4;
	p->vertexlist = new int[p->numberofvertices];
	p->vertexlist[0] = 1;
	p->vertexlist[1] = 2;
	p->vertexlist[2] = 3;
	p->vertexlist[3] = 4;

	// Facet 2. The rightmost facet.
	f = &in.facetlist[1];
	f->numberofpolygons = 1;
	f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
	f->numberofholes = 0;
	f->holelist = NULL;
	p = &f->polygonlist[0];
	p->numberofvertices = 4;
	p->vertexlist = new int[p->numberofvertices];
	p->vertexlist[0] = 5;
	p->vertexlist[1] = 6;
	p->vertexlist[2] = 7;
	p->vertexlist[3] = 8;

	// Facet 3. The bottom facet.
	f = &in.facetlist[2];
	f->numberofpolygons = 1;
	f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
	f->numberofholes = 0;
	f->holelist = NULL;
	p = &f->polygonlist[0];
	p->numberofvertices = 4;
	p->vertexlist = new int[p->numberofvertices];
	p->vertexlist[0] = 1;
	p->vertexlist[1] = 5;
	p->vertexlist[2] = 6;
	p->vertexlist[3] = 2;

	// Facet 4. The back facet.
	f = &in.facetlist[3];
	f->numberofpolygons = 1;
	f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
	f->numberofholes = 0;
	f->holelist = NULL;
	p = &f->polygonlist[0];
	p->numberofvertices = 4;
	p->vertexlist = new int[p->numberofvertices];
	p->vertexlist[0] = 2;
	p->vertexlist[1] = 6;
	p->vertexlist[2] = 7;
	p->vertexlist[3] = 3;

	// Facet 5. The top facet.
	f = &in.facetlist[4];
	f->numberofpolygons = 1;
	f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
	f->numberofholes = 0;
	f->holelist = NULL;
	p = &f->polygonlist[0];
	p->numberofvertices = 4;
	p->vertexlist = new int[p->numberofvertices];
	p->vertexlist[0] = 3;
	p->vertexlist[1] = 7;
	p->vertexlist[2] = 8;
	p->vertexlist[3] = 4;

	// Facet 6. The front facet.
	f = &in.facetlist[5];
	f->numberofpolygons = 1;
	f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
	f->numberofholes = 0;
	f->holelist = NULL;
	p = &f->polygonlist[0];
	p->numberofvertices = 4;
	p->vertexlist = new int[p->numberofvertices];
	p->vertexlist[0] = 4;
	p->vertexlist[1] = 8;
	p->vertexlist[2] = 5;
	p->vertexlist[3] = 1;

	// Set 'in.facetmarkerlist'

	in.facetmarkerlist[0] = -1;
	in.facetmarkerlist[1] = -2;
	in.facetmarkerlist[2] = 0;
	in.facetmarkerlist[3] = 0;
	in.facetmarkerlist[4] = 0;
	in.facetmarkerlist[5] = 0;
}

void tetgen_tetrahedralize(char *switches, tetgenio *mesh_in,
	tetgenio *mesh_out) {
	try {
		tetrahedralize(switches, mesh_in, mesh_out);
	}
	catch (int e) {
		switch (e) {
		case 1:  // Out of memory.
			printf("  Tetra mesh failed:  Out of memory.\n");
		case 3:
			printf("  Tetra mesh failed:  self-intersection.\n");
		case 4:
			printf("  Tetra mesh failed:  small input feature size.\n");
		case 5:
			printf("  Tetra mesh failed: Two very close input facets. \n");
		}
		printf("  Tetra mesh failed:  unknown reason.\n");
	}
	printf("Tetgen tetrahedralize succeeded. ");
}

void tetrahedralize_with_hollows(tetgenio *mesh_in,
	double volume_constraint,
	double quality_constraint,
	tetgenio *mesh_out) {
	// Tetrahedralize the Piecewise Linear Complex. (option p)
	//   comformed (Y)
	//   mark all surface separated regions (A)
	//   compute neighbor list (n)
	//   index from 0 (z)
	//   quiet Q
	//   First pass will fill all cavities, perhaps with poor quality.
	tetgenio mesh_unhollow;
	mesh_unhollow.firstnumber = 0;
	std::cout << "\n\nTetrahedralizing.";
	std::cout<<"meshin: "<< mesh_in->numberofpoints <<"\n";
	write_tetgenio_to_vtk(*mesh_in, "mesh_in.vtk");
	tetgen_tetrahedralize("pYAznQ", mesh_in, &mesh_unhollow);
	write_tetgenio_to_vtk(*mesh_in, "mesh_in_after_tetgen.vtk");
	std::cout << "mesh_unhollow: " << mesh_in->numberofpoints << "\n";
	write_tetgenio_to_vtk(mesh_unhollow,"unhollow.vtk");
	//if (err_val != D3D_status::SUCCESS) return err_val;
	// Carve out the undesired regions.
	// Hollow bone design should also keep/remove tets according to engineering
	// considerations here.
	tetgenio mesh_hollow;
	mesh_hollow.firstnumber = 0;
	carve_regions(&mesh_unhollow, &mesh_hollow);
	write_tetgenio_to_vtk(mesh_hollow, "hollow.vtk");
	// Improve the mesh and apply quality constraints.
	//   Switches are chosen to reconsider an old tetmesh (r)
	//   do quality mesh generation (q) with a specified quality bound
	//   (1.414), and apply a maximum volume constraint (a0.1).
	//   switch O3 sets the optimization level.
	char switches[128];
	int optimization_level = 2;
	sprintf(switches, "zrO%dq%fa%fQfe", optimization_level, quality_constraint,
		volume_constraint);
	std::cout << "\nRemeshing with optimization switches " << switches << "\n";
	tetgen_tetrahedralize(switches, &mesh_hollow, mesh_out);
	std::cout << "\nhollow cells: " << mesh_hollow.numberofpoints << " " << mesh_hollow.numberofedges << " " << mesh_hollow.numberoftrifaces << " " << mesh_hollow.numberoftetrahedra<<"\n";
	std::cout << "\nout cells: " << mesh_out->numberofpoints << " " << mesh_out->numberofedges << " " << mesh_out->numberoftrifaces << " " << mesh_out->numberoftetrahedra << "\n";
	write_tetgenio_to_vtk(*mesh_out, "outout.vtk");
	//return err_val;
}  //tetrahedralize_with_hollows

tetgenio_data_list tetrahedralize_with_hollows(tetgenio_data_list mesh_in,
	double volume_constraint,
	double quality_constraint) {
	tetgenio tetgenio_in;
	tetgenio tetgenio_out;
	datalist_to_tetgenio(mesh_in, tetgenio_in);
	tetrahedralize_with_hollows(&tetgenio_in, volume_constraint, quality_constraint, &tetgenio_out);
	tetgenio_data_list mesh_out = tetgenio_to_datalist(&tetgenio_out);
	return mesh_out;
}

int euler_characteristic(tetgenio_data_list x) {
	//Compute the Euler characteristic of the simplicial complex x.
	return x.points.size() - x.edges.size() / 2 + x.triangles.size() / 3 - x.tetrahedra.size() / 4;
}

double det_three_by_three(std::vector<std::vector<double>> x) {
	return x[0][0] * (x[1][1] * x[2][2] - x[1][2] * x[2][1])
		+ x[0][1] * (x[1][2] * x[2][0] - x[1][0] * x[2][2])
		+ x[0][2] * (x[1][0] * x[2][1] - x[1][1] * x[2][0]);

}

double volume(tetgenio_data_list x) {
	//Compute the 3-volume of the tetrahedra of x.
	double vol = 0;
	int num_tets = x.tetrahedra.size() / 4;
	for (int i = 0; i < num_tets; i++) {
		std::vector<std::vector<double>> tet_edge_matrix;
		int pt_id_0 = x.tetrahedra[4 * i];
		for (int j = 1; j < 4; j++) {
			int pt_id_j = x.tetrahedra[4 * i + j];
			std::vector<double> tet_edge_matrix_j;
			for (int k = 0; k < 3; k++) {
				double y = x.points[3 * pt_id_j + k] - x.points[3 * pt_id_0 + k];
				tet_edge_matrix_j.push_back(y);
				//std::cout << y << " ";
			}
			//std::cout << "\n";
			tet_edge_matrix.push_back(tet_edge_matrix_j);
		}

		vol += std::abs(det_three_by_three(tet_edge_matrix)) / 6;
	}
	return vol;
}


/*D3D_status write_tetgenio_to_bdf(const tetgenio &tt, boost::filesystem::path filename,
                                 bool bReorderID = true)*/
int write_tetgenio_to_bdf(const tetgenio &tt, boost::filesystem::path filename) {
    // Save a Tetgen tetgenio to a d3d bdf file
	// Only writes points and tetrahedra
    if (tt.mesh_dim != 3) {
        printf("\n The BDF writer only supports meshes embedded in 3-space.");
        //return D3D_status::ERROR_TETGEN_UNKNOWN_FAILURE;
		return 1;
    }
    std::cout << "Writing to " << filename << "\n";
    boost::filesystem::ofstream fout{filename};

	if (fout.is_open()){
		int nPropertyId = 1;
		int firstnumber = 1; //tt.firstnumber;
		int np = tt.numberofpoints;
		int ntets = tt.numberoftetrahedra;
		// d3d headder
		fout<< "TITLE = written by Divergent 3D system\n";
		fout<<"BEGIN BULK\n";
		//Write points.
		for (int i = 0; i < np; i++) {
            char charbuffer[128];
            sprintf(charbuffer,
                        "GRID*   %-32d%-16e%-16e*\n*       %-16e\n",
				i + firstnumber, tt.pointlist[i * 3], tt.pointlist[i * 3 + 1],
				tt.pointlist[i * 3 + 2]);
            fout << charbuffer;
		}
		int firstindexdiff = firstnumber - tt.firstnumber;
		for (int i = 0; i < ntets; i++) {
            char charbuffer[128];
			sprintf(charbuffer, "CTETRA  %-8d%-8d", i + firstnumber, nPropertyId);
            fout << charbuffer;
			for (int k = 0, j = 4; k < 4; k++, j++) {
                char charbuffer[128];
				sprintf(charbuffer, "%-8d", tt.tetrahedronlist[i * 4 + k] + firstindexdiff);
                fout << charbuffer;
			}
			fout<<"\n";
		}
		fout << "ENDDATA\n";
		fout.close();
	} else {
		std::cout<<"File I/O Error:  Cannot create file " << filename << "\n";
		//return D3D_status::CANNOT_OPEN_FILE;
		return 1;
	}
    //return D3D_status::SUCCESS;
	return 0;
}

void tetra_save_bdf(const char *filename, const tetgenio &tt,
	bool bReorderID = true) {
	FILE *fout;
	char outfilename[FILENAMESIZE];
	REAL *ptlist = NULL;
	int *tetlist = NULL;
	int i, j, k, nPropertyId = 1;

	if (tt.mesh_dim != 3) {
		printf("\n not three dimensional mesh!  skip bdf saver");
		return;
	}

	sprintf(outfilename, "%s", filename);
	fout = fopen(outfilename, "w");

	int firstnumber = tt.firstnumber;
	int np = tt.numberofpoints;
	int ntets = tt.numberoftetrahedra;
	int numberofcorners = tt.numberofcorners;

	ptlist = tt.pointlist;
	tetlist = tt.tetrahedronlist;

	printf("Saving nodes and elems to %s\n", outfilename);

	// d3d headder
	fprintf(fout,
		"TITLE = written by Divergent 3D system (Jan 23 16:19:26 2018) \n");
	fprintf(fout, "BEGIN BULK\n");

	if (!bReorderID) {
		for (i = 0; i < np; i++) {
			fprintf(fout, "GRID*   %-32d%-16e%-16e*\n*       %-16e\n",
				i + firstnumber, ptlist[i * 3], ptlist[i * 3 + 1],
				ptlist[i * 3 + 2]);
		}

		for (i = 0; i < ntets; i++) {
			fprintf(fout, "CTETRA  %-8d%-8d", i + firstnumber, nPropertyId);
			for (k = 0, j = 4; k < numberofcorners; k++, j++) {
				if (j % 10 == 0) {
					fprintf(fout, "\n        ");
				}
				fprintf(fout, "%-8d", tetlist[i * numberofcorners + k]);
			}
			fprintf(fout, "\n");
		}
	}
	else {
		// assume:
		//  1. interior nodes are all placed after input nodes for (-pY), which
		//  is always true
		//  2. pointmarkerlist[np] keeps node identifiers (yes in mesh2PLC), 0
		//  for interior nodes
		int id;
		int *ind2id = new int[np];
		--firstnumber;
		for (i = 0; i < np; i++) {
			id = i; //tt.pointmarkerlist[i];
			if (id == 0)
				id = ++firstnumber;
			else
				firstnumber = std::max(id, firstnumber);
			ind2id[i] = id;
			fprintf(fout, "GRID*   %-32d%-16e%-16e*\n*       %-16e\n", id,
				ptlist[i * 3], ptlist[i * 3 + 1], ptlist[i * 3 + 2]);
		}

		firstnumber = tt.firstnumber;
		for (i = 0; i < ntets; i++) {
			fprintf(fout, "CTETRA  %-8d%-8d", i + firstnumber, nPropertyId);
			for (k = 0, j = 4; k < numberofcorners; k++, j++) {
				if (j % 10 == 0) fprintf(fout, "\n        ");
				id = tetlist[i * numberofcorners + k];  // index starting at 1
				fprintf(fout, "%-8d", ind2id[id - 1]);
			}
			fprintf(fout, "\n");
		}
		delete[] ind2id;
	}

	fprintf(fout, "ENDDATA\n");
	fclose(fout);
}

int main() {
	std::cout << "\n\nSALUTON MUNDO.\n\n";
	tetgenio_data_list planetsurface;
	planetsurface.points = {
		1.0,1.0,1.0,
		1.0,-1.0,-1.0,
		-1.0,1.0,-1.0,
		-1.0,-1.0,1.0,
		2.0,2.0,2.0,
		2.0,-2.0,-2.0,
		-2.0,2.0,-2.0,
		-2.0,-2.0,2.0
	};
	planetsurface.triangles = {
		0,1,2,
		0,3,1,
		0,2,3,
		1,3,2,
		4,5,6,
		4,7,5,
		4,6,7,
		5,7,6
	};
	tetgenio mesh_in, mesh_out;
	datalist_to_tetgenio(planetsurface, mesh_in);
	tetrahedralize("pYef", &mesh_in, &mesh_out);
	write_tetgenio_to_bdf(mesh_out,"farts.bdf");
	//tetra_save_bdf("farts42.bdf", mesh_out);
}