#include "vector"
#include "iostream"
#include <tetgen.h>
#include "set"
#include "map"
#include "algorithm"
#include "array"
#include "queue"
#include "tetgen.h" // Defined tetgenio, tetrahedralize().

void print(std::set<std::array<int, 2>> x) {
	for (auto y : x) {
		std::cout << y[0] << " " << y[1] <<"\n";
	}
}

struct graph_adjacency_list {
	std::set<int> vertices;
	std::set<std::array<int, 2>> edges;
};

void print(graph_adjacency_list x) {
	print(x.edges);
}

std::map<int, bool> compute_two_coloring(graph_adjacency_list g, int root=0) {
	// Computes a 2-coloring of a graph from the adjacency list.
	// The colors are True, False and the root is colored False by convention.
	// Input adjacency list must represent a tree.
	// This does not check connectivity or 
	std::map<int, bool> coloring;
	coloring[root] = false;
	//Copy edges to a que
	std::queue<std::array<int,2>> edge_que;
	for (auto edge : g.edges) {
		edge_que.push(edge);
	}
	//For every edge, color the vertices opposite if either has a color.
	while (!edge_que.empty()) {
		auto edge = edge_que.front();
		edge_que.pop();
		//Scan coloring for a vertex of the edge. Color if one vertex found. Reque if not found.
		if (coloring.find(edge[0]) != coloring.end()) coloring[edge[1]] = !coloring[edge[0]];
		else if (coloring.find(edge[1]) != coloring.end()) coloring[edge[0]] = !coloring[edge[1]];
		else edge_que.push(edge);
	}
	return coloring;
}

void print_tet_attributes(tetgenio* tetg)
{
	printf("%d  %d  %d\n", tetg->numberoftetrahedra, tetg->numberofcorners,
		tetg->numberoftetrahedronattributes);
	int firstnumber = tetg->firstnumber;
	for (int i = 0; i < tetg->numberoftetrahedra; i++) {
		printf("%d", i + firstnumber);
		for (int j = 0; j < tetg->numberofcorners; j++) {
			printf("  %5d", tetg->tetrahedronlist[i * tetg->numberofcorners + j]);
		}
		for (int j = 0; j < tetg->numberoftetrahedronattributes; j++) {
			printf("  %g",
				tetg->tetrahedronattributelist[i * tetg->numberoftetrahedronattributes + j]);
		}
		printf("\n");
	}
}

void print_tet_neighbors(tetgenio* tetg)
{

	printf("%d  %d\n", tetg->numberoftetrahedra, tetg->mesh_dim + 1);
	for (int i = 0; i < tetg->numberoftetrahedra; i++) {
		if (tetg->mesh_dim == 2) {
			printf("%d  %5d  %5d  %5d", i + tetg->firstnumber, tetg->neighborlist[i * 3],
				tetg->neighborlist[i * 3 + 1], tetg->neighborlist[i * 3 + 2]);
		}
		else {
			printf("%d  %5d  %5d  %5d  %5d", i + tetg->firstnumber,
				tetg->neighborlist[i * 4], tetg->neighborlist[i * 4 + 1],
				tetg->neighborlist[i * 4 + 2], tetg->neighborlist[i * 4 + 3]);
		}
		printf("\n");
	}
}

std::vector<int> filter_tets_by_region (tetgenio* tetg, std::set<int> regions) {
	//Return a list of all tets in the regions listed
	std::vector<int> tets_found;
	for (int tet_i = 0; tet_i < tetg->numberoftetrahedra; tet_i++) {
		int reg_i = tetg->tetrahedronattributelist[tet_i * tetg->numberoftetrahedronattributes];
		if (regions.find(reg_i) != regions.end()) {
			tets_found.push_back(tet_i);
		}
	}
	return tets_found;
}

graph_adjacency_list get_region_adjacency_edges(tetgenio* tetin)
{
	// Extract a list of edges from the regions labeling the tetrahedra in 
	std::set<std::array<int, 2>> region_adjacency_edges;
	std::set<int> regions;
	for (int i = 0; i < tetin->numberoftetrahedra; i++) {
		int region_i = tetin->tetrahedronattributelist[i];
		for (int j = 0; j < 4; j++) {
			int ngb_id = tetin->neighborlist[i * 4 + j];
			int region_j;
			if (ngb_id >= 0) {
				region_j = tetin->tetrahedronattributelist[ngb_id];
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
	graph_adjacency_list g;
	g.vertices = regions;
	g.edges = region_adjacency_edges;
	return g;
}

void filter_tetrahedra(tetgenio* tetin, tetgenio* tetout, std::vector<int> tets_kept) {
	//Copy over all the kept tetrahedra together with their points.
	std::set<int> pnts_kept;
	tetout->firstnumber = 0;
	//First traverse the tets of tetin and find the points of kept tets.
	for (int tet : tets_kept) {
		for (int j = 0; j < 4; j++) {
			pnts_kept.insert(tetin->tetrahedronlist[4 * tet + j]);
		}
	}
	//Relabel the points
	std::map<int, int> pnt_id_map;
	int pnt_counter = 0;
	for (int pnt : pnts_kept) {
		pnt_id_map[pnt] = pnt_counter;
		pnt_counter++;
	}	
	//Copy the kept  points into a new tetgenio with updated ids.
	//tetgenio tetout;
	tetout->numberofpoints = pnts_kept.size();
	tetout->numberofcorners = 4;
	tetout->pointlist = new REAL[tetout->numberofpoints * 3];
	for (int pnt : pnts_kept) {
		int pnt_new_id = pnt_id_map[pnt];
		for (int j = 0; j < 3; j++) {
			tetout->pointlist[3 * pnt_new_id + j] = tetin->pointlist[3 * pnt + j];
		}
	}
	//std::cout << "\n";
	//Copy the kept tets into a new tetgenio with updated ids.
	tetout->numberoftetrahedra = tets_kept.size();
	tetout->tetrahedronlist = new int[tetout->numberoftetrahedra * 4];
	int tet_counter = 0;
	for (int tet : tets_kept) {
		for (int j = 0; j < 4; j++) {
			//std::cout<< tetin->tetrahedronlist[4 * tet + j] << " |-> " << pnt_id_map[tetin->tetrahedronlist[4 * tet + j]] <<", ";
			tetout->tetrahedronlist[4 * tet_counter + j] = pnt_id_map[tetin->tetrahedronlist[4 * tet + j]];
		}
		tet_counter++;
	}
	for (int i = 0;i < 4 * tets_kept.size(); i++) {
		std::cout << tetout->tetrahedronlist[i] << " ";
	}
	//return tetout;
}


int example_from_wias()
{
	tetgenio in, out;
	tetgenio::facet *f;
	tetgenio::polygon *p;
	int i;

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
	for (i = 4; i < 8; i++) {
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

	// Output the PLC to files 'barin.node' and 'barin.poly'.
	//in.save_nodes("barin");
	//in.save_poly("barin");

	// Tetrahedralize the PLC. Switches are chosen to read a PLC (p),
	//   do quality mesh generation (q) with a specified quality bound
	//   (1.414), and apply a maximum volume constraint (a0.1).

	tetrahedralize("pq1.414a0.1Ank", &in, &out);

	// Output mesh to files 'barout.node', 'barout.ele' and 'barout.face'.
	out.save_nodes("barout");
	out.save_elements("barout");
	out.save_faces("barout");
	out.save_neighbors("barout");
	std::cout << "\n" << out.numberofpointattributes <<"\n";

	print_tet_neighbors(&out);

	graph_adjacency_list region_adj_graph = get_region_adjacency_edges(&out);
	print(region_adj_graph);
	return 0;
}

int main() {
	std::cout << "\n\nSALUTON MUNDO.\n\n";
	//example_from_wias();
	tetgenio orbs;
	orbs.firstnumber = 0;
	////char input_file_path[38] = "C:\\Users\\sscott\\Pictures\\orbs.vtk";
	////orbs.load_vtk(input_file_path);
	//char input_file_path[38] = "C:\\Users\\sscott\\Pictures\\orbs.stl";
	//char input_file_path[46] = "C:\\Users\\sscott\\Pictures\\nested_tetras.stl";
	char input_file_path[47] = "C:\\Users\\sscott\\Pictures\\nested_spheres.stl";
	orbs.load_stl(input_file_path);
	std::cout << "edges orbs " << orbs.numberofedges << "\n";

	char ctrl_string[6] = "zAnpk";
	tetgenio volorbs;
	volorbs.firstnumber = 0;
	//tetrahedralize("k", &volorbs, NULL);
	tetrahedralize(ctrl_string, &orbs, &volorbs);
	std::cout<< "volorb first numb: "<<volorbs.firstnumber<<"\n";
	volorbs.save_nodes("volorb");
	volorbs.save_elements("volorb");
	std::cout << "edges volorbs " << volorbs.numberofedges << "\n";
	//tetrahedralize("kz", &volorbs, NULL);
	//print_tet_attributes(&volorbs);
	// 2-color the tetmesh's regions with {True,False} according to if we should keep them.
	auto region_adjacency_edges = get_region_adjacency_edges(&volorbs);
	print(region_adjacency_edges);
	auto coloring = compute_two_coloring(region_adjacency_edges);
	// Find the regions that are colored False so that we can delete them.
	std::set<int> regions_kept;
	std::map<int, bool>::iterator it = coloring.begin();
	while (it != coloring.end())
	{
		if (it->second) {
			regions_kept.insert(it->first);
			std::cout << "keep region " << it->first << "\n";
		}
		it++;
	}
	std::vector<int> tets_kept = filter_tets_by_region(&volorbs, regions_kept);
	tetgenio carved;
	filter_tetrahedra(&volorbs, &carved, tets_kept);
	for (int i = 0;i < 3 * carved.numberofpoints; i++) {
		std::cout << carved.pointlist[i] << " ";
	}
	std::cout << "\n";
	carved.save_nodes("carved");
	carved.save_elements("carved");
	std::cout <<"edges  carved "<< carved.numberofedges<<"\n";
	tetgenio nice;
	nice.firstnumber = 0;
	tetrahedralize("rkzKO/10C", &carved, &nice);
	nice.save_nodes("nice");
	nice.save_elements("nice");
	std::cout << "edges nice " << nice.numberofedges << "\n";
	nice.save_poly("poly");
	return 0;
}