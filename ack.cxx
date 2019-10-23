#include "vector"
#include "iostream"
#include <tetgen.h>
#include "set"
#include "tuple"
#include "algorithm"
#include "array"
#include "tetgen.h" // Defined tetgenio, tetrahedralize().


void print(std::set<std::array<int, 2>> x) {
	for (auto y : x) {
		std::cout << y[0] << " " << y[1] <<"\n";
	}
}

void print_tet_attributes(tetgenio* out)
{
	printf("%d  %d  %d\n", out->numberoftetrahedra, out->numberofcorners,
		out->numberoftetrahedronattributes);
	int firstnumber = 0;
	for (int i = 0; i < out->numberoftetrahedra; i++) {
		printf("%d", i + firstnumber);
		for (int j = 0; j < out->numberofcorners; j++) {
			printf("  %5d", out->tetrahedronlist[i * out->numberofcorners + j]);
		}
		for (int j = 0; j < out->numberoftetrahedronattributes; j++) {
			printf("  %g",
				out->tetrahedronattributelist[i * out->numberoftetrahedronattributes + j]);
		}
		printf("\n");
	}
}


void print_tet_neighbors(tetgenio* out)
{

	printf("%d  %d\n", out->numberoftetrahedra, out->mesh_dim + 1);
	for (int i = 0; i < out->numberoftetrahedra; i++) {
		if (out->mesh_dim == 2) {
			printf("%d  %5d  %5d  %5d", i + out->firstnumber, out->neighborlist[i * 3],
				out->neighborlist[i * 3 + 1], out->neighborlist[i * 3 + 2]);
		}
		else {
			printf("%d  %5d  %5d  %5d  %5d", i + out->firstnumber,
				out->neighborlist[i * 4], out->neighborlist[i * 4 + 1],
				out->neighborlist[i * 4 + 2], out->neighborlist[i * 4 + 3]);
		}
		printf("\n");
	}
}



std::set<std::array<int,2>> get_region_adjacency_edges(tetgenio* tetin)
{
	// Extract a list of edges from the regions labeling the tetrahedra in 
	std::set<std::array<int, 2>> region_adjacency_edges;
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
			}
		}
	}
	return region_adjacency_edges;
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

	std::set<std::array<int, 2>> region_adj_graph = get_region_adjacency_edges(&out);
	print(region_adj_graph);
	return 0;
}

int main() {
	std::cout << "\n\nSALUTON MUNDO.\n\n";
	//example_from_wias();
	
	tetgenio orbs;
	////char input_file_path[38] = "C:\\Users\\sscott\\Pictures\\orbs.vtk";
	////orbs.load_vtk(input_file_path);
	//char input_file_path[38] = "C:\\Users\\sscott\\Pictures\\orbs.stl";
	char input_file_path[46] = "C:\\Users\\sscott\\Pictures\\nested_tetras.stl";
	orbs.load_stl(input_file_path);
	char ctrl_string[6] = "pAnkz";
	tetgenio volorbs;
	tetrahedralize(ctrl_string, &orbs, &volorbs);
	auto region_adjacency_edges = get_region_adjacency_edges(&volorbs);
	print(region_adjacency_edges);
	volorbs.save_elements("appler");
	volorbs.save_neighbors("appler");
	volorbs.
	return 0;
}