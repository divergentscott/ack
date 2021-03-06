#include <iostream>
#include <limits>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "zoning_cartographer_svg.h"

std::string path_prefix = "";

namespace exda{
    std::vector<Eigen::Vector2d> grid_points = {
        {0, 0},
        {5, 0},
        {5, 1},
        {6, 1},
        {6, 2},
        {5, 2},
        {5, 3},
        {1, 3},
        {1, 2},
        {2, 2},
        {2, 1},
        {0, 1},
        {3, 1},
        {4, 1},
        {4, 2},
        {3, 2},
    };

    std::vector<std::vector<int>> lines = {
        {0,1},
        {1,2},
        {2,3},
        {3,4},
        {4,5},
        {5,6},
        {6,7},
        {7,8},
        {8,9},
        {9,10},
        {10,11},
        {11,0},
        {12,13},
        {13,14},
        {14,15},
        {15,12}
    };

    std::vector<Eigen::Vector2d> splotchpoints = {
        {0,0},
        {2,0},
        {2,-1},
        {5,-1},
        {5,3},
        {6,3},
        {6,1},
        {7,1},
        {7,0},
        {8,0},
        {8,1},
        {7.5,1},
        {7.5,4},
        {1,4},
        {1,2},
        {0,2}
    };

    std::vector<std::vector<int>> splotchlines = {
        {0,1},
        {1,2},
        {2,3},
        {3,4},
        {4,5},
        {5,6},
        {6,7},
        {7,8},
        {8,9},
        {9,10},
        {10,11},
        {11,12},
        {12,13},
        {13,14},
        {14,15},
        {15,0}
    };

}

void example1(){
    CardinalCurveCollection ccc;
    ccc.setGridPointsAndCells(exda::grid_points, exda::lines);
    std::cout << ccc.get_number_of_points();
    for (int foo=0; foo<exda::lines.size(); foo++){
        std::cout << "e " << foo << " prev " << ccc.get_prev_edge(foo) << " next " << ccc.get_next_edge(foo) << std::endl;
    }
    std::cout << "true is " << true << std::endl;
    std::cout << "false is " << false << std::endl;
    std::cout << "(3.5,1.5) is out " << ccc.is_point_in({3.5,1.5}) << std::endl;
    std::cout << "(2.5,1.5) is in " << ccc.is_point_in({2.5,1.5}) << std::endl;
    std::cout << "(1.5,1.5) is out " << ccc.is_point_in({1.5,1.5}) << std::endl;

    // ccc should have a cyclic ordering of the points
    // if we have a point to edge look up we can manage a traversal
};

void example2(){
    ZoningCommisioner vac;
    vac.insertCurves(exda::grid_points, exda::lines);
    vac.populateNeighbors();
    std::cout << "frontier west" << vac.westmost_frontier_id_ << std::endl;
    std::cout << "frontire east" << vac.eastmost_frontier_id_ << std::endl;
    
    for (auto foo=0; foo<vac.vacant_.get_number_of_edges(); foo++){
        std::cout << foo << std::endl;
		auto p_ids = vac.vacant_.get_points_of_edge(foo);
		Eigen::Vector2d p0 = vac.vacant_.get_point(p_ids[0]);
		Eigen::Vector2d p1 = vac.vacant_.get_point(p_ids[1]);
		std::cout << " pt " << p_ids[0] << " @ " << p0.transpose() << std::endl;
        std::cout << " pt " << p_ids[1] << " @ " << p1.transpose() << std::endl;
        if (vac.shapes_[foo] == NeighborhoodShape::kZag)            std::cout << "zag" << std::endl;
        if (vac.shapes_[foo] == NeighborhoodShape::kNotchNorth)            std::cout << "north" << std::endl;
        if (vac.shapes_[foo] == NeighborhoodShape::kNotchEast)            std::cout << "east" << std::endl;
        if (vac.shapes_[foo] == NeighborhoodShape::kNotchSouth)            std::cout << "south" << std::endl;
        if (vac.shapes_[foo] == NeighborhoodShape::kNotchWest)            std::cout << "west" << std::endl;
    }
    std::cout << "Patrol down from 11 " << std::endl;
    Zone patty11 = vac.zoneFromEastNotch(11);
    Zone patty7 = vac.zoneFromEastNotch(7);
};

void example4(){
    ZoningCommisioner vac;
    vac.insertCurves(exda::splotchpoints, exda::splotchlines);
    vac.populateNeighbors();
    Zone peopleheadeast = vac.zoneFromEastNotch(15);
    std::cout << "downtown landmarks: " << std::endl;
    for (auto p : peopleheadeast.landmarks_downtown_.points_){
        std::cout << p.transpose() << std::endl;
    }
    std::cout << "uptown landmarks: " << std::endl;
    for (auto p : peopleheadeast.landmarks_uptown_.points_){
        std::cout << p.transpose() << std::endl;
    }
    PlankHiker calj(peopleheadeast.landmarks_downtown_, 0.5, true);//3.3, false);
    calj.hike();
//    calj.hikeTraildowntown();
    std::cout << "downtown camps:" << std::endl;
    for (auto p : calj.camps_.points_){
        std::cout << p.transpose() << std::endl;
    };
    /*
     downtown camps:
     0    0
     1.7   0
     1.7   3
     6 3
     6 1
     */
    PlankHiker calk(peopleheadeast.landmarks_uptown_, 0.5, false);//3.3, false);
    calk.hike();
//    calj.hikeTraildowntown();
    std::cout << "uptown camps:" << std::endl;
    for (auto p : calk.camps_.points_){
        std::cout << p.transpose() << std::endl;
    };
    /*
     uptown camps:
     0    2
     1 2
     1 4
     4.2   4
     4.2   1
     */
};

void example5(double width = 0.5, double height = 0.5, std::string afile = "/Users/sscott/Programs/ack/ackler2.svg"){
    ZoningCommisioner vac;
    vac.insertCurves(exda::splotchpoints, exda::splotchlines);
    vac.populateNeighbors();
    Zone peopleheadeast = vac.zoneFromEastNotch(15);
    Surveyor calh(peopleheadeast,width, height);
    calh.hike();
    svgvis::ZoningCartographerSVG vv;
    std::vector<std::string> funcolors = {"#F75C03", "#D90368", "#820263", "#291720", "#04A777"};
    vv.addCardinalCurveCollection(vac.vacant_, funcolors[3]);
    vv.addCardinalPath(calh.camps_downtown_, funcolors[0]);
    vv.addCardinalPath(calh.camps_uptown_,  funcolors[1]);
    vv.addRectangles(calh.valid_lots_, width, height, funcolors[4]);
    vv.writeScalableVectorGraphics(afile);
    std::cout << "Valids: " << std::endl;
    for (auto p : calh.valid_lots_){
        std::cout << p.transpose() << std::endl;
    }
    /*
     With width 0.5 height 0.4
     0 0
     1 0
     2 0
      2 -1
     4.5  -1
     4.5   3
     6 3
     6 1
     7 1
     7 0
     */
};

void example7(double width = 3.4, double height = 2.1){
    std::vector<Eigen::Vector2d> notch_north_pts = {
        {0,0},
        {1,0},
        {1,-1},
        {2,-1},
    };
    
    std::vector<Eigen::Vector2d> notchy_points;
    for (int foo=0; foo< 20; foo++){
        for (auto p : notch_north_pts){
            auto q = p + Eigen::Vector2d({2*foo,0});
            notchy_points.push_back(q);
        }
    }
    notchy_points.push_back({40, 10});
    notchy_points.push_back({5, 10});
    notchy_points.push_back({5, 20});
    notchy_points.push_back({40, 20});
    notchy_points.push_back({40, 30});
    notchy_points.push_back({0, 30});

    for (auto foo: notchy_points){
        std::cout << foo.transpose() << std::endl;
    }
    int npts = notchy_points.size();
    std::vector<std::vector<int>> notchy_lines;
    for (int foo=0; foo < npts; foo++){
        notchy_lines.push_back({foo, (foo+1)%npts});
    }
    ZoningCommisioner vac;
    std::vector<Eigen::Vector2d> notchy_points_arraysstyle;
    for (auto foo : notchy_points){
        notchy_points_arraysstyle.push_back({foo[0],foo[1]});
    }
    vac.insertCurves(notchy_points_arraysstyle, notchy_lines);
    vac.populateNeighbors();
    vac.constructZoneCovering();
    Surveyor calh(vac.zones_[0], width, height);
    calh.hike();
    svgvis::ZoningCartographerSVG vv;
    std::vector<std::string> funcolors = {"#F75C03", "#D90368", "#820263", "#291720", "#04A777"};
    
    vv.addCardinalCurveCollection(vac.vacant_, funcolors[3]);
    vv.addCardinalPath(calh.camps_downtown_, funcolors[0]);
    vv.addCardinalPath(calh.camps_uptown_,  funcolors[1]);
    vv.addRectangles(calh.valid_lots_, width, height, funcolors[4]);
    vv.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/notchy.svg");
    
};



void example_basic(std::vector<Eigen::Vector2d> points, double width, double height,  std::string filename){
    int npts = points.size();
    std::vector<std::vector<int>> lines;
    for (int foo=0; foo < npts; foo++){
        lines.push_back({foo, (foo+1)%npts});
    }
    ZoningCommisioner vac;
    vac.insertCurves(points, lines);
    vac.populateNeighbors();
    vac.findFrontier();
    std::cout << vac.zones_.size() << std::endl;
    if (vac.zones_.size() > 0){
        
    }
    Surveyor calh(vac.zones_[0], width, height);
    calh.hike();
    svgvis::ZoningCartographerSVG vv;
    std::vector<std::string> funcolors = {"#F75C03", "#D90368", "#820263", "#291720", "#04A777"};
//
    vv.addCardinalCurveCollection(vac.vacant_, funcolors[3]);
    vv.addCardinalPath(calh.camps_downtown_, funcolors[0]);
    vv.addCardinalPath(calh.camps_uptown_,  funcolors[1]);
    if (calh.valid_lots_.size() > 0) vv.addRectangle(calh.getMostValid(), width, height, funcolors[4]);
    vv.writeScalableVectorGraphics(filename);
//
//
}

void example_basic(std::vector<Eigen::Vector2d> points, std::vector<std::vector<int>> lines, double width, double height,  std::string filename){
    ZoningCommisioner vac;
    vac.insertCurves(points, lines);
    vac.populateNeighbors();
    vac.findFrontier();
//    std::cout << "Numer of trails: " << vac.trails_.size() << std::endl;
    svgvis::ZoningCartographerSVG vv;
    std::vector<std::string> funcolors = {"#F75C03", "#D90368", "#820263", "#291720", "#04A777"};
//
    vv.addCardinalCurveCollection(vac.vacant_, funcolors[3]);
    for (auto tt: vac.zones_){
        Surveyor calh(tt, width, height);
        calh.hike();
        vv.addCardinalPath(calh.camps_downtown_, funcolors[0]);
        vv.addCardinalPath(calh.camps_uptown_,  funcolors[1]);
        vv.addRectangle(calh.getMostValid(), width, height, funcolors[4]);
    }
    vv.writeScalableVectorGraphics(filename);
}


void example_sev(){
    std::vector<Eigen::Vector2d> points = {{0,0},{10,0},{10,10},{0,10}};
    example_basic(points, 3.3,2,path_prefix + "/Users/sscott/Programs/ack/basic0.svg");
    points = {{0,2},{3.3,2},{3.3,0},{10,0},{10,10},{0,10}};
    example_basic(points, 3.2,1.6,path_prefix + "/Users/sscott/Programs/ack/basic1.svg");
    points = {{0,2},{3.3,2},{3.3,1.6},{6.5,1.6},{6.5,0},{10,0},{10,10},{0,10}};
    example_basic(points, 3.1,2.3,path_prefix + "/Users/sscott/Programs/ack/basic2.svg");
    points = {{0,2},{3.3,2},{3.3,1.6},{6.5,1.6},{6.5,2.3},{9.6,2.3},{9.6,0},{10,0},{10,10},{0,10}};
    example_basic(points, 7,1.2,path_prefix + "/Users/sscott/Programs/ack/basic3.svg");
    points ={{0,3.5}, {7,3.5}, {7,2.3}, {9.6,2.3}, {9.6,0}, {10,0}, {10,10}, {0,10}, {0,2}, {3.3,2}, {3.3,1.6}, {6.5,1.6}, {6.5,2.3}, {0, 2.3}};
    std::vector<std::vector<int>> somelines = {{0,1}, {1,2}, {2,3}, {3,4}, {4,5}, {5,6}, {6,7}, {7,0}, {8,9}, {9,10}, {10,11}, {11,12}, {12,13}, {13,8}};
    example_basic(points, somelines, 0.3, 0.6,path_prefix + "/Users/sscott/Programs/ack/basic4.svg");
}

void example_9(){
    std::vector<Eigen::Vector2d> points ={{0,3.5}, {7,3.5}, {7,2.3}, {9.6,2.3}, {9.6,0}, {10,0}, {10,10}, {0,10}, {0,2}, {3.3,2}, {3.3,1.6}, {6.5,1.6}, {6.5,2.3}, {0, 2.3}};
    std::vector<std::vector<int>> somelines = {{0,1}, {1,2}, {2,3}, {3,4}, {4,5}, {5,6}, {6,7}, {7,0}, {8,9}, {9,10}, {10,11}, {11,12}, {12,13}, {13,8}};
    ZoningCommisioner wild;
    wild.insertCurves(points,somelines);
    wild.populateNeighbors();
    wild.constructZoneCovering();
    Eigen::Vector2d place;
    bool is_placeable = wild.findPlacement(0.3, 0.6, place);
    if (is_placeable){
        std::cout << "Can place." << std::endl;
        svgvis::ZoningCartographerSVG vv;
        vv.addCardinalCurveCollection(wild.vacant_, "black");
        for (const Zone& t: wild.zones_){
            vv.addCardinalPath(t.landmarks_downtown_, "green");
            vv.addCardinalPath(t.landmarks_uptown_, "blue");
        }
        vv.addRectangle(place, 0.3, 0.6, "red");
        vv.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/ex9.svg");
        std::cout << "Placed at " << place.transpose() << std::endl;
    }
}

void example_10(){
    std::vector<Eigen::Vector2d> points = {{0,1},{1,1},{1,0},{10,0},{10,10},{0,10}};
    std::vector<std::vector<int>> somelines = {{0,1},{1,2},{2,3},{3,4},{4,5},{5,0}};
    //
    ZoningCommisioner wild;
    wild.insertCurves(points, somelines);
    wild.populateNeighbors();
    wild.constructZoneCovering();

    Zone t = wild.zones_[0];
    std::cout << " Before " << std::endl;
    for (auto p :  t.landmarks_downtown_.points_){
        std::cout << p.transpose() << std::endl;
    }

    Eigen::Vector2d placement;
    double width = 2.4;
    double height = 1.7;
    wild.findPlacement(width, height, placement);
//    wild.removeRectangle(placement, width, height);
    t = wild.zones_[0];
//    removeRectangle(wild.trails_, placement, width, height);
    std::cout << " After " << std::endl;
    for (auto p :  t.landmarks_downtown_.points_){
        std::cout << p.transpose() << std::endl;
    }
    //
};

void example_11(){
    std::vector<Eigen::Vector2d> points = {{0,0},{10,0},{10,10},{0,10}};
    std::vector<std::vector<int>> somelines = {{0,1},{1,2},{2,3},{3,0}};
    //
    ZoningCommisioner wild;
    wild.insertCurves(points, somelines);
    wild.populateNeighbors();
    wild.constructZoneCovering();
    //
    svgvis::ZoningCartographerSVG wsvg;
    wsvg.addCardinalCurveCollection(wild.vacant_);
	std::vector<Eigen::Vector2d> whs = {{5.4, 2.1}, {4.1, 2.4}, {3.5, 1.9}, {2.5, 2.5}, {1.9, 1.3}, {0.9, 0.8} };
    for (auto wh : whs){
        double width = wh[0];
        double height = wh[1];
        Eigen::Vector2d placement;
        wild.findPlacement(width, height, placement);
        for (auto &t: wild.zones_){
            wsvg.addCardinalPath(t.landmarks_uptown_, "blue");
            wsvg.addCardinalPath(t.landmarks_downtown_, "green");
        }
        //removeRectangle(wild.trails_, placement, width, height);
		wild.zoneOff(placement, width, height);
        wsvg.addRectangle(placement, width, height, svgvis::chaosHex());
    }
    wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example11.svg");
}

void example_12() {
	std::vector<Eigen::Vector2d> points = { {0,0},{10,0},{10,10},{0,10} };
	std::vector<std::vector<int>> somelines = { {0,1},{1,2},{2,3},{3,0} };
	//
	ZoningCommisioner wild;
	wild.insertCurves(points, somelines);
	wild.populateNeighbors();
	wild.constructZoneCovering();
	//
	svgvis::ZoningCartographerSVG wsvg;
	wsvg.addCardinalCurveCollection(wild.vacant_);
    std::vector<Eigen::Vector2d> whs = { {6.9, 4.4} , { 6.6,3.3 } , {5.4, 2.1},  {4.1, 2.4} , {3.5, 1.9} , {2.5, 2.5}, {1.9, 1.3}, {0.9, 0.8}};
    for (int foo=0; foo<20; foo++) whs.push_back({0.8,0.8});
	for (auto foo=0; foo<29; foo++) whs.push_back({ 0.4,0.4 });
	//std::vector<Eigen::Vector2d> whs = { {6.9, 4.4}};
	for (auto wh : whs)  {
		double width = wh[0];
		double height = wh[1];
		Eigen::Vector2d placement;
		bool is_placable = wild.findPlacement(width, height, placement);
		//for (auto &t : wild.trails_) {
		//	wsvg.addCardinalPath(t.landmarks_uptown_, "blue");
		//	wsvg.addCardinalPath(t.landmarks_downtown_, "green");
		//	std::cout << "svgbox: " << wsvg.min_x_ << " " << wsvg.min_y_ << " " << wsvg.max_x_ << " " << wsvg.max_y_ << std::endl;
		//}
		//removeRectangle(wild.trails_, placement, width, height);
		if (is_placable) {
			wild.zoneOff(placement, width, height);
			wsvg.addRectangle(placement, width, height);
			wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example12.svg");
		}
		else {
			wsvg.addRectangle({11,0}, width, height, svgvis::chaosHex());
		}
	}
    std::cout << "VACANCY POINTS" << std::endl;
    for (auto p : wild.vacant_.points_){
        std::cout << "{ " << p[0] << ", " << p[1] << " }," << std::endl;
    }
    std::cout << "VACANCY EDGES" << std::endl;
    for (auto ee : wild.vacant_.edges_){
        std::cout << "{ " << ee[0] << ", " << ee[1] << " }," << std::endl;
    }

	wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example12.svg");
}

void example_125() {
	std::vector<Eigen::Vector2d> points = { {0,0},{10,0},{10,10},{0,10} };
	std::vector<std::vector<int>> somelines = { {0,1},{1,2},{2,3},{3,0} };
	//
	ZoningCommisioner wild;
	wild.insertCurves(points, somelines);
	wild.populateNeighbors();
	wild.constructZoneCovering();
	//
	svgvis::ZoningCartographerSVG wsvg;
	wsvg.addCardinalCurveCollection(wild.vacant_);
	std::vector<Eigen::Vector2d> whs = {
			{9, 6},
			{ 9, 2 },
			{ 8, 6 },
			{ 8, 1 },
			{ 7, 6 },
			{ 6, 6 },
			{ 5, 5 },
			{ 5, 1 },
			{ 4, 4 },
			{ 4, 4 },
			{ 4, 2 },
			{ 3, 3 },
			{ 3, 2 },
			{ 3, 1 },
			{ 2, 2 },
			{ 2, 2 },
			{ 1, 1 },
			{ 1, 1 },
			{ 1, 1 },
			{ 1, 1 },
			{ 1, 1 },
			{ 1, 1 },
	};
	//std::vector<Eigen::Vector2d> whs = { {6.9, 4.4}};
	for (auto wh : whs) {
		double width = wh[0];
		double height = wh[1];
		Eigen::Vector2d placement;
		bool is_placable = wild.findPlacement(width, height, placement);
		//for (auto &t : wild.trails_) {
		//	wsvg.addCardinalPath(t.landmarks_uptown_, "blue");
		//	wsvg.addCardinalPath(t.landmarks_downtown_, "green");
		//	std::cout << "svgbox: " << wsvg.min_x_ << " " << wsvg.min_y_ << " " << wsvg.max_x_ << " " << wsvg.max_y_ << std::endl;
		//}
		//removeRectangle(wild.trails_, placement, width, height);
		if (is_placable) {
			wild.zoneOff(placement, width, height);
			wsvg.addRectangle(placement, width, height);
			wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example125.svg");
		}
		else {
			wsvg.addRectangle({ 11,0 }, width, height, svgvis::chaosHex());
		}
	}
	std::cout << "VACANCY POINTS" << std::endl;
	for (auto p : wild.vacant_.points_) {
		std::cout << "{ " << p[0] << ", " << p[1] << " }," << std::endl;
	}
	std::cout << "VACANCY EDGES" << std::endl;
	for (auto ee : wild.vacant_.edges_) {
		std::cout << "{ " << ee[0] << ", " << ee[1] << " }," << std::endl;
	}
	wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example125.svg");
}


void example_13() {
//    VACANCY POINTS
    std::vector<Eigen::Vector2d> points = {
		{ 8.5, 4.1 },
        { 8.5, 3.8 },
        { 8.8, 3.8 },
        { 8.8, 4.1 },
        { 8.5, 4.6 },
        { 8.2, 4.6 },
        { 8.2, 4.9 },
        { 8.5, 4.9 },
        { 8.9, 7.7 },
        { 6.6, 7.7 },
        { 6.6, 7 },
        { 8.2, 7 },
        { 8.2, 7.3 },
        { 8.9, 7.3 },
        { 9.7, 7.3 },
        { 9.8, 7.3 },
        { 9.8, 4.9 },
        { 9.3, 4.9 },
        { 9.3, 4.1 },
        { 9.6, 4.1 },
        { 9.6, 3.3 },
        { 9.7, 3.3 },
        { 9.7, 2.5 },
        { 9.4, 2.5 },
        { 9.4, 0 },
        { 10, 0 },
        { 10, 10 },
        { 0, 10 },
        { 0, 9.8 },
        { 5.4, 9.8 },
        { 5.4, 9.6 },
        { 8.9, 9.6 },
        { 8.9, 9.7 },
        { 9.7, 9.7 },
        { 6.9, 4.6 },
        { 6.9, 4.4 },
        { 6.6, 4.4 },
        { 6.6, 4.6 }
    };
//    VACANCY EDGES
    std::vector<std::vector<int>> somelines = {
		{ 0, 1 },
        { 1, 2 },
        { 2, 3 },
        { 3, 0 },
        { 4, 5 },
        { 5, 6 },
        { 6, 7 },
        { 7, 4 },
        { 8, 9 },
        { 9, 10 },
        { 10, 11 },
        { 11, 12 },
        { 12, 13 },
        { 13, 8 },
        { 14, 15 },
        { 15, 16 },
        { 16, 17 },
        { 17, 18 },
        { 18, 19 },
        { 19, 20 },
        { 20, 21 },
        { 21, 22 },
        { 22, 23 },
        { 23, 24 },
        { 24, 25 },
        { 25, 26 },
        { 26, 27 },
        { 27, 28 },
        { 28, 29 },
        { 29, 30 },
        { 30, 31 },
        { 31, 32 },
        { 32, 33 },
        { 33, 14 },
        { 34, 35 },
        { 35, 36 },
        { 36, 37 },
        { 37, 34 }
    };
    //
    ZoningCommisioner wild;
    wild.insertCurves(points, somelines);
    wild.populateNeighbors();
    wild.constructZoneCovering();
    //
    svgvis::ZoningCartographerSVG wsvg;
    wsvg.addCardinalCurveCollection(wild.vacant_);
    std::vector<Eigen::Vector2d> whs = {{0.4,0.4}};
    for (auto wh : whs)  {
        double width = wh[0];
        double height = wh[1];
        Eigen::Vector2d placement;
        bool is_placable = wild.findPlacement(width, height, placement);
		int trail_cnt = 0;
		for (auto &t : wild.zones_) {
			svgvis::ZoningCartographerSVG svger;
			svger.addCardinalCurveCollection(wild.vacant_, "black");
			svger.addCardinalPath(t.landmarks_uptown_, "blue");
			svger.addCardinalPath(t.landmarks_downtown_, "green");
			Surveyor caliper_hiker(t, width, height);
			caliper_hiker.hike();
			svger.addCardinalPath(caliper_hiker.camps_uptown_, "orange");
			svger.addCardinalPath(caliper_hiker.camps_downtown_, "red");
			svger.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example13_"+std::to_string(trail_cnt)+".svg");
			trail_cnt++;
		}
		//for (auto &t : wild.trails_) {
  //          wsvg.addCardinalPath(t.landmarks_uptown_, "blue");
  //          wsvg.addCardinalPath(t.landmarks_downtown_, "green");
		//	CaliperHiker caliper_hiker(t, width, height);
		//	caliper_hiker.hike();
		//	wsvg.addCardinalPath(caliper_hiker.camps_uptown_, "black");
		//	wsvg.addCardinalPath(caliper_hiker.camps_downtown_, "red");
  //      }
        wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example13.svg");
        //removeRectangle(wild.trails_, placement, width, height);
        if (is_placable) {
            wild.zoneOff(placement, width, height);
            wsvg.addRectangle(placement, width, height);
            wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example13.svg");
        } else {
            wsvg.addRectangle({11,0}, width, height);
            wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example13.svg");
        }
    }
    wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example13.svg");
}

void example_132() {
//    VACANCY POINTS
    std::vector<Eigen::Vector2d> points = {

    };
//    VACANCY EDGES
    std::vector<std::vector<int>> somelines = {

    };
    //
    ZoningCommisioner wild;
    wild.insertCurves(points, somelines);
    wild.populateNeighbors();
    wild.constructZoneCovering();
    //
    svgvis::ZoningCartographerSVG wsvg;
    wsvg.addCardinalCurveCollection(wild.vacant_);
    std::vector<Eigen::Vector2d> whs = {{0.4,0.4}};
    for (auto wh : whs)  {
        double width = wh[0];
        double height = wh[1];
        Eigen::Vector2d placement;
        bool is_placable = wild.findPlacement(width, height, placement);
        int trail_cnt = 0;
        for (auto &t : wild.zones_) {
            svgvis::ZoningCartographerSVG svger;
            svger.addCardinalCurveCollection(wild.vacant_, "black");
            svger.addCardinalPath(t.landmarks_uptown_, "blue");
            svger.addCardinalPath(t.landmarks_downtown_, "green");
            Surveyor caliper_hiker(t, width, height);
            caliper_hiker.hike();
            svger.addCardinalPath(caliper_hiker.camps_uptown_, "orange");
            svger.addCardinalPath(caliper_hiker.camps_downtown_, "red");
            svger.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example132_"+std::to_string(trail_cnt)+".svg");
            trail_cnt++;
        }
        //for (auto &t : wild.trails_) {
        //          wsvg.addCardinalPath(t.landmarks_uptown_, "blue");
        //          wsvg.addCardinalPath(t.landmarks_downtown_, "green");
        //	CaliperHiker caliper_hiker(t, width, height);
        //	caliper_hiker.hike();
        //	wsvg.addCardinalPath(caliper_hiker.camps_uptown_, "black");
        //	wsvg.addCardinalPath(caliper_hiker.camps_downtown_, "red");
        //      }
        wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example132.svg");
        //removeRectangle(wild.trails_, placement, width, height);
        if (is_placable) {
            wild.zoneOff(placement, width, height);
            wsvg.addRectangle(placement, width, height);
            wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example132.svg");
        } else {
            wsvg.addRectangle({11,0}, width, height);
            wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example132.svg");
        }
    }
    wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example132.svg");
}

void example_135() {
	//    VACANCY POINTS
	std::vector<Eigen::Vector2d> points = {
		{ 8, 10 },
		{ 10, 10 },
		{ 10, 8 },
		{ 8, 8 }
	};
	//    VACANCY EDGES
	std::vector<std::vector<int>> somelines = {
		{ 0, 1 },
		{ 1, 2 },
		{ 2, 3 },
		{3, 0}
	};
	//
	ZoningCommisioner wild;
	wild.insertCurves(points, somelines);
	wild.populateNeighbors();
	wild.constructZoneCovering();
	//
	svgvis::ZoningCartographerSVG wsvg;
	wsvg.addCardinalCurveCollection(wild.vacant_);
	std::vector<Eigen::Vector2d> whs = { {2,2} };
	for (auto wh : whs) {
		double width = wh[0];
		double height = wh[1];
		Eigen::Vector2d placement;
		bool is_placable = wild.findPlacement(width, height, placement);
		int trail_cnt = 0;
		for (auto &t : wild.zones_) {
			svgvis::ZoningCartographerSVG svger;
			svger.addCardinalCurveCollection(wild.vacant_, "black");
			svger.addCardinalPath(t.landmarks_uptown_, "blue");
			svger.addCardinalPath(t.landmarks_downtown_, "green");
			Surveyor caliper_hiker(t, width, height);
			caliper_hiker.hike();
			svger.addCardinalPath(caliper_hiker.camps_uptown_, "orange");
			svger.addCardinalPath(caliper_hiker.camps_downtown_, "red");
			svger.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example135_" + std::to_string(trail_cnt) + ".svg");
			trail_cnt++;
		}
		//for (auto &t : wild.trails_) {
  //          wsvg.addCardinalPath(t.landmarks_uptown_, "blue");
  //          wsvg.addCardinalPath(t.landmarks_downtown_, "green");
		//	CaliperHiker caliper_hiker(t, width, height);
		//	caliper_hiker.hike();
		//	wsvg.addCardinalPath(caliper_hiker.camps_uptown_, "black");
		//	wsvg.addCardinalPath(caliper_hiker.camps_downtown_, "red");
  //      }
		wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example135.svg");
		//removeRectangle(wild.trails_, placement, width, height);
		if (is_placable) {
			wild.zoneOff(placement, width, height);
			wsvg.addRectangle(placement, width, height);
			wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example135.svg");
		}
		else {
			wsvg.addRectangle({ 11,0 }, width, height);
			wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example135.svg");
		}
	}
	wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example135.svg");
}

void example_14() {
	std::vector<Eigen::Vector2d> points = { {0,0}, {1,0}, {1,-1}, {3.4,-1}, {3.4,-.6}, {5,-.6}, {5,2}, {0,2} };
    std::vector<std::vector<int>> somelines = { {0,1},{1,2},{2,3},{3,4},{4,5},{5,6},{6,7},{7,0} };
    //
    ZoningCommisioner wild;
    wild.insertCurves(points, somelines);
    wild.populateNeighbors();
    wild.constructZoneCovering();
    //
    svgvis::ZoningCartographerSVG wsvg;
    wsvg.addCardinalCurveCollection(wild.vacant_);
	std::vector<Eigen::Vector2d> whs = { {2.4,1}, {2.4,1}, {2.4,1}, {2.4,1}, {2.4,1}, {2.4,1} };
    for (auto wh : whs)  {
        double width = wh[0];
        double height = wh[1];
        Eigen::Vector2d placement;
        bool is_placable = wild.findPlacement(width, height, placement);
        for (auto &t : wild.zones_) {
            //wsvg.addCardinalPath(t.landmarks_uptown_, "blue");
            //wsvg.addCardinalPath(t.landmarks_downtown_, "green");
			Surveyor caliper_hiker(t, width, height);
			caliper_hiker.hike();
			wsvg.addCardinalPath(caliper_hiker.camps_downtown_, "red");
        }
        //removeRectangle(wild.trails_, placement, width, height);
        if (is_placable) {
            wild.zoneOff(placement, width, height);
            wsvg.addRectangle(placement, width, height);
            wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example14.svg");
        } else {
            wsvg.addRectangle({6,0}, width, height);
            wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example14.svg");
        }
    }
    wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example14.svg");
}
//{6.9, 4.4}, { 6.6,3.3 },

void example_15() {
	std::vector<Eigen::Vector2d> points = {
		{0,0},
		{10,0},
		{10,10},
		{0,10},
		{0,6},
		{6,6},
		{6,4},
		{0,4},
	};
	std::vector<std::vector<int>> somelines = {
		{0,1},
		{1,2},
		{2,3},
		{3,4},
		{4,5},
		{5,6},
		{6,7},
		{7,0},
	};
	//
	ZoningCommisioner wild;
	wild.insertCurves(points, somelines);
	wild.populateNeighbors();
	wild.constructZoneCovering();
	//
	svgvis::ZoningCartographerSVG wsvg;
	wsvg.addCardinalCurveCollection(wild.vacant_, "black");
	wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example15.svg");
	std::vector<Eigen::Vector2d> whs = { {2.4,1} };
	for (auto wh : whs) {
		double width = wh[0];
		double height = wh[1];
		Eigen::Vector2d placement;
		bool is_placable = wild.findPlacement(width, height, placement);
		for (auto &t : wild.zones_) {
			wsvg.addCardinalPath(t.landmarks_uptown_, "blue");
			wsvg.addCardinalPath(t.landmarks_downtown_, "green");
			Surveyor caliper_hiker(t, width, height);
			caliper_hiker.hike();
			wsvg.addCardinalPath(caliper_hiker.camps_downtown_, "red");
		}
		//removeRectangle(wild.trails_, placement, width, height);
		if (is_placable) {
			wild.zoneOff(placement, width, height);
			wsvg.addRectangle(placement, width, height);
			wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example15.svg");
		}
		else {
			wsvg.addRectangle({ 10,0 }, width, height);
			wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example15.svg");
		}
	}
	wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example15.svg");
}

void example_test_edge_intersect0(){
	Hedge a = { Eigen::Vector2d({0,0}), Eigen::Vector2d({4,0}) };
	Hedge b = { Eigen::Vector2d({0,0}), Eigen::Vector2d({4,0}) };
	Hedge isect;
	auto seg_rel = edgeIntersection(a,b,true,isect);
	std::cout << _debug_seg_rel_print(seg_rel) << " isect " << isect[0].transpose() << " -> " << isect[1].transpose() << std::endl;
};

void example_addRectTest0() {
	double width = 10;
	double height = 10;
	Eigen::Vector2d position = { 0,0 };
	std::vector<PointList> chains = {
		{{-1, 1}, {0, 1}, {0, 2}, {-1, 2}},
		{{-1, 3}, {0, 3}, {0, 4}, {-1, 4}},
		{{-1, 5}, {0, 5}, {0, 6}, {-1, 6}},
		{{1, 12}, {1, 10}, {2, 10}, {2, 11}, {3, 11}, {3, 10}, {4,10}, {4,12}},
		{{10,13},{10,-3}},
		{{3,-3},{3,0},{4,0},{4,-1},{5,-1},{5,0},{6,0},{6,-3}}
	};
	svgvis::ZoningCartographerSVG vv0;
	vv0.addRectangle(position, width, height);
	for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
	vv0.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_rect0_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	svgvis::ZoningCartographerSVG vv;
	vv.addRectangle(position, width, height);
	for (PointList &pl : pls) vv.addPointList(pl, svgvis::chaosHex());
	vv.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_rect0.svg");
}

void example_addRectTest1() {
	double width = 10;
	double height = 10;
	Eigen::Vector2d position = { 0,0 };
	std::vector<PointList> chains = {
		{{0, 12}, {0, 0}, {12,0}}
	};
	svgvis::ZoningCartographerSVG vv0;
	vv0.addRectangle(position, width, height);
	for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
	vv0.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_rect1_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	svgvis::ZoningCartographerSVG vv;
	vv.addRectangle(position, width, height);
	for (PointList &pl : pls) vv.addPointList(pl, svgvis::chaosHex());
	vv.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_rect1.svg");
}

void example_addRectTest2() {
	double width = 10;
	double height = 10;
	Eigen::Vector2d position = { 0,0 };
	std::vector<PointList> chains = {
		{{0, 12}, {0, 0}, {10,0}, {10,12}}
	};
	svgvis::ZoningCartographerSVG vv0;
	vv0.addRectangle(position, width, height);
	for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
	vv0.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_rect2_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	svgvis::ZoningCartographerSVG vv;
	vv.addRectangle(position, width, height);
	for (PointList &pl : pls) vv.addPointList(pl, svgvis::chaosHex());
	vv.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_rect2.svg");
}

void example_addRectTest3() {
	double width = 10;
	double height = 10;
	Eigen::Vector2d position = { 0,0 };
	std::vector<PointList> chains = {
		{{0, 12}, {0, -2}, {5,-2}, {5,0}, {12,0}}
	};
	svgvis::ZoningCartographerSVG vv0;
	vv0.addRectangle(position, width, height);
	for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
	vv0.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_rect3_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	svgvis::ZoningCartographerSVG vv;
	vv.addRectangle(position, width, height);
	for (PointList &pl : pls) vv.addPointList(pl, svgvis::chaosHex());
	vv.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_rect3.svg");
}

void example_addRectTest4() {
	double width = 10;
	double height = 10;
	Eigen::Vector2d position = { 0,0 };
	std::vector<PointList> chains = {
		{{0, 12}, {0, -2}, {7,-2}, {7,0}, {8,0}, {8, -2}, {9,-2}, {9,-4},}
	};
	svgvis::ZoningCartographerSVG vv0;
	vv0.addRectangle(position, width, height);
	for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
	vv0.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_rect4_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	svgvis::ZoningCartographerSVG vv;
	vv.addRectangle(position, width, height);
	for (PointList &pl : pls) vv.addPointList(pl, svgvis::chaosHex());
	vv.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_rect4.svg");
}

void example_addRectTest5() {
	double width = 10;
	double height = 10;
	Eigen::Vector2d position = { 0,0 };
	std::vector<PointList> chains = {
		{{0, 12}, {0, -2}, {1,-2}, {1,0}, {2,0}, {2, -2}, {3,-2}, {3,0}, {4,0}, {4, -3}, {5,-3}, {5,0}, {6,0}, {6, -2.6}, {7,-2.6}, {7,0}, {8,0}, {8, -2}, {9,-2}, {9,-4},}
	};
	svgvis::ZoningCartographerSVG vv0;
	vv0.addRectangle(position, width, height);
	for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
	vv0.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_rect5_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	svgvis::ZoningCartographerSVG vv;
	vv.addRectangle(position, width, height);
	for (PointList &pl : pls) vv.addPointList(pl, svgvis::chaosHex());
	vv.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_rect5.svg");
}

void example_addRectTest6() {
	double width = 10;
	double height = 10;
	Eigen::Vector2d position = { 0,0 };
	std::vector<PointList> chains = {
		{
			{-5,-3},
			{-5,-2},
			{-4,-2},
			{-4,-3},
			{-5,-3}
		},
		{
			{0, 12},
			{0, -2}, 
			{1,-2}, 
			{1,0}, 
			{2,0}, 
			{2, -2}, 
			{3,-2}, 
			{3,0}, 
			{4,0}, 
			{4, -3}, 
			{5,-3}, 
			{5,0}, 
			{6,0}, 
			{6, -2.6}, 
			{7,-2.6}, 
			{7,0}, 
			{8,0}, 
			{8, -2}, 
			{9,-2}, 
			{9,-4},
			{10,-4},
			{10,12},
			{0,12}
		}
	};
	svgvis::ZoningCartographerSVG vv0;
	vv0.addRectangle(position, width, height);
	for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
	vv0.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_rect6_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	svgvis::ZoningCartographerSVG vv;
	vv.addRectangle(position, width, height);
	for (PointList &pl : pls) vv.addPointList(pl, svgvis::chaosHex());
	vv.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_rect6.svg");
}

void example_addRectTest7() {
	double width = 5.4;
	double height = 2.1;
	Eigen::Vector2d position = { 0,0 };
	std::vector<PointList> chains = {
		{
			{0,0},
			{0,10},
			{10,10},
			{10,0},
			{0,0}
		}
	};
	svgvis::ZoningCartographerSVG vv0;
	vv0.addRectangle(position, width, height);
	for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
	vv0.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_rect7_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	svgvis::ZoningCartographerSVG vv;
	vv.addRectangle(position, width, height);
	for (PointList &pl : pls) vv.addPointList(pl, svgvis::chaosHex());
	vv.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_rect7.svg");
}

void example_addRectTest8() {
    double width = 5;
    double height = 5;
    Eigen::Vector2d position = {5,0};
    std::vector<PointList> chains = {
        {
            {5,0},
            {10,0},
            {10,10},
            {0,10},
            {0,5},
            {5,5},
            {5,0}
        }
    };
    svgvis::ZoningCartographerSVG vv0;
    vv0.addRectangle(position, width, height);
    for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
    vv0.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_rect8_before.svg");

    std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
    svgvis::ZoningCartographerSVG vv;
    vv.addRectangle(position, width, height);
    for (PointList &pl : pls) vv.addPointList(pl, svgvis::chaosHex());
    vv.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_rect8.svg");
}

void example_curve_update() {
	std::vector<Eigen::Vector2d> points = { {0,0},{10,0},{10,10},{0,10} };
	std::vector<std::vector<int>> somelines = { {0,1},{1,2},{2,3},{3,0} };
	CardinalCurveCollection ccc;
	ccc.setGridPointsAndCells(points, somelines);
	svgvis::ZoningCartographerSVG vv;
	auto pls = ccc.getPointCycles();
	for (PointList &pl : pls) vv.addPointList(pl, svgvis::chaosHex());
	vv.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_curve_update.svg");
}

void example_why_is_trail_fail() {
	std::vector<Eigen::Vector2d> points = { 
		{5.4, 9.6},
		{5.4, 9.8},
		{0, 9.8},
		{ 0, 10},
		{10, 10},
		{10,  0},
		{6.9,   0},
		{6.9, 4.4},
		{6.6, 4.4},
		{6.6, 7.7},
		{8.9, 7.7},
		{8.9, 9.6}
	};
	std::vector<std::vector<int>> somelines = { {0,1},{1,2},{2,3},{3,4},{4,5},{5,6},{6,7},{7,8},{8,9},{9,10},{10,11},{11,0}};
	std::vector<double> wh = { 3.5, 1.9 };
	double width = wh[0];
	double height = wh[1];
	Eigen::Vector2d placement;
	ZoningCommisioner wild;
	wild.insertCurves(points, somelines);
	wild.populateNeighbors();
	wild.constructZoneCovering();
	svgvis::ZoningCartographerSVG wsvg;
	wsvg.addCardinalCurveCollection(wild.vacant_, "yellow");
	for (auto &t : wild.zones_) {
		wsvg.addCardinalPath(t.landmarks_uptown_, "blue");
		wsvg.addCardinalPath(t.landmarks_downtown_, "green");
	}
	wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/badtrail.svg");
	bool is_placable = wild.findPlacement(width, height, placement);
	if (is_placable) {
		std::cout << "PLACE AT: " << placement.transpose() << std::endl;
		wild.zoneOff(placement, width, height);
		wsvg.addPointList(wild.vacant_.points_);
		//wsvg.addRectangle(placement, width, height);
		wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/badtrail.svg");
	}
	else {
		wsvg.addRectangle({ 11,0 }, width, height);
	}
	wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/badtrail.svg");
}

#include "zoning_board.h"

void example_zoning_board1() {
	ZoningBoard zb;
	PointList ps = { {0,0},{10,0},{10,10},{0,10} };
	std::vector<std::vector<int>> es = { {0,1},{1,2},{2,3},{3,0} };
	zb.annexVacancy(ps, es);
	std::vector<Eigen::Vector2d> rectangles = {
		{1,0.5},{1,0.5},{1,0.5},{1,0.5},{1,0.5},{1,0.5},{6,6},{7,6},{8,6},{9,6}, {4,4},{1,1},{1,1},{1,1},{1,1},{1,1},{3,2},{3,3},{1,8},{2,2},{4,4},{5,1},{4,2},{2,2},{9,2},{5,5},{3,1}
	};
	zb.setApplicantRectangles(rectangles);
	zb.zone();
	svgvis::ZoningCartographerSVG zsvg;
	zsvg.addZoningBoardReport(zb);
	zsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/zb1.svg");
};

void example_zoning_board2() {
	ZoningBoard zb;
	
	PointList ps = { {0,0},{10,0},{10,10},{0,10} };
	std::vector<std::vector<int>> es = { {0,1},{1,2},{2,3},{3,0} };
	zb.annexVacancy(ps, es);
	
	PointList ps2 = {{0,0}, {0,-2}, {6,-2}, {6,0}, {8,0}, {8,6}, {6,6}, {6,8}, {0,8}, {0,6}, {-2,6}, {-2,0}};
	zb.annexVacancy(ps2);

	PointList ps3 = { {0,0},{10,0},{10,5},{5,5},{5,10},{0,10} };
	zb.annexVacancy(ps3);
	
	PointList ps4 = { {0,0},{10,0},{10,10},{6,10},{6,4},{4,4},{4,10},{0,10} };
	zb.annexVacancy(ps4);

	std::vector<Eigen::Vector2d> rectangles = {
		{10,1.4},{10,1.4}, {0.3,0.8},{0.3,0.8},{0.3,4.8}, {1,0.5},{1,0.5},{1,0.5},{1,0.5},{0.3,0.8},{0.3,0.8},{0.3,4.8}, {1,0.5},{1,0.5},{1,0.5},{1,0.5},{1,0.5},{1,0.5},{6,6},{7,6},{8,6},{9,6}, {4,4},{1,1},{1,1},{1,1},{1,1},{1,1},{3,2},{3,3},{1,8},{2,2},{4,4},{5,1},{4,2},{2,2},{9,2},{5,5},{3,1}
	};

	zb.setApplicantRectangles(rectangles);
	zb.zone();

	svgvis::ZoningCartographerSVG zsvg;
	zsvg.addZoningBoardReport(zb);
	zsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/zb2.svg");
};

void example_zoning_board3() {
	ZoningBoard zb;

	PointList ps = { {0,0},{10,0},{10,10},{0,10} };
	std::vector<std::vector<int>> es = { {0,1},{1,2},{2,3},{3,0} };
	zb.annexVacancy(ps, es);

	PointList ps2 = { {0,0}, {0,-2}, {6,-2}, {6,0}, {8,0}, {8,6}, {6,6}, {6,8}, {0,8}, {0,6}, {-2,6}, {-2,0} };
	zb.annexVacancy(ps2);

	PointList ps3 = { {0,0},{10,0},{10,5},{5,5},{5,10},{0,10} };
	zb.annexVacancy(ps3);

	PointList ps4 = { {0,0},{10,0},{10,10},{6,10},{6,4},{4,4},{4,10},{0,10} };
	zb.annexVacancy(ps4);

	std::vector<Eigen::Vector2d> rectangles = {
		{2,2},{3,3},{4,4},{1,5},{3.8,4.2},{1.3,1.4}
	};

	std::vector<int> multiplicities = {
		5,10,15,7,9,6
	};

	zb.setApplicantRectangles(rectangles, multiplicities);
	zb.zone();

	svgvis::ZoningCartographerSVG zsvg;
	zsvg.addZoningBoardReport(zb);
	zsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/zb3.svg");
};

void example_zoning_board4() {
	//!!!! Can we deal with a non-simply connected example?
	// This non-simply connected vacancy will really screw the packer up.
	// Probably requires a good chunk of work to test for.
	// Need to prevent infinite loops though.
	ZoningBoard zb;

	PointList ps = { {0,0},{10,0},{10,10},{0,10}, {4,4},{6,4},{6,6},{4,6}};
	std::vector<std::vector<int>> es = { {0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4} };
	zb.annexVacancy(ps, es);

	std::vector<Eigen::Vector2d> rectangles = {
		{5,5}
	};

	std::vector<int> multiplicities = {
		5
	};

	zb.setApplicantRectangles(rectangles, multiplicities);
	zb.zone();

	svgvis::ZoningCartographerSVG zsvg;
	zsvg.addZoningBoardReport(zb);
	zsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/zb4.svg");
};

void example_zoning_board5() {
	// Example with a set number of rectangles to pack and an arbitrary number of vacancy copies
	ZoningBoard zb;

	PointList ps = { {0,0},{10,0},{10,10},{0,10}};
	std::vector<std::vector<int>> es = { {0,1},{1,2},{2,3},{3,0}};
	
	zb.annexVacancy(ps, es, -1);

	std::vector<Eigen::Vector2d> rectangles = {
		{5,5}
	};

	std::vector<int> multiplicities = {
		21
	};

	zb.setApplicantRectangles(rectangles, multiplicities);
	zb.zone();

	svgvis::ZoningCartographerSVG zsvg;
	zsvg.addZoningBoardReport(zb);
	zsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/zb5.svg");
};

void example_zoning_board6() {
	// Example with a set number of rectangles to pack and an arbitrary number of vacancy copies
	ZoningBoard zb;

	PointList ps = { {0,0},{10,0},{10,10},{0,10} };
	std::vector<std::vector<int>> es = { {0,1},{1,2},{2,3},{3,0} };
	zb.annexVacancy(ps, es, 3);

	std::vector<Eigen::Vector2d> rectangles = {
		{5,5}
	};

	std::vector<int> multiplicities = {
		21
	};

	zb.setApplicantRectangles(rectangles, multiplicities);
	zb.zone();

	svgvis::ZoningCartographerSVG zsvg;
	zsvg.addZoningBoardReport(zb);
	zsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/zb6.svg");
};

#include "vtkSTLReader.h"
#include "vtkXMLPolyDataWriter.h"
#include "vtkXMLUnstructuredGridWriter.h"

#include "boost/filesystem/path.hpp"

#include "planar_bounding_box.h"

void _debug_write_polydata( vtkSmartPointer<vtkPolyData> mesh, boost::filesystem::path path){
    auto writer1 = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
    writer1->SetInputData(mesh);
    writer1->SetFileName(path.string().c_str());
    writer1->Write();
}

void example_bound_box(){

    //#READ#
    boost::filesystem::path in_path = path_prefix + "/Users/sscott/Programs/ack/trex_connected.stl";
    auto reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(in_path.string().c_str());
    reader->Update();
    auto in_poly = reader->GetOutput();
    //#READ#

    
    PlanarBoxBounder pbb;
    pbb.surface_ = in_poly;
    pbb.projection_normal_ = {1, 1, 1};
    pbb.projection_normal_.normalize();
    pbb.projectToHull();
    
    _debug_write_polydata(pbb.hull_, path_prefix + "/Users/sscott/Programs/ack/trex_hull.vtp");

    auto recthull = makePolydataRectangle(pbb.corner_, pbb.box_x_dir_, pbb.box_y_dir_);
    _debug_write_polydata(recthull, path_prefix + "/Users/sscott/Programs/ack/trex_hull_rect.vtp");

    
    auto surf2 = pbb.getTransformedSurface();
    _debug_write_polydata(surf2, path_prefix + "/Users/sscott/Programs/ack/trex_transformed.vtp");
    
    auto rect = makePolydataRectangle({0,0,0}, {pbb.width,0,0}, {0,pbb.height,0});
    _debug_write_polydata(rect, path_prefix + "/Users/sscott/Programs/ack/trex_target_rect.vtp");
}

void example_tpix() {
	CardinalCurveCollection ccc;
	std::vector<double> diff_seq = {
		1,-2,1,-1,1,-1,2,1,1,1,2,1,2,1,1,6,1,1,8,-1,1,-4,-5,-1,3,-1,-4,-2,2,-2,-1,1,-1,-3,-1,-1,-1,-1,-1,-1,2,-1,-3,1,-2,-1,-1,-1,-1,-1,1,-1,-2,4,-1,1,-1,1,-1,1,-1,1,-1,1,-1
	};
	ccc.generateFromDifferenceSequence(diff_seq);
	//for (auto & pt : ccc.points_) {
	//	pt = { -pt[0],pt[1] };
	//}
	//scale
	svgvis::ZoningCartographerSVG zsvg;
	zsvg.addCardinalCurveCollection(ccc);
	zsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/t_pix.svg");
}

void example_tpix_zoner() {
	CardinalCurveCollection ccc;
	std::vector<double> diff_seq = {
		1,-2,1,-1,1,-1,2,1,1,1,2,1,2,1,1,6,1,1,8,-1,1,-4,-5,-1,3,-1,-4,-2,2,-2,-1,1,-1,-3,-1,-1,-1,-1,-1,-1,2,-1,-3,1,-2,-1,-1,-1,-1,-1,1,-1,-2,4,-1,1,-1,1,-1,1,-1,1,-1,1,-1
	};
	double scale = 25.0;
	ccc.generateFromDifferenceSequence(diff_seq, scale);
	for (auto & pt : ccc.points_) {
		pt = { -pt[0],pt[1] };
	}
	//
	svgvis::ZoningCartographerSVG zsvg2;
	zsvg2.addCardinalCurveCollection(ccc);
	zsvg2.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/t_pix.svg");
	//
	std::vector<std::vector<int>> lines;
	for (const auto edge : ccc.edges_) {
		std::vector<int> line(edge.begin(), edge.end());
		lines.push_back(line);
	}
	//
	ZoningCommisioner wild;
	wild.insertCurves(ccc.points_, lines);
	wild.populateNeighbors();
	wild.constructZoneCovering();
	int trail_cnt = 0;
	//
	svgvis::ZoningCartographerSVG wsvg;
	wsvg.default_stroke_width_ = 3;
	wsvg.addCardinalCurveCollection(wild.vacant_);
	double width = 40.5;
	double height = 23.6;
	std::vector<Eigen::Vector2d> whs;
	int copies = 50;
	for (auto cop = 0; cop < copies; cop++) whs.push_back({width, height});
	int cop_cnt = 0;
	for (auto wh : whs) {
		std::cout << "working copy " << cop_cnt << std::endl;
		cop_cnt++;
		double width = wh[0];
		double height = wh[1];
		Eigen::Vector2d placement;
		bool is_placable = wild.findPlacement(width, height, placement);
		int trail_cnt = 0;
		for (auto &t : wild.zones_) {
			svgvis::ZoningCartographerSVG svger;
			svger.default_stroke_width_ = 3;
			svger.addCardinalCurveCollection(wild.vacant_, "black");
			svger.addCardinalPath(t.landmarks_uptown_, "blue");
			svger.addCardinalPath(t.landmarks_downtown_, "green");
			Surveyor caliper_hiker(t, width, height);
			caliper_hiker.hike();
			svger.addCardinalPath(caliper_hiker.camps_uptown_, "orange");
			svger.addCardinalPath(caliper_hiker.camps_downtown_, "red");
			svger.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_tpix_zone" + std::to_string(trail_cnt) + ".svg");
			trail_cnt++;
		}
		//for (auto &t : wild.trails_) {
  //          wsvg.addCardinalPath(t.landmarks_uptown_, "blue");
  //          wsvg.addCardinalPath(t.landmarks_downtown_, "green");
		//	CaliperHiker caliper_hiker(t, width, height);
		//	caliper_hiker.hike();
		//	wsvg.addCardinalPath(caliper_hiker.camps_uptown_, "black");
		//	wsvg.addCardinalPath(caliper_hiker.camps_downtown_, "red");
  //      }
		wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_tpix_zone.svg");
		//removeRectangle(wild.trails_, placement, width, height);
		if (is_placable) {
			wild.zoneOff(placement, width, height);
			wsvg.addRectangle(placement, width, height);
			wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_tpix_zone.svg");
		}
		else {
			wsvg.addRectangle({ 11,0 }, width, height);
			wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_tpix_zone.svg");
		}
	}
	wsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/example_tpix_zone.svg");
}

void example_pack_tpix() {
	//Setup the vacancy
	CardinalCurveCollection ccc;
	std::vector<double> diff_seq = {
		1,-2,1,-1,1,-1,2,1,1,1,2,1,2,1,1,6,1,1,8,-1,1,-4,-5,-1,3,-1,-4,-2,2,-2,-1,1,-1,-3,-1,-1,-1,-1,-1,-1,2,-1,-3,1,-2,-1,-1,-1,-1,-1,1,-1,-2,4,-1,1,-1,1,-1,1,-1,1,-1,1,-1
	};
	double scale = 25.0;
	ccc.generateFromDifferenceSequence(diff_seq, scale);
	for (auto & pt : ccc.points_) {
		pt = { -pt[0],pt[1] };
	}
	//

	ZoningBoard zb;
	zb.annexVacancy(ccc.points_);

	zb.setApplicantRectangles({ {40.5, 23.6} }, { 50 });
	zb.zone();

	svgvis::ZoningCartographerSVG zsvg;
	zsvg.addZoningBoardReport(zb);
	zsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/pack_tpix.svg");

};

void example_box_of_rex() {
	//Setup the vacancy

	CardinalCurveCollection ccc;
	std::vector<double> diff_seq = {
		1,-2,1,-1,1,-1,2,1,1,1,2,1,2,1,1,6,1,1,8,-1,1,-4,-5,-1,3,-1,-4,-2,2,-2,-1,1,-1,-3,-1,-1,-1,-1,-1,-1,2,-1,-3,1,-2,-1,-1,-1,-1,-1,1,-1,-2,4,-1,1,-1,1,-1,1,-1,1,-1,1,-1
	};
	double scale = 25.0;
	ccc.generateFromDifferenceSequence(diff_seq, scale);
	for (auto & pt : ccc.points_) {
		pt = { -pt[0],pt[1] };
	}
	//

	ZoningBoard zb;
	zb.allow_rotations = true;

	PointList ps = { {0,0},{100,0},{100,110},{0,110} };
	std::vector<std::vector<int>> es = { {0,1},{1,2},{2,3},{3,0} };
	zb.annexVacancy(ps, es);

	//#READ#
	boost::filesystem::path in_path = path_prefix + "/Users/sscott/Programs/ack/trex_connected.stl";
	auto reader = vtkSmartPointer<vtkSTLReader>::New();
	reader->SetFileName(in_path.string().c_str());
	reader->Update();
	auto in_poly = reader->GetOutput();
	//#READ#

	PlanarBoxBounder pbb;
	pbb.surface_ = in_poly;
	pbb.projection_normal_ = { 1,-1, 0 };
	pbb.projection_normal_.normalize();
	pbb.projectToHull();
	pbb.width, pbb.height;

	zb.setApplicantRectangles({ {pbb.width, pbb.height} }, { 20 });
	zb.zone();


	svgvis::ZoningCartographerSVG zsvg;
	zsvg.addZoningBoardReport(zb);
	zsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/box_of_rex.svg");
}

#include "packmesh_manager.h"

void example_rexbox() {
	//#READ#
	boost::filesystem::path in_path = path_prefix + "/Users/sscott/Programs/ack/trex_connected.stl";
	auto reader = vtkSmartPointer<vtkSTLReader>::New();
	reader->SetFileName(in_path.string().c_str());
	reader->Update();
	auto in_poly = reader->GetOutput();
	//#READ#

	//Setup the vacancy
	PackMeshManager pmm;
	pmm.insertMesh(in_poly, { 1,-1, 0}, 4);
	pmm.insertMesh(in_poly, { 1, 4, 0 }, 4);
	pmm.insertMesh(in_poly, { 2, 1, 0 }, 4);
	pmm.insertMesh(in_poly, { 1, 0, 1 }, 4);

	PointList ps = { {0,0},{150,0},{150,150},{0,150} };
	pmm.insertPackspace(ps);

	pmm.pack();
	vtkSmartPointer<vtkPolyData> packed_space = pmm.getPackMesh();
	svgvis::ZoningCartographerSVG svgw;
	svgw.addZoningBoardReport(pmm.zoning_board_);
	svgw.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/rexbox.svg");

	auto writ = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
	boost::filesystem::path  outpath = (path_prefix + "/Users/sscott/Programs/ack/rexbox.vtp");
	writ->SetInputData(packed_space);
	writ->SetFileName(outpath.string().c_str());
	writ->Write();
}

void example_rex_of_rex() {
	//Setup the vacancy
	CardinalCurveCollection ccc;
	std::vector<double> diff_seq = {
		1,-2,1,-1,1,-1,2,1,1,1,2,1,2,1,1,6,1,1,8,-1,1,-4,-5,-1,3,-1,-4,-2,2,-2,-1,1,-1,-3,-1,-1,-1,-1,-1,-1,2,-1,-3,1,-2,-1,-1,-1,-1,-1,1,-1,-2,4,-1,1,-1,1,-1,1,-1,1,-1,1,-1
	};
	double scale = 25.0;
	ccc.generateFromDifferenceSequence(diff_seq, scale);
	for (auto & pt : ccc.points_) {
		pt = { -pt[0],pt[1] };
	}
	//

	ZoningBoard zb;
	zb.annexVacancy(ccc.points_);

	//#READ#
	boost::filesystem::path in_path = path_prefix + "/Users/sscott/Programs/ack/trex_connected.stl";
	auto reader = vtkSmartPointer<vtkSTLReader>::New();
	reader->SetFileName(in_path.string().c_str());
	reader->Update();
	auto in_poly = reader->GetOutput();
	//#READ#

	PlanarBoxBounder pbb;
	pbb.surface_ = in_poly;
	pbb.projection_normal_ = { 1, 1, 1 };
	pbb.projection_normal_.normalize();
	pbb.projectToHull();
	pbb.width, pbb.height;

	zb.setApplicantRectangles({ {pbb.width, pbb.height} }, { 50 });
	zb.zone();


	svgvis::ZoningCartographerSVG zsvg;
	zsvg.addZoningBoardReport(zb);
	zsvg.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/rex_of_rex.svg");
}

void example_nitpick() {
	//#READ#
	boost::filesystem::path in_path = path_prefix + "/Users/sscott/Programs/ack/fuca_real.stl";
	auto reader = vtkSmartPointer<vtkSTLReader>::New();
	reader->SetFileName(in_path.string().c_str());
	reader->Update();
	auto in_poly = reader->GetOutput();
	//#READ#

	//Setup the vacancy
	PackMeshManager pmm;
	pmm.insertMesh(in_poly, { 0, 1, 0 }, 1);
	pmm.insertMesh(in_poly, { 0, -1, 0 }, 1);

	PointList ps = { {0,0},{1500,0},{1500,1500},{0,1500} };
	pmm.insertPackspace(ps);

	pmm.pack();
	vtkSmartPointer<vtkPolyData> packed_space = pmm.getPackMesh();
	svgvis::ZoningCartographerSVG svgw;
	svgw.addZoningBoardReport(pmm.zoning_board_);
	svgw.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/nitpick.svg");

	auto writ = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
	boost::filesystem::path  outpath = (path_prefix + "/Users/sscott/Programs/ack/nitpick.vtp");
	writ->SetInputData(packed_space);
	writ->SetFileName(outpath.string().c_str());
	writ->Write();
}

void example_multimodel() {
	//#READ#
	boost::filesystem::path in_path = path_prefix + "/Users/sscott/Programs/ack/trex_connected.stl";
	auto reader = vtkSmartPointer<vtkSTLReader>::New();
	reader->SetFileName(in_path.string().c_str());
	reader->Update();
	auto rex = reader->GetOutput();
	//#READ#

	//#READ#
	boost::filesystem::path in_path1 = path_prefix + "/Users/sscott/Programs/ack/fuca_real.stl";
	auto reader1 = vtkSmartPointer<vtkSTLReader>::New();
	reader1->SetFileName(in_path1.string().c_str());
	reader1->Update();
	auto fuca_real = reader1->GetOutput();
	//#READ#

	//#READ#
	boost::filesystem::path in_path2 = path_prefix + "/Users/sscott/Programs/ack/fuca_wrapped.stl";
	auto reader2 = vtkSmartPointer<vtkSTLReader>::New();
	reader2->SetFileName(in_path2.string().c_str());
	reader2->Update();
	auto fuca_wrap = reader2->GetOutput();
	//#READ#

	//Setup the vacancy
	PackMeshManager pmm;
	pmm.insertMesh(rex, { 1, 4, 0 }, 5);
	pmm.insertMesh(fuca_real, { 0, -2, -1 }, 3);
	pmm.insertMesh(fuca_wrap, { 0, -2, -1 }, 3);
	pmm.insertMesh(fuca_real, { 1, 1, 1 }, 3);
	pmm.insertMesh(fuca_wrap, { 1, 1, 1 }, 3);

	PointList ps = { {0,0},{1000,0},{1000,1000},{0,1000} };
	pmm.insertPackspace(ps);

	pmm.pack();
	vtkSmartPointer<vtkPolyData> packed_space = pmm.getPackMesh();
	svgvis::ZoningCartographerSVG svgw;
	svgw.addZoningBoardReport(pmm.zoning_board_);
	svgw.writeScalableVectorGraphics(path_prefix + "/Users/sscott/Programs/ack/multimodel.svg");

	auto writ = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
	boost::filesystem::path  outpath = (path_prefix + "/Users/sscott/Programs/ack/multimodel.vtp");
	writ->SetInputData(packed_space);
	writ->SetFileName(outpath.string().c_str());
	writ->Write();
}

#include <chrono>
#include <random>

void example_basic_zone_timed(std::vector<Eigen::Vector2d>& rects, double sqsize, boost::filesystem::path svgpath = "") {
	if (svgpath.empty()) {
		svgpath = path_prefix + "/Users/sscott/Programs/ack/rand_rects_"+std::to_string(rects.size())+".svg";
	}

	auto start = std::chrono::high_resolution_clock::now();

	ZoningBoard zb;
	PointList ps = { {0,0},{sqsize,0},{sqsize,sqsize},{0,sqsize} };
	std::vector<std::vector<int>> es = { {0,1},{1,2},{2,3},{3,0} };
	zb.annexVacancy(ps, es);
	zb.setApplicantRectangles(rects);
	zb.zone();

	// Get ending timepoint 
	auto stop = std::chrono::high_resolution_clock::now();

	// Get duration. Substart timepoints to  
	// get durarion. To cast it to proper unit 
	// use duration cast method 
	auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
	std::cout << "trial, time: " << rects.size() << ", " << duration.count() << std::endl;

	if (true) {
		svgvis::ZoningCartographerSVG zsvg;
		zsvg.default_stroke_width_ = sqsize / 200.0;
		zsvg.addZoningBoardReport(zb);
		zsvg.writeScalableVectorGraphics(svgpath.string());
	}
}
 
void example_zoningboard_randrects(int nsamples) {
	std::default_random_engine generator;
	std::exponential_distribution<double> distribution(1);
	std::vector<Eigen::Vector2d> rects(nsamples);
	double totarea = 0; 
	for (auto foo = 0; foo < rects.size(); foo++) {
		double a0 = distribution(generator);
		double b0 = distribution(generator);
		rects[foo] = { a0,b0 };
		totarea += a0 * b0;
	}
	double packspace_side = std::sqrt(totarea);
	example_basic_zone_timed(rects, packspace_side);
}


void example_high_mult_ruca(int multiplicity) {
	//#READ#
	boost::filesystem::path in_path1 = path_prefix + "/Users/sscott/Programs/ack/fuca_real.stl";
	auto reader1 = vtkSmartPointer<vtkSTLReader>::New();
	reader1->SetFileName(in_path1.string().c_str());
	reader1->Update();
	auto fuca_real = reader1->GetOutput();
	//#READ#
		//Setup the vacancy
	PackMeshManager pmm;
	pmm.insertMesh(fuca_real, { 0, -2, -1 }, multiplicity);
	auto rec = pmm.rectangles[0];
	double packspace_side = 1.01 * std::sqrt(rec[0] * rec[1] * multiplicity);
	PointList ps = { {0,0},{packspace_side,0},{packspace_side,packspace_side},{0,packspace_side} };
	pmm.insertPackspace(ps);
	pmm.pack();

	if (true) {
		svgvis::ZoningCartographerSVG zsvg;
		zsvg.default_stroke_width_ = packspace_side / 200.0;
		zsvg.addZoningBoardReport(pmm.zoning_board_);
		boost::filesystem::path svgpath = path_prefix + "/Users/sscott/Programs/ack/ruca_multiplicity_" + std::to_string(multiplicity) + ".svg";
		zsvg.writeScalableVectorGraphics(svgpath.string());
	}
}

void example_ruca_rand_z(int sample_count) {
    //
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0,1.0);
    std::vector<Eigen::Vector3d> zdirs(sample_count);
    //
    const double CONSTANT_TAO = 6.28318530718;
    for (auto foo = 0; foo < zdirs.size(); foo++) {
        double theta = CONSTANT_TAO * distribution(generator);
        double phi = acos(1 - 2 * distribution(generator));
        double x = sin(phi) * cos(theta);
        double y = sin(phi) * sin(theta);
        double z = cos(phi);
        zdirs[foo] = {x,y,z};
    }

    double sqsize = 300 * std::sqrt(sample_count);
    PackMeshManager pmm;
    PointList ps = {{0,0},{sqsize,0},{sqsize,sqsize},{0,sqsize}};
    std::vector<std::vector<int>> es = { {0,1},{1,2},{2,3},{3,0} };
    pmm.insertPackspace(ps);

    
    //#READ#
    boost::filesystem::path in_path1 = path_prefix + "/Users/sscott/Programs/ack/fuca_real.stl";
    auto reader1 = vtkSmartPointer<vtkSTLReader>::New();
    reader1->SetFileName(in_path1.string().c_str());
    reader1->Update();
    auto fuca_real = reader1->GetOutput();
    //#READ#
        //Setup the vacancy
    for (auto foo=0; foo<sample_count; foo++){
        pmm.insertMesh(fuca_real, zdirs[foo]);
    }
    
    pmm.pack();
    
    vtkSmartPointer<vtkPolyData> packed_space = pmm.getPackMesh();
    auto writ = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
    boost::filesystem::path  outpath = (path_prefix +"/Users/sscott/Programs/ack/rand_dir" + std::to_string(sample_count) + ".vtp");
    writ->SetInputData(packed_space);
    writ->SetFileName(outpath.string().c_str());
    writ->Write();
    
    if (true) {
        svgvis::ZoningCartographerSVG zsvg;
        zsvg.default_stroke_width_ = sqsize / 200.0;
        zsvg.addZoningBoardReport(pmm.zoning_board_);
        boost::filesystem::path svgpath = path_prefix + "/Users/sscott/Programs/ack/rand_dir" + std::to_string(sample_count) + ".svg";
        zsvg.writeScalableVectorGraphics(svgpath.string());
    }

}

void example_benchmark() {
	std::vector<int> trials = { 10,100,1000,10000,1000000,10000000 };
	for (auto x : trials) {

		// Get starting timepoint 
		auto start = std::chrono::high_resolution_clock::now();

		// Call the function, here sort() 
		example_high_mult_ruca(x);

		// Get ending timepoint 
		auto stop = std::chrono::high_resolution_clock::now();

		// Get duration. Substart timepoints to  
		// get durarion. To cast it to proper unit 
		// use duration cast method 
		auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
		std::cout << "trial, time: " << x << ", " << duration.count() << std::endl;
	}
}

void example_slm500() {
    //
    PackMeshManager pmm;
    PointList ps = {
        {-230,-120},
        {-230,-140},
        {230,-140},
        {230,-120},
        {250,-120},
        {250,120},
        {230,120},
        {230,140},
        {-230,140},
        {-230,120},
        {-250,120},
        {-250,-120},
    };
    PointList ps2;
    for (auto p : ps) ps2.push_back(2*p);
    pmm.insertPackspace(ps2);

    
    //#READ#
    boost::filesystem::path in_path1 = path_prefix + "/Users/sscott/Programs/ack/fuca_real.stl";
    auto reader1 = vtkSmartPointer<vtkSTLReader>::New();
    reader1->SetFileName(in_path1.string().c_str());
    reader1->Update();
    auto fuca_real = reader1->GetOutput();
    //#READ#
        //Setup the vacancy
    pmm.insertMesh(fuca_real, {0,-1,0}, 20);
    pmm.pack();
    
    vtkSmartPointer<vtkPolyData> packed_space = pmm.getPackMesh();
    auto writ = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
    boost::filesystem::path  outpath = (path_prefix +"/Users/sscott/Programs/ack/slm500.vtp");
    writ->SetInputData(packed_space);
    writ->SetFileName(outpath.string().c_str());
    writ->Write();
    
    if (true) {
        svgvis::ZoningCartographerSVG zsvg;
        zsvg.default_stroke_width_ = 2;
        zsvg.addZoningBoardReport(pmm.zoning_board_);
        boost::filesystem::path svgpath = path_prefix + "/Users/sscott/Programs/ack/slm500.svg";
        zsvg.writeScalableVectorGraphics(svgpath.string());
    }

}

int main() {
    std::cout << "Saluton Mundo!" << std::endl;
//    example1();
//    example2();
//    example4();
//    example5();
//    example7();
//    example_9();
    example_10();
    example_11();
    example_12();
    example_125();
    example_13();
    example_135();
    example_15();
    example_tpix_zoner();
    example_nitpick();
    example_why_is_trail_fail();
    example_zoning_board1();
    example_zoning_board2();
    example_zoning_board3();
    example_zoning_board4();
    example_zoning_board5();


	//boost::filesystem::path in_path = path_prefix + "/Users/sscott/Pictures/trex_connected.stl";
//	example_zoningboard_randrects(150);
//    example_slm500();
}




