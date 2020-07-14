#include <iostream>
#include <limits>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "wilderness_cartographer_svg.h"

/*
 A vacancy is a polygon (or possible a non-simply connected region with polygon boundaries) that has only vertical and horizontal edges.
 
 Setting up the vacancy:
    From polygons:
    Compute the boundary connected components and compute an orientation
    Join any colinear segments into larger segments.
 
 Divide the vacancy into patrols:
    Patrol shows the top and bottom points
 
 
 For each rectangle:
     Given a rectangle size width by height:
     
     Generate a caliper with the width:
        Compute all caliper positions across the patrol top and bottom
        Compute the heights available at all positions
        Get a list of all rectangle spots available
     
     Pick the lowest, then leftest one.
     
     Go back and update the patrols.
 */


namespace exda{
    std::vector<std::array<double,3>> grid_points = {
        {0, 0, 0},
        {5, 0, 0},
        {5, 1, 0},
        {6, 1, 0},
        {6, 2, 0},
        {5, 2, 0},
        {5, 3, 0},
        {1, 3, 0},
        {1, 2, 0},
        {2, 2, 0},
        {2, 1, 0},
        {0, 1, 0},
        {3, 1, 0},
        {4, 1, 0},
        {4, 2, 0},
        {3, 2, 0},
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

    std::vector<std::array<double,3>> splotchpoints = {
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
    Trail patty11 = vac.trailblaze(11);
    Trail patty7 = vac.trailblaze(7);
};

void example4(){
    ZoningCommisioner vac;
    vac.insertCurves(exda::splotchpoints, exda::splotchlines);
    vac.populateNeighbors();
    Trail peopleheadeast = vac.trailblaze(15);
    std::cout << "Valley landmarks: " << std::endl;
    for (auto p : peopleheadeast.landmarks_valley_.points_){
        std::cout << p.transpose() << std::endl;
    }
    std::cout << "Mountain landmarks: " << std::endl;
    for (auto p : peopleheadeast.landmarks_mountain_.points_){
        std::cout << p.transpose() << std::endl;
    }
    PlankHiker calj(peopleheadeast.landmarks_valley_, 0.5, true);//3.3, false);
    calj.hike();
//    calj.hikeTrailValley();
    std::cout << "Valley camps:" << std::endl;
    for (auto p : calj.camps_.points_){
        std::cout << p.transpose() << std::endl;
    };
    /*
     Valley camps:
     0    0
     1.7   0
     1.7   3
     6 3
     6 1
     */
    PlankHiker calk(peopleheadeast.landmarks_mountain_, 0.5, false);//3.3, false);
    calk.hike();
//    calj.hikeTrailValley();
    std::cout << "Mountain camps:" << std::endl;
    for (auto p : calk.camps_.points_){
        std::cout << p.transpose() << std::endl;
    };
    /*
     Mountain camps:
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
    Trail peopleheadeast = vac.trailblaze(15);
    CaliperHiker calh(peopleheadeast,width, height);
    calh.hike();
    WildernessCartographerSVG vv;
    std::vector<std::string> funcolors = {"#F75C03", "#D90368", "#820263", "#291720", "#04A777"};
    vv.addCardinalCurveCollection(vac.vacant_, funcolors[3]);
    vv.addCardinalPath(calh.camps_valley_, funcolors[0]);
    vv.addCardinalPath(calh.camps_mountain_,  funcolors[1]);
    vv.addRectangles(calh.valid_, width, height, funcolors[4]);
    vv.writeScalableVectorGraphics(afile);
    std::cout << "Valids: " << std::endl;
    for (auto p : calh.valid_){
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
    std::vector<std::array<double,3>> notchy_points_arraysstyle;
    for (auto foo : notchy_points){
        notchy_points_arraysstyle.push_back({foo[0],foo[1],0});
    }
    vac.insertCurves(notchy_points_arraysstyle, notchy_lines);
    vac.populateNeighbors();
    vac.trailblaze();
    CaliperHiker calh(vac.trails_[0], width, height);
    calh.hike();
    WildernessCartographerSVG vv;
    std::vector<std::string> funcolors = {"#F75C03", "#D90368", "#820263", "#291720", "#04A777"};
    
    vv.addCardinalCurveCollection(vac.vacant_, funcolors[3]);
    vv.addCardinalPath(calh.camps_valley_, funcolors[0]);
    vv.addCardinalPath(calh.camps_mountain_,  funcolors[1]);
    vv.addRectangles(calh.valid_, width, height, funcolors[4]);
    vv.writeScalableVectorGraphics("/Users/sscott/Programs/ack/notchy.svg");
    
};



void example_basic(std::vector<std::array<double,3>> points, double width, double height,  std::string filename){
    int npts = points.size();
    std::vector<std::vector<int>> lines;
    for (int foo=0; foo < npts; foo++){
        lines.push_back({foo, (foo+1)%npts});
    }
    ZoningCommisioner vac;
    vac.insertCurves(points, lines);
    vac.populateNeighbors();
    vac.findFrontier();
    std::cout << vac.trails_.size() << std::endl;
    if (vac.trails_.size() > 0){
        
    }
    CaliperHiker calh(vac.trails_[0], width, height);
    calh.hike();
    WildernessCartographerSVG vv;
    std::vector<std::string> funcolors = {"#F75C03", "#D90368", "#820263", "#291720", "#04A777"};
//
    vv.addCardinalCurveCollection(vac.vacant_, funcolors[3]);
    vv.addCardinalPath(calh.camps_valley_, funcolors[0]);
    vv.addCardinalPath(calh.camps_mountain_,  funcolors[1]);
    if (calh.valid_.size() > 0) vv.addRectangle(calh.getMostValid(), width, height, funcolors[4]);
    vv.writeScalableVectorGraphics(filename);
//
//
}

void example_basic(std::vector<std::array<double,3>> points, std::vector<std::vector<int>> lines, double width, double height,  std::string filename){
    ZoningCommisioner vac;
    vac.insertCurves(points, lines);
    vac.populateNeighbors();
    vac.findFrontier();
//    std::cout << "Numer of trails: " << vac.trails_.size() << std::endl;
    WildernessCartographerSVG vv;
    std::vector<std::string> funcolors = {"#F75C03", "#D90368", "#820263", "#291720", "#04A777"};
//
    vv.addCardinalCurveCollection(vac.vacant_, funcolors[3]);
    for (auto tt: vac.trails_){
        CaliperHiker calh(tt, width, height);
        calh.hike();
        vv.addCardinalPath(calh.camps_valley_, funcolors[0]);
        vv.addCardinalPath(calh.camps_mountain_,  funcolors[1]);
        vv.addRectangle(calh.getMostValid(), width, height, funcolors[4]);
    }
    vv.writeScalableVectorGraphics(filename);
}


void example_sev(){
    std::vector<std::array<double,3>> points = {{0,0},{10,0},{10,10},{0,10}};
    example_basic(points, 3.3,2,"/Users/sscott/Programs/ack/basic0.svg");
    points = {{0,2},{3.3,2},{3.3,0},{10,0},{10,10},{0,10}};
    example_basic(points, 3.2,1.6,"/Users/sscott/Programs/ack/basic1.svg");
    points = {{0,2},{3.3,2},{3.3,1.6},{6.5,1.6},{6.5,0},{10,0},{10,10},{0,10}};
    example_basic(points, 3.1,2.3,"/Users/sscott/Programs/ack/basic2.svg");
    points = {{0,2},{3.3,2},{3.3,1.6},{6.5,1.6},{6.5,2.3},{9.6,2.3},{9.6,0},{10,0},{10,10},{0,10}};
    example_basic(points, 7,1.2,"/Users/sscott/Programs/ack/basic3.svg");
    points ={{0,3.5}, {7,3.5}, {7,2.3}, {9.6,2.3}, {9.6,0}, {10,0}, {10,10}, {0,10}, {0,2}, {3.3,2}, {3.3,1.6}, {6.5,1.6}, {6.5,2.3}, {0, 2.3}};
    std::vector<std::vector<int>> somelines = {{0,1}, {1,2}, {2,3}, {3,4}, {4,5}, {5,6}, {6,7}, {7,0}, {8,9}, {9,10}, {10,11}, {11,12}, {12,13}, {13,8}};
    example_basic(points, somelines, 0.3, 0.6,"/Users/sscott/Programs/ack/basic4.svg");
}

void example_9(){
    std::vector<std::array<double,3>> points ={{0,3.5}, {7,3.5}, {7,2.3}, {9.6,2.3}, {9.6,0}, {10,0}, {10,10}, {0,10}, {0,2}, {3.3,2}, {3.3,1.6}, {6.5,1.6}, {6.5,2.3}, {0, 2.3}};
    std::vector<std::vector<int>> somelines = {{0,1}, {1,2}, {2,3}, {3,4}, {4,5}, {5,6}, {6,7}, {7,0}, {8,9}, {9,10}, {10,11}, {11,12}, {12,13}, {13,8}};
    ZoningCommisioner wild;
    wild.insertCurves(points,somelines);
    wild.populateNeighbors();
    wild.trailblaze();
    Eigen::Vector2d place;
    bool is_placeable = wild.findPlacement(0.3, 0.6, place);
    if (is_placeable){
        std::cout << "Can place." << std::endl;
        WildernessCartographerSVG vv;
        vv.addCardinalCurveCollection(wild.vacant_, "black");
        for (const Trail& t: wild.trails_){
            vv.addCardinalPath(t.landmarks_valley_, "green");
            vv.addCardinalPath(t.landmarks_mountain_, "blue");
        }
        vv.addRectangle(place, 0.3, 0.6, "red");
        vv.writeScalableVectorGraphics("/Users/sscott/Programs/ack/ex9.svg");
        std::cout << "Placed at " << place.transpose() << std::endl;
    }
}

void example_10(){
    std::vector<std::array<double,3>> points = {{0,1},{1,1},{1,0},{10,0},{10,10},{0,10}};
    std::vector<std::vector<int>> somelines = {{0,1},{1,2},{2,3},{3,4},{4,5},{5,0}};
    //
    ZoningCommisioner wild;
    wild.insertCurves(points, somelines);
    wild.populateNeighbors();
    wild.trailblaze();

    Trail t = wild.trails_[0];
    std::cout << " Before " << std::endl;
    for (auto p :  t.landmarks_valley_.points_){
        std::cout << p.transpose() << std::endl;
    }

    Eigen::Vector2d placement;
    double width = 2.4;
    double height = 1.7;
    wild.findPlacement(width, height, placement);
//    wild.removeRectangle(placement, width, height);
    t = wild.trails_[0];
//    removeRectangle(wild.trails_, placement, width, height);
    std::cout << " After " << std::endl;
    for (auto p :  t.landmarks_valley_.points_){
        std::cout << p.transpose() << std::endl;
    }
    //
};

void example_11(){
    std::vector<std::array<double,3>> points = {{0,0},{10,0},{10,10},{0,10}};
    std::vector<std::vector<int>> somelines = {{0,1},{1,2},{2,3},{3,0}};
    //
    ZoningCommisioner wild;
    wild.insertCurves(points, somelines);
    wild.populateNeighbors();
    wild.trailblaze();
    //
    WildernessCartographerSVG wsvg;
    wsvg.addCardinalCurveCollection(wild.vacant_);
	std::vector<Eigen::Vector2d> whs = {{5.4, 2.1}, {4.1, 2.4}, {3.5, 1.9}, {2.5, 2.5}, {1.9, 1.3}, {0.9, 0.8} };
    for (auto wh : whs){
        double width = wh[0];
        double height = wh[1];
        Eigen::Vector2d placement;
        wild.findPlacement(width, height, placement);
        for (auto &t: wild.trails_){
            wsvg.addCardinalPath(t.landmarks_mountain_, "blue");
            wsvg.addCardinalPath(t.landmarks_valley_, "green");
        }
        //removeRectangle(wild.trails_, placement, width, height);
		wild.zoneOff(placement, width, height);
        wsvg.addRectangle(placement, width, height, svgvis::chaosHex());
    }
    wsvg.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example11.svg");
}

void example_12() {
	std::vector<std::array<double, 3>> points = { {0,0},{10,0},{10,10},{0,10} };
	std::vector<std::vector<int>> somelines = { {0,1},{1,2},{2,3},{3,0} };
	//
	ZoningCommisioner wild;
	wild.insertCurves(points, somelines);
	wild.populateNeighbors();
	wild.trailblaze();
	//
	WildernessCartographerSVG wsvg;
	wsvg.addCardinalCurveCollection(wild.vacant_);
	std::vector<Eigen::Vector2d> whs = { {6.9, 4.4} , { 6.6,3.3 } , {5.4, 2.1},  {4.1, 2.4} , {3.5, 1.9} , {2.5, 2.5}, {1.9, 1.3}, {0.9, 0.8}, {0.8,0.8}, {0.8,0.8}, {0.8,0.8} };
	for (auto wh : whs)  {
		double width = wh[0];
		double height = wh[1];
		Eigen::Vector2d placement;
		bool is_placable = wild.findPlacement(width, height, placement);
		for (auto &t : wild.trails_) {
			wsvg.addCardinalPath(t.landmarks_mountain_, "blue");
			wsvg.addCardinalPath(t.landmarks_valley_, "green");
		}
		//removeRectangle(wild.trails_, placement, width, height);
		if (is_placable) {
			wild.zoneOff(placement, width, height);
			wsvg.addPointList(wild.vacant_.points_);
			wsvg.addRectangle(placement, width, height);
			wsvg.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example12.svg");
		}
		else {
			wsvg.addRectangle({11,0}, width, height, svgvis::chaosHex());
		}
	}
	wsvg.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example12.svg");
}

//{6.9, 4.4}, { 6.6,3.3 },

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
	WildernessCartographerSVG vv0;
	vv0.addRectangle(position, width, height);
	for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
	vv0.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect0_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	WildernessCartographerSVG vv;
	vv.addRectangle(position, width, height);
	for (PointList &pl : pls) vv.addPointList(pl, svgvis::chaosHex());
	vv.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect0.svg");
}

void example_addRectTest1() {
	double width = 10;
	double height = 10;
	Eigen::Vector2d position = { 0,0 };
	std::vector<PointList> chains = {
		{{0, 12}, {0, 0}, {12,0}}
	};
	WildernessCartographerSVG vv0;
	vv0.addRectangle(position, width, height);
	for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
	vv0.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect1_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	WildernessCartographerSVG vv;
	vv.addRectangle(position, width, height);
	for (PointList &pl : pls) vv.addPointList(pl, svgvis::chaosHex());
	vv.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect1.svg");
}

void example_addRectTest2() {
	double width = 10;
	double height = 10;
	Eigen::Vector2d position = { 0,0 };
	std::vector<PointList> chains = {
		{{0, 12}, {0, 0}, {10,0}, {10,12}}
	};
	WildernessCartographerSVG vv0;
	vv0.addRectangle(position, width, height);
	for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
	vv0.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect2_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	WildernessCartographerSVG vv;
	vv.addRectangle(position, width, height);
	for (PointList &pl : pls) vv.addPointList(pl, svgvis::chaosHex());
	vv.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect2.svg");
}

void example_addRectTest3() {
	double width = 10;
	double height = 10;
	Eigen::Vector2d position = { 0,0 };
	std::vector<PointList> chains = {
		{{0, 12}, {0, -2}, {5,-2}, {5,0}, {12,0}}
	};
	WildernessCartographerSVG vv0;
	vv0.addRectangle(position, width, height);
	for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
	vv0.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect3_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	WildernessCartographerSVG vv;
	vv.addRectangle(position, width, height);
	for (PointList &pl : pls) vv.addPointList(pl, svgvis::chaosHex());
	vv.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect3.svg");
}

void example_addRectTest4() {
	double width = 10;
	double height = 10;
	Eigen::Vector2d position = { 0,0 };
	std::vector<PointList> chains = {
		{{0, 12}, {0, -2}, {7,-2}, {7,0}, {8,0}, {8, -2}, {9,-2}, {9,-4},}
	};
	WildernessCartographerSVG vv0;
	vv0.addRectangle(position, width, height);
	for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
	vv0.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect4_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	WildernessCartographerSVG vv;
	vv.addRectangle(position, width, height);
	for (PointList &pl : pls) vv.addPointList(pl, svgvis::chaosHex());
	vv.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect4.svg");
}

void example_addRectTest5() {
	double width = 10;
	double height = 10;
	Eigen::Vector2d position = { 0,0 };
	std::vector<PointList> chains = {
		{{0, 12}, {0, -2}, {1,-2}, {1,0}, {2,0}, {2, -2}, {3,-2}, {3,0}, {4,0}, {4, -3}, {5,-3}, {5,0}, {6,0}, {6, -2.6}, {7,-2.6}, {7,0}, {8,0}, {8, -2}, {9,-2}, {9,-4},}
	};
	WildernessCartographerSVG vv0;
	vv0.addRectangle(position, width, height);
	for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
	vv0.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect5_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	WildernessCartographerSVG vv;
	vv.addRectangle(position, width, height);
	for (PointList &pl : pls) vv.addPointList(pl, svgvis::chaosHex());
	vv.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect5.svg");
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
	WildernessCartographerSVG vv0;
	vv0.addRectangle(position, width, height);
	for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
	vv0.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect6_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	WildernessCartographerSVG vv;
	vv.addRectangle(position, width, height);
	for (PointList &pl : pls) vv.addPointList(pl, svgvis::chaosHex());
	vv.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect6.svg");
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
	WildernessCartographerSVG vv0;
	vv0.addRectangle(position, width, height);
	for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
	vv0.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect7_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	WildernessCartographerSVG vv;
	vv.addRectangle(position, width, height);
	for (PointList &pl : pls) vv.addPointList(pl, svgvis::chaosHex());
	vv.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect7.svg");
}

void example_curve_update() {
	std::vector<std::array<double, 3>> points = { {0,0},{10,0},{10,10},{0,10} };
	std::vector<std::vector<int>> somelines = { {0,1},{1,2},{2,3},{3,0} };
	CardinalCurveCollection ccc;
	ccc.setGridPointsAndCells(points, somelines);
	WildernessCartographerSVG vv;
	auto pls = ccc.getPointCycles();
	for (PointList &pl : pls) vv.addPointList(pl, svgvis::chaosHex());
	vv.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_curve_update.svg");
}

void example_why_is_trail_fail() {
	std::vector<std::array<double, 3>> points = { 
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
	wild.trailblaze();
	WildernessCartographerSVG wsvg;
	wsvg.addCardinalCurveCollection(wild.vacant_, "yellow");
	for (auto &t : wild.trails_) {
		wsvg.addCardinalPath(t.landmarks_mountain_, "blue");
		wsvg.addCardinalPath(t.landmarks_valley_, "green");
	}
	wsvg.writeScalableVectorGraphics("/Users/sscott/Programs/ack/badtrail.svg");
	bool is_placable = wild.findPlacement(width, height, placement);
	if (is_placable) {
		std::cout << "PLACE AT: " << placement.transpose() << std::endl;
		wild.zoneOff(placement, width, height);
		wsvg.addPointList(wild.vacant_.points_);
		//wsvg.addRectangle(placement, width, height);
		wsvg.writeScalableVectorGraphics("/Users/sscott/Programs/ack/badtrail.svg");
	}
	else {
		wsvg.addRectangle({ 11,0 }, width, height);
	}
	wsvg.writeScalableVectorGraphics("/Users/sscott/Programs/ack/badtrail.svg");
}

int main() {
    std::cout << "Saluton Mundo!" << std::endl;
	example_12();
	std::cout << "end" << std::endl;
}

// Restructure the wilderness class to use compute trails from fully cyclic chains and then use this thingy!!!!



