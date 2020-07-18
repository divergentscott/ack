#include <iostream>
#include <limits>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "zoning_cartographer_svg.h"


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
    Zone patty11 = vac.trailblaze(11);
    Zone patty7 = vac.trailblaze(7);
};

void example4(){
    ZoningCommisioner vac;
    vac.insertCurves(exda::splotchpoints, exda::splotchlines);
    vac.populateNeighbors();
    Zone peopleheadeast = vac.trailblaze(15);
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
    Zone peopleheadeast = vac.trailblaze(15);
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
    vac.trailblaze();
    Surveyor calh(vac.trails_[0], width, height);
    calh.hike();
    svgvis::ZoningCartographerSVG vv;
    std::vector<std::string> funcolors = {"#F75C03", "#D90368", "#820263", "#291720", "#04A777"};
    
    vv.addCardinalCurveCollection(vac.vacant_, funcolors[3]);
    vv.addCardinalPath(calh.camps_downtown_, funcolors[0]);
    vv.addCardinalPath(calh.camps_uptown_,  funcolors[1]);
    vv.addRectangles(calh.valid_lots_, width, height, funcolors[4]);
    vv.writeScalableVectorGraphics("/Users/sscott/Programs/ack/notchy.svg");
    
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
    std::cout << vac.trails_.size() << std::endl;
    if (vac.trails_.size() > 0){
        
    }
    Surveyor calh(vac.trails_[0], width, height);
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
    for (auto tt: vac.trails_){
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
    std::vector<Eigen::Vector2d> points ={{0,3.5}, {7,3.5}, {7,2.3}, {9.6,2.3}, {9.6,0}, {10,0}, {10,10}, {0,10}, {0,2}, {3.3,2}, {3.3,1.6}, {6.5,1.6}, {6.5,2.3}, {0, 2.3}};
    std::vector<std::vector<int>> somelines = {{0,1}, {1,2}, {2,3}, {3,4}, {4,5}, {5,6}, {6,7}, {7,0}, {8,9}, {9,10}, {10,11}, {11,12}, {12,13}, {13,8}};
    ZoningCommisioner wild;
    wild.insertCurves(points,somelines);
    wild.populateNeighbors();
    wild.trailblaze();
    Eigen::Vector2d place;
    bool is_placeable = wild.findPlacement(0.3, 0.6, place);
    if (is_placeable){
        std::cout << "Can place." << std::endl;
        svgvis::ZoningCartographerSVG vv;
        vv.addCardinalCurveCollection(wild.vacant_, "black");
        for (const Zone& t: wild.trails_){
            vv.addCardinalPath(t.landmarks_downtown_, "green");
            vv.addCardinalPath(t.landmarks_uptown_, "blue");
        }
        vv.addRectangle(place, 0.3, 0.6, "red");
        vv.writeScalableVectorGraphics("/Users/sscott/Programs/ack/ex9.svg");
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
    wild.trailblaze();

    Zone t = wild.trails_[0];
    std::cout << " Before " << std::endl;
    for (auto p :  t.landmarks_downtown_.points_){
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
    wild.trailblaze();
    //
    svgvis::ZoningCartographerSVG wsvg;
    wsvg.addCardinalCurveCollection(wild.vacant_);
	std::vector<Eigen::Vector2d> whs = {{5.4, 2.1}, {4.1, 2.4}, {3.5, 1.9}, {2.5, 2.5}, {1.9, 1.3}, {0.9, 0.8} };
    for (auto wh : whs){
        double width = wh[0];
        double height = wh[1];
        Eigen::Vector2d placement;
        wild.findPlacement(width, height, placement);
        for (auto &t: wild.trails_){
            wsvg.addCardinalPath(t.landmarks_uptown_, "blue");
            wsvg.addCardinalPath(t.landmarks_downtown_, "green");
        }
        //removeRectangle(wild.trails_, placement, width, height);
		wild.zoneOff(placement, width, height);
        wsvg.addRectangle(placement, width, height, svgvis::chaosHex());
    }
    wsvg.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example11.svg");
}

void example_12() {
	std::vector<Eigen::Vector2d> points = { {0,0},{10,0},{10,10},{0,10} };
	std::vector<std::vector<int>> somelines = { {0,1},{1,2},{2,3},{3,0} };
	//
	ZoningCommisioner wild;
	wild.insertCurves(points, somelines);
	wild.populateNeighbors();
	wild.trailblaze();
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
			wsvg.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example12.svg");
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

	wsvg.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example12.svg");
}

void example_125() {
	std::vector<Eigen::Vector2d> points = { {0,0},{10,0},{10,10},{0,10} };
	std::vector<std::vector<int>> somelines = { {0,1},{1,2},{2,3},{3,0} };
	//
	ZoningCommisioner wild;
	wild.insertCurves(points, somelines);
	wild.populateNeighbors();
	wild.trailblaze();
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
			wsvg.writeScalableVectorGraphics("C:/Users/sscott/Programs/ack/example125.svg");
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
	wsvg.writeScalableVectorGraphics("C:/Users/sscott/Programs/ack/example125.svg");
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
    wild.trailblaze();
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
		for (auto &t : wild.trails_) {
			svgvis::ZoningCartographerSVG svger;
			svger.addCardinalCurveCollection(wild.vacant_, "black");
			svger.addCardinalPath(t.landmarks_uptown_, "blue");
			svger.addCardinalPath(t.landmarks_downtown_, "green");
			Surveyor caliper_hiker(t, width, height);
			caliper_hiker.hike();
			svger.addCardinalPath(caliper_hiker.camps_uptown_, "orange");
			svger.addCardinalPath(caliper_hiker.camps_downtown_, "red");
			svger.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example13_"+std::to_string(trail_cnt)+".svg");
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
        wsvg.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example13.svg");
        //removeRectangle(wild.trails_, placement, width, height);
        if (is_placable) {
            wild.zoneOff(placement, width, height);
            wsvg.addRectangle(placement, width, height);
            wsvg.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example13.svg");
        } else {
            wsvg.addRectangle({11,0}, width, height);
            wsvg.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example13.svg");
        }
    }
    wsvg.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example13.svg");
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
	wild.trailblaze();
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
		for (auto &t : wild.trails_) {
			svgvis::ZoningCartographerSVG svger;
			svger.addCardinalCurveCollection(wild.vacant_, "black");
			svger.addCardinalPath(t.landmarks_uptown_, "blue");
			svger.addCardinalPath(t.landmarks_downtown_, "green");
			Surveyor caliper_hiker(t, width, height);
			caliper_hiker.hike();
			svger.addCardinalPath(caliper_hiker.camps_uptown_, "orange");
			svger.addCardinalPath(caliper_hiker.camps_downtown_, "red");
			svger.writeScalableVectorGraphics("C:/Users/sscott/Programs/ack/example135_" + std::to_string(trail_cnt) + ".svg");
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
		wsvg.writeScalableVectorGraphics("C:/Users/sscott/Programs/ack/example135.svg");
		//removeRectangle(wild.trails_, placement, width, height);
		if (is_placable) {
			wild.zoneOff(placement, width, height);
			wsvg.addRectangle(placement, width, height);
			wsvg.writeScalableVectorGraphics("C:/Users/sscott/Programs/ack/example135.svg");
		}
		else {
			wsvg.addRectangle({ 11,0 }, width, height);
			wsvg.writeScalableVectorGraphics("C:/Users/sscott/Programs/ack/example135.svg");
		}
	}
	wsvg.writeScalableVectorGraphics("C:/Users/sscott/Programs/ack/example135.svg");
}

void example_14() {
	std::vector<Eigen::Vector2d> points = { {0,0}, {1,0}, {1,-1}, {3.4,-1}, {3.4,-.6}, {5,-.6}, {5,2}, {0,2} };
    std::vector<std::vector<int>> somelines = { {0,1},{1,2},{2,3},{3,4},{4,5},{5,6},{6,7},{7,0} };
    //
    ZoningCommisioner wild;
    wild.insertCurves(points, somelines);
    wild.populateNeighbors();
    wild.trailblaze();
    //
    svgvis::ZoningCartographerSVG wsvg;
    wsvg.addCardinalCurveCollection(wild.vacant_);
	std::vector<Eigen::Vector2d> whs = { {2.4,1}, {2.4,1}, {2.4,1}, {2.4,1}, {2.4,1}, {2.4,1} };
    for (auto wh : whs)  {
        double width = wh[0];
        double height = wh[1];
        Eigen::Vector2d placement;
        bool is_placable = wild.findPlacement(width, height, placement);
        for (auto &t : wild.trails_) {
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
            wsvg.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example14.svg");
        } else {
            wsvg.addRectangle({6,0}, width, height);
            wsvg.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example14.svg");
        }
    }
    wsvg.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example14.svg");
}
//{6.9, 4.4}, { 6.6,3.3 },

void example_15() {
	std::vector<Eigen::Vector2d> points = { {0,0}, {1,0}, {1,-1}, {3.4,-1}, {3.4,-.6}, {5,-.6}, {5,-2.5}, {6,-2.5}, {6,-3}, {8.4,-3}, {8.4,5}, {0,5} };
	std::vector<std::vector<int>> somelines = { {0,1},{1,2},{2,3},{3,4},{4,5},{5,6},{6,7}, {7,8}, {8,9}, {9,10}, {10,11}, {11,0}};
	//
	ZoningCommisioner wild;
	wild.insertCurves(points, somelines);
	wild.populateNeighbors();
	wild.trailblaze();
	//
	svgvis::ZoningCartographerSVG wsvg;
	wsvg.addCardinalCurveCollection(wild.vacant_, "black");
	wsvg.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example15.svg");
	std::vector<Eigen::Vector2d> whs = { {2.4,1} };
	for (auto wh : whs) {
		double width = wh[0];
		double height = wh[1];
		Eigen::Vector2d placement;
		bool is_placable = wild.findPlacement(width, height, placement);
		for (auto &t : wild.trails_) {
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
			wsvg.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example15.svg");
		}
		else {
			wsvg.addRectangle({ 10,0 }, width, height);
			wsvg.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example15.svg");
		}
	}
	wsvg.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example15.svg");
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
	vv0.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect0_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	svgvis::ZoningCartographerSVG vv;
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
	svgvis::ZoningCartographerSVG vv0;
	vv0.addRectangle(position, width, height);
	for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
	vv0.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect1_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	svgvis::ZoningCartographerSVG vv;
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
	svgvis::ZoningCartographerSVG vv0;
	vv0.addRectangle(position, width, height);
	for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
	vv0.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect2_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	svgvis::ZoningCartographerSVG vv;
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
	svgvis::ZoningCartographerSVG vv0;
	vv0.addRectangle(position, width, height);
	for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
	vv0.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect3_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	svgvis::ZoningCartographerSVG vv;
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
	svgvis::ZoningCartographerSVG vv0;
	vv0.addRectangle(position, width, height);
	for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
	vv0.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect4_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	svgvis::ZoningCartographerSVG vv;
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
	svgvis::ZoningCartographerSVG vv0;
	vv0.addRectangle(position, width, height);
	for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
	vv0.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect5_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	svgvis::ZoningCartographerSVG vv;
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
	svgvis::ZoningCartographerSVG vv0;
	vv0.addRectangle(position, width, height);
	for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
	vv0.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect6_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	svgvis::ZoningCartographerSVG vv;
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
	svgvis::ZoningCartographerSVG vv0;
	vv0.addRectangle(position, width, height);
	for (PointList &pl : chains) vv0.addPointList(pl, svgvis::chaosHex());
	vv0.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect7_before.svg");

	std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
	svgvis::ZoningCartographerSVG vv;
	vv.addRectangle(position, width, height);
	for (PointList &pl : pls) vv.addPointList(pl, svgvis::chaosHex());
	vv.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect7.svg");
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
    vv0.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect8_before.svg");

    std::vector<PointList> pls = addRectangleModZ2(chains, position, width, height);
    svgvis::ZoningCartographerSVG vv;
    vv.addRectangle(position, width, height);
    for (PointList &pl : pls) vv.addPointList(pl, svgvis::chaosHex());
    vv.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_rect8.svg");
}

void example_curve_update() {
	std::vector<Eigen::Vector2d> points = { {0,0},{10,0},{10,10},{0,10} };
	std::vector<std::vector<int>> somelines = { {0,1},{1,2},{2,3},{3,0} };
	CardinalCurveCollection ccc;
	ccc.setGridPointsAndCells(points, somelines);
	svgvis::ZoningCartographerSVG vv;
	auto pls = ccc.getPointCycles();
	for (PointList &pl : pls) vv.addPointList(pl, svgvis::chaosHex());
	vv.writeScalableVectorGraphics("/Users/sscott/Programs/ack/example_curve_update.svg");
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
	wild.trailblaze();
	svgvis::ZoningCartographerSVG wsvg;
	wsvg.addCardinalCurveCollection(wild.vacant_, "yellow");
	for (auto &t : wild.trails_) {
		wsvg.addCardinalPath(t.landmarks_uptown_, "blue");
		wsvg.addCardinalPath(t.landmarks_downtown_, "green");
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

#include "zoning_board.h"

void example_zoning_board1() {
	ZoningBoard zb;
	PointList ps = { {0,0},{10,0},{10,10},{0,10} };
	std::vector<std::vector<int>> es = { {0,1},{1,2},{2,3},{3,0} };
	zb.addVacancy(ps, es);
	std::vector<Eigen::Vector2d> rectangles = {
		{1,0.5},{1,0.5},{1,0.5},{1,0.5},{1,0.5},{1,0.5},{6,6},{7,6},{8,6},{9,6}, {4,4},{1,1},{1,1},{1,1},{1,1},{1,1},{3,2},{3,3},{1,8},{2,2},{4,4},{5,1},{4,2},{2,2},{9,2},{5,5},{3,1}
	};
	zb.setRectangles(rectangles);
	zb.zone();
	svgvis::ZoningCartographerSVG zsvg;
	zsvg.addZoningBoardReport(zb);
	zsvg.writeScalableVectorGraphics("C:/Users/sscott/Programs/ack/zb1.svg");
};

void example_zoning_board2() {
	ZoningBoard zb;
	
	PointList ps = { {0,0},{10,0},{10,10},{0,10} };
	std::vector<std::vector<int>> es = { {0,1},{1,2},{2,3},{3,0} };
	zb.addVacancy(ps, es);
	
	PointList ps2 = {{0,0}, {0,-2}, {6,-2}, {6,0}, {8,0}, {8,6}, {6,6}, {6,8}, {0,8}, {0,6}, {-2,6}, {-2,0}};
	zb.addVacancy(ps2);

	PointList ps3 = { {0,0},{10,0},{10,5},{5,5},{5,10},{0,10} };
	zb.addVacancy(ps3);
	
	PointList ps4 = { {0,0},{10,0},{10,10},{6,10},{6,4},{4,4},{4,10},{0,10} };
	zb.addVacancy(ps4);

	std::vector<Eigen::Vector2d> rectangles = {
		{10,1.4},{10,1.4}, {0.3,0.8},{0.3,0.8},{0.3,4.8}, {1,0.5},{1,0.5},{1,0.5},{1,0.5},{0.3,0.8},{0.3,0.8},{0.3,4.8}, {1,0.5},{1,0.5},{1,0.5},{1,0.5},{1,0.5},{1,0.5},{6,6},{7,6},{8,6},{9,6}, {4,4},{1,1},{1,1},{1,1},{1,1},{1,1},{3,2},{3,3},{1,8},{2,2},{4,4},{5,1},{4,2},{2,2},{9,2},{5,5},{3,1}
	};

	zb.setRectangles(rectangles);
	zb.zone();

	svgvis::ZoningCartographerSVG zsvg;
	zsvg.addZoningBoardReport(zb);
	zsvg.writeScalableVectorGraphics("C:/Users/sscott/Programs/ack/zb2.svg");
};

void example_zoning_board3() {
	ZoningBoard zb;

	PointList ps = { {0,0},{10,0},{10,10},{0,10} };
	std::vector<std::vector<int>> es = { {0,1},{1,2},{2,3},{3,0} };
	zb.addVacancy(ps, es);

	PointList ps2 = { {0,0}, {0,-2}, {6,-2}, {6,0}, {8,0}, {8,6}, {6,6}, {6,8}, {0,8}, {0,6}, {-2,6}, {-2,0} };
	zb.addVacancy(ps2);

	PointList ps3 = { {0,0},{10,0},{10,5},{5,5},{5,10},{0,10} };
	zb.addVacancy(ps3);

	PointList ps4 = { {0,0},{10,0},{10,10},{6,10},{6,4},{4,4},{4,10},{0,10} };
	zb.addVacancy(ps4);

	std::vector<Eigen::Vector2d> rectangles = {
		{2,2},{3,3},{4,4},{1,5},{3.8,4.2},{1.3,1.4}
	};

	std::vector<int> multiplicities = {
		5,10,15,7,9,6
	};

	zb.setRectangles(rectangles, multiplicities);
	zb.zone();

	svgvis::ZoningCartographerSVG zsvg;
	zsvg.addZoningBoardReport(zb);
	zsvg.writeScalableVectorGraphics("C:/Users/sscott/Programs/ack/zb3.svg");
};

void example_zoning_board4() {
	ZoningBoard zb;

	PointList ps = { {0,0},{10,0},{10,10},{0,10} , {4,4},{6,4},{6,6},{4,4}};
	std::vector<std::vector<int>> es = { {0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4} };
	zb.addVacancy(ps, es);

	std::vector<Eigen::Vector2d> rectangles = {
		{2,2},{3,3},{4,4},{1,5},{3.8,4.2},{1.3,1.4}
	};

	std::vector<int> multiplicities = {
		5,10,15,7,9,6
	};

	zb.setRectangles(rectangles, multiplicities);
	zb.zone();

	svgvis::ZoningCartographerSVG zsvg;
	zsvg.addZoningBoardReport(zb);
	zsvg.writeScalableVectorGraphics("C:/Users/sscott/Programs/ack/zb4.svg");
};

int main() {
    std::cout << "Saluton Mundo!" << std::endl;
	example_zoning_board4();
	//example_125();
	//example_135();
//	example_13() ;
	//example_15();
	//example_14();
	std::cout << "end" << std::endl;
}




