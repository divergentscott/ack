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
    CabbieCurveCollection ccc;
    ccc.insertEdges(exda::grid_points, exda::lines);
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
    Wilderness vac;
    vac.insertCurves(exda::grid_points, exda::lines);
    vac.populateNeighbors();
    vac.findFrontier();
    std::cout << "frontier west" << vac.westmost_frontier_id_ << std::endl;
    std::cout << "frontire east" << vac.eastmost_frontier_id_ << std::endl;
    
    for (int foo=0; foo<vac.vsides_.size(); foo++){
        std::cout << foo << std::endl;
        VacancySide vs = vac.vsides_[foo];
        std::cout << " pt " << vs.point_ids[0] << " @ " << vs.points[0].transpose() << std::endl;
        std::cout << " pt " << vs.point_ids[1] << " @ " << vs.points[1].transpose() << std::endl;
        std::cout << "length " << vs.length << " ";
        if (vs.orientation == EdgeOrientation::kEast) std::cout << "horiz";
        if (vs.orientation == EdgeOrientation::kNorth) std::cout << "verti" ;
        if (vs.neighbor_shape == NeighborhoodShape::kZag)            std::cout << "zag" << std::endl;
        if (vs.neighbor_shape == NeighborhoodShape::kNotchNorth)            std::cout << "north" << std::endl;
        if (vs.neighbor_shape == NeighborhoodShape::kNotchEast)            std::cout << "east" << std::endl;
        if (vs.neighbor_shape == NeighborhoodShape::kNotchSouth)            std::cout << "south" << std::endl;
        if (vs.neighbor_shape == NeighborhoodShape::kNotchWest)            std::cout << "west" << std::endl;
    }
    
    std::cout << "Patrol down from 11 " << std::endl;
    Trail patty11 = vac.trailblaze(11);
    Trail patty7 = vac.trailblaze(7);
    
};

void example4(){
    Wilderness vac;
    vac.insertCurves(exda::splotchpoints, exda::splotchlines);
    vac.populateNeighbors();
    vac.findFrontier();
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
    Wilderness vac;
    vac.insertCurves(exda::splotchpoints, exda::splotchlines);
    vac.populateNeighbors();
    vac.findFrontier();
    Trail peopleheadeast = vac.trailblaze(15);
    CaliperHiker calh(peopleheadeast,width, height);
    calh.hike();
    WildernessCartographerSVG vv;
    std::vector<std::string> funcolors = {"#F75C03", "#D90368", "#820263", "#291720", "#04A777"};
    vv.addCabbieCurveCollection(vac.curves, funcolors[3]);
    vv.addCabbiePath(calh.camps_valley_, funcolors[0]);
    vv.addCabbiePath(calh.camps_mountain_,  funcolors[1]);
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
    Wilderness vac;
    std::vector<std::array<double,3>> notchy_points_arraysstyle;
    for (auto foo : notchy_points){
        notchy_points_arraysstyle.push_back({foo[0],foo[1],0});
    }
    vac.insertCurves(notchy_points_arraysstyle, notchy_lines);
    vac.populateNeighbors();
    vac.findFrontier();
    vac.trailblaze();
    CaliperHiker calh(vac.trails_[0], width, height);
    calh.hike();
    WildernessCartographerSVG vv;
    std::vector<std::string> funcolors = {"#F75C03", "#D90368", "#820263", "#291720", "#04A777"};
    
    vv.addCabbieCurveCollection(vac.curves, funcolors[3]);
    vv.addCabbiePath(calh.camps_valley_, funcolors[0]);
    vv.addCabbiePath(calh.camps_mountain_,  funcolors[1]);
    vv.addRectangles(calh.valid_, width, height, funcolors[4]);
    vv.writeScalableVectorGraphics("/Users/sscott/Programs/ack/notchy.svg");
    
};



void example_basic(std::vector<std::array<double,3>> points, double width, double height,  std::string filename){
    int npts = points.size();
    std::vector<std::vector<int>> lines;
    for (int foo=0; foo < npts; foo++){
        lines.push_back({foo, (foo+1)%npts});
    }
    Wilderness vac;
    vac.insertCurves(points, lines);
    vac.populateNeighbors();
    vac.findFrontier();
    vac.trailblaze();
    std::cout << vac.trails_.size() << std::endl;
    if (vac.trails_.size() > 0){
        
    }
    CaliperHiker calh(vac.trails_[0], width, height);
    calh.hike();
    WildernessCartographerSVG vv;
    std::vector<std::string> funcolors = {"#F75C03", "#D90368", "#820263", "#291720", "#04A777"};
//
    vv.addCabbieCurveCollection(vac.curves, funcolors[3]);
    vv.addCabbiePath(calh.camps_valley_, funcolors[0]);
    vv.addCabbiePath(calh.camps_mountain_,  funcolors[1]);
    if (calh.valid_.size() > 0) vv.addRectangle(calh.getMostValid(), width, height, funcolors[4]);
    vv.writeScalableVectorGraphics(filename);
//
//
}

void example_basic(std::vector<std::array<double,3>> points, std::vector<std::vector<int>> lines, double width, double height,  std::string filename){
    Wilderness vac;
    vac.insertCurves(points, lines);
    vac.populateNeighbors();
    vac.findFrontier();
    vac.trailblaze();
    std::cout << "Numer of trails: " << vac.trails_.size() << std::endl;
    WildernessCartographerSVG vv;
    std::vector<std::string> funcolors = {"#F75C03", "#D90368", "#820263", "#291720", "#04A777"};
//
    vv.addCabbieCurveCollection(vac.curves, funcolors[3]);
    for (auto tt: vac.trails_){
        CaliperHiker calh(tt, width, height);
        calh.hike();
        vv.addCabbiePath(calh.camps_valley_, funcolors[0]);
        vv.addCabbiePath(calh.camps_mountain_,  funcolors[1]);
        vv.addRectangle(calh.getMostValid(), width, height, funcolors[4]);
    }
    vv.writeScalableVectorGraphics(filename);


}


void example_sev(){
    std::vector<std::array<double,3>> points;// = {{0,0},{10,0},{10,10},{0,10}};
//    example_basic(points, 3.3,2,"/Users/sscott/Programs/ack/basic0.svg");
//    points = {{0,2},{3.3,2},{3.3,0},{10,0},{10,10},{0,10}};
//    example_basic(points, 3.2,1.6,"/Users/sscott/Programs/ack/basic1.svg");
//    points = {{0,2},{3.3,2},{3.3,1.6},{6.5,1.6},{6.5,0},{10,0},{10,10},{0,10}};
//    example_basic(points, 3.1,2.3,"/Users/sscott/Programs/ack/basic2.svg");
//    points = {{0,2},{3.3,2},{3.3,1.6},{6.5,1.6},{6.5,2.3},{9.6,2.3},{9.6,0},{10,0},{10,10},{0,10}};
//    example_basic(points, 7,1.2,"/Users/sscott/Programs/ack/basic3.svg");
    points ={{0,3.5}, {7,3.5}, {7,2.3}, {9.6,2.3}, {9.6,0}, {10,0}, {10,10}, {0,10}, {0,2}, {3.3,2}, {3.3,1.6}, {6.5,1.6}, {6.5,2.3}, {0, 2.3}};
    std::vector<std::vector<int>> somelines = {{0,1}, {1,2}, {2,3}, {3,4}, {4,5}, {5,6}, {6,7}, {7,0}, {8,9}, {9,10}, {10,11}, {11,12}, {12,13}, {13,8}};
    example_basic(points, somelines, 0.3, 0.6,"/Users/sscott/Programs/ack/basic4.svg");

}

int main() {
    std::cout << "Saluton Mundo!" << std::endl;
    example_sev();
}




