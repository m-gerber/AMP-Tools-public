#include "MyBugAlgorithm.h"
#include <thread>

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a path definition.
    amp::Path2D path;

    // amp::Line line1, line2;
    // amp::Point p1, p2;
    // p1.x = 0;
    // p1.y = 0;
    // p2.x = 5;
    // p2.y = 5;
    // line1.p1 = p1;
    // line1.p2 = p2;
    // p1.x = 3;
    // p1.y = 3;
    // p2.x = 7;
    // p2.y = 7;
    // line2.p1 = p1;
    // line2.p2 = p2;
    // std::cout << "intersects? " << (lineIntersect2(line1,line2) ? "yes" : "no") << std::endl;
    // PAUSE;

    int algo = 1;

    std::vector<amp::PathPoints> bug_path = makePath(2, problem);

    double distance;
    for (int i = 0; i < bug_path.size(); i++) distance += bug_path[i].step;

    //std::cout << "Using Bug " << problem.algo << ", the Bug travelled " << distance << " units." << std::endl;

    for (int i = 0; i < bug_path.size(); i++) {
        path.waypoints.push_back(Eigen::Vector2d(bug_path[i].x, bug_path[i].y));
    }
    return path;
}
