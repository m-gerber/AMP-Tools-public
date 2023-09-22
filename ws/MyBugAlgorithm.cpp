#include "MyBugAlgorithm.h"
#include <thread>

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a path definition.
    amp::Path2D path;

    int algo = 2;

    std::vector<amp::PathPoints> bug_path = makePath(algo, problem);

    double distance;
    for (int i = 0; i < bug_path.size(); i++) distance += bug_path[i].step;

    std::cout << "Using Bug " << algo << ", the Bug travelled " << distance << " units." << std::endl;

    for (int i = 0; i < bug_path.size(); i++) {
        path.waypoints.push_back(Eigen::Vector2d(bug_path[i].x, bug_path[i].y));
    }
    return path;
}
