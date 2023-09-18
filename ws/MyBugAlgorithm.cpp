#include "MyBugAlgorithm.h"
#include <thread>

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a path definition.
    amp::Path2D path;

    // Initialize the starting point for the bug to be the intial point for the bug.
    double x_bug = problem.q_init[0];
    double y_bug = problem.q_init[1];

    // Create a vector of type PathPoints to hold x, y, coordinates of steps, step sizes, and distance to goal.
    std::vector<amp::PathPoints> bug_path;
    std::vector<amp::PathPoints> follow_path;

    // Define the first point in this vector to be start point.
    amp::PathPoints path_points;
    path_points.x = x_bug;
    path_points.y = y_bug;
    path_points.step = 0;
    bug_path.push_back(path_points);

    // Set the value for sizes of steps to take toward the goal with each move.
    double step_size = 0.02;
    // Set the distance to check ahead of the bug at each step to avoid obstacle collision.
    double check_distance = 0.02;

    // Create doubles to hold the x, y, and Euclidian distance to goal along with the angle to the goal.
    double dx, dy, distance_to_goal, angle_to_goal;
    // Create doubles to hold the step size taken in the x and y directions.
    double step_x, step_y;
    double forward_check;
    int follow_obstacle;

    int hit_index;
    int following = 0;

    int num_angles = 20;
    double ahead_x;
    double ahead_y;
    double angle_view, angle_start, angle_delta;

    bool inside;
    
    // Bug 1 Algorithm
    while (!goalReached(bug_path.back().x,bug_path.back().y, step_size, problem)) {
        dx = problem.q_goal[0] - bug_path.back().x;
        dy = problem.q_goal[1] - bug_path.back().y;
        distance_to_goal = sqrt(pow(dx,2) + pow(dy,2));
        bug_path.back().goal_distance = distance_to_goal;
        angle_to_goal = atan(dy/dx);
        if (dx < 0) {
            angle_to_goal += M_PI;
        }

        angle_view  = M_PI;
        angle_delta = angle_view / (num_angles - 1);

        ahead_x = bug_path.back().x + check_distance * cos(angle_to_goal);
        ahead_y = bug_path.back().y + check_distance * sin(angle_to_goal);
        inside = inPolygon(ahead_x,ahead_y,problem);

        if (!inside) {
            step_x = step_size * cos(angle_to_goal);
            step_y = step_size * sin(angle_to_goal);

            path_points.x = bug_path.back().x + step_x;
            path_points.y = bug_path.back().y + step_y;
            path_points.step = step_size;
            path_points.goal_distance = distance_to_goal;
            bug_path.push_back(path_points);
        } else {
            follow_path = bugFollow(bug_path.back().x, bug_path.back().y, 1, problem);
            bug_path.insert(bug_path.end(), follow_path.begin(), follow_path.end());
        }
    }
    
    path_points.x = problem.q_goal[0];
    path_points.y = problem.q_goal[1];
    bug_path.push_back(path_points);

    // path.waypoints.push_back(problem.q_init);
    // path.waypoints.push_back(Eigen::Vector2d(1.0, 5.0));
    // path.waypoints.push_back(Eigen::Vector2d(3.0, 9.0));
    // path.waypoints.push_back(problem.q_goal);

    for (int i = 0; i < bug_path.size(); i++) {
        path.waypoints.push_back(Eigen::Vector2d(bug_path[i].x, bug_path[i].y));
    }
    return path;
}
