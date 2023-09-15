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

    // Define the first point in this vector to be start point.
    amp::PathPoints path_points;
    path_points.x = x_bug;
    path_points.y = y_bug;
    path_points.step = 0;
    bug_path.push_back(path_points);

    // Set an iterator.
    int iter = 0;
    // Set the value for sizes of steps to take toward the goal with each move.
    double step_size = 0.02;
    // Set the distance to check ahead of the bug at each step to avoid obstacle collision.
    double check_distance = 0.25;

    // Create doubles to hold the x, y, and Euclidian distance to goal along with the angle to the goal.
    double dx, dy, distance_to_goal, angle_to_goal;
    // Create doubles to hold the step size taken in the x and y directions.
    double step_x, step_y;
    double forward_check;
    int follow_obstacle;

    int hit_index;
    int following = 0;

    int num_angles = 20;
    double ahead_x[num_angles];
    double ahead_y[num_angles];
    double angle_view, angle_start, angle_delta;

    bool inside;

    int test = 0;


    bug_path[iter].x = 0.763675;
    bug_path[iter].y = 5.14368;
    bug_path[iter].x = 13;
    bug_path[iter].y = 5;
    amp::WallInfo closestWall = findWall(bug_path[iter].x,bug_path[iter].y, problem);
    amp::PolyInfo nearbyPolgon = nearPolygon(bug_path[iter].x,bug_path[iter].y,problem);

    std::cout << "(x,y):   (" << bug_path[iter].x << "," << bug_path[iter].y << ")" << std::endl;
    std::cout << "(x1,y1): (" << (closestWall.x1) << "," << (closestWall.y1) << ")" << std::endl;
    std::cout << "(x2,y2): (" << (closestWall.x2) << "," << (closestWall.y2) << ")" << std::endl << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100000));
    
    // Bug 1 Algorithm
    while (!goalReached(bug_path[iter].x,bug_path[iter].y, step_size, problem) && iter <= 10000) {
        dx = problem.q_goal[0] - bug_path[iter].x;
        dy = problem.q_goal[1] - bug_path[iter].y;
        distance_to_goal = sqrt(pow(dx,2) + pow(dy,2));
        bug_path[iter].goal_distance = distance_to_goal;
        angle_to_goal = atan(dy/dx);

        angle_view  = M_PI/2;
        angle_start = angle_to_goal - angle_view/2;
        angle_delta = angle_view / (num_angles - 1);

        inside = false;
        std::vector<amp::PolyInfo> nearbyPolygon;
        nearbyPolygon.resize(num_angles);
        amp::PolyInfo tempPolygon;
        for (int i = 0; i < num_angles; i++) {
            if (i*angle_delta <= M_PI/4) {
                ahead_x[i] = bug_path[iter].x + check_distance;
                ahead_y[i] = bug_path[iter].y + check_distance * tan(angle_start + i*angle_delta);
            } else {
                ahead_y[i] = bug_path[iter].y + check_distance;
                ahead_x[i] = bug_path[iter].x + check_distance * tan(angle_start + i*angle_delta);
            }
            tempPolygon = nearPolygon(ahead_x[i],ahead_y[i],problem);
            nearbyPolygon[i] = tempPolygon;
            if (nearbyPolygon[i].inside) {
                inside = true;
                check_distance = check_distance*99/100;
                following = 1;
                hit_index = iter;
            }
        }

        // std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (!inside && !following) {
            step_x = step_size * cos(angle_to_goal);
            step_y = step_size * sin(angle_to_goal);

            path_points.x = bug_path[iter].x + step_x;
            path_points.y = bug_path[iter].y + step_y;
            path_points.step = step_size;
            path_points.goal_distance = distance_to_goal;
            bug_path.push_back(path_points);
            iter++;
        } else {
            test++;

            amp::WallInfo closestWall = findWall(bug_path[iter].x, bug_path[iter].y, problem);

            // std::cout << "(x,y):   (" << bug_path[iter].x << "," << bug_path[iter].y << ")" << std::endl;
            // std::cout << "(x1,y1): (" << (closestWall.x1) << "," << (closestWall.y1) << ")" << std::endl;
            // std::cout << "(x2,y2): (" << (closestWall.x2) << "," << (closestWall.y2) << ")" << std::endl << std::endl;

            double angle_along_wall = atan((closestWall.y1-closestWall.y2)/(closestWall.x1-closestWall.x2));
            double angle_next = atan((closestWall.y0-closestWall.y1)/(closestWall.x0-closestWall.x1));
            double internal_angle = abs(angle_along_wall - angle_next);
            if (internal_angle > M_PI) {
                internal_angle = internal_angle - M_PI;
            }
            double turn_angle = (2*M_PI - internal_angle) / 2;
            
            for (int i = 0; i < num_angles; i++) {
                nearbyPolygon[i].inside = 0;
            }

            double end_x = closestWall.x1 + (check_distance-0.05) * cos(turn_angle);
            double end_y = closestWall.y1 + (check_distance-0.05) * sin(turn_angle);

            inside = false;

            while(!inside && (bug_path[iter].x <= end_x) && (bug_path[iter].y <= end_y)) {
                angle_start = angle_along_wall - angle_view/2;

                inside = false;
                for (int i = 0; i < num_angles; i++) {
                    if (i*angle_delta <= M_PI/4) {
                        ahead_x[i] = bug_path[iter].x + check_distance;
                        ahead_y[i] = bug_path[iter].y + check_distance * tan(angle_start + i*angle_delta);
                    } else {
                        ahead_y[i] = bug_path[iter].y + check_distance;
                        ahead_x[i] = bug_path[iter].x + check_distance * tan(angle_start + i*angle_delta);
                    }
                    tempPolygon = nearPolygon(ahead_x[i],ahead_y[i],problem);
                    nearbyPolygon[i] = tempPolygon;
                    if (nearbyPolygon[i].inside) {
                        inside = true;
                    }
                }

                // nearbyPolygon = nearPolygon(ahead_x,ahead_y,problem);

                step_x = step_size * cos(angle_along_wall);
                step_y = step_size * sin(angle_along_wall);

                path_points.x = bug_path[iter].x + step_x;
                path_points.y = bug_path[iter].y + step_y;
                path_points.step = step_size;
                path_points.goal_distance = distance_to_goal;
                bug_path.push_back(path_points);
                iter++;
            }

            // std::cout << (closestWall.x1) << std::endl;
            // std::cout << (closestWall.y1) << std::endl;
            // std::cout << (closestWall.x2) << std::endl;
            // std::cout << (closestWall.y2) << std::endl;
            // std::cout << (bug_path[iter].x <= end_x) << std::endl;
            // std::cout << (bug_path[iter].x) << std::endl;
            // std::cout << (end_x) << std::endl;
            // std::cout << (bug_path[iter].y <= end_y) << std::endl;
            // std::cout << (bug_path[iter].y) << std::endl;
            // std::cout << (end_y) << std::endl << std::endl;
            
            if (test == 10) {
                break;
            }
            //break;
            
            // if (closestWall.isVertex && ) {

            // }

            // path_points.x = 0.75;
            // path_points.y = 5.25;
            // bug_path.push_back(path_points);
            // break;
        }
    }
    
    path_points.x = problem.q_goal[0];
    path_points.y = problem.q_goal[1];
    path_points.x = 10;
    path_points.y = 10;
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
