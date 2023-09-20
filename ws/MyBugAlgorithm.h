#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

#include <thread>

namespace amp {

struct PathPoints {
    double x;
    double y;
    double step;
    double goal_distance = -1;
};

struct Point {
    double x;
    double y;
};

struct Line : Point {
    Point p1;
    Point p2;
};

}

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        std::vector<amp::PathPoints> makePath(int algo, const amp::Problem2D& problem) {
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
                    follow_path = bugFollow(bug_path.back().x, bug_path.back().y, algo, problem);
                    bug_path.insert(bug_path.end(), follow_path.begin(), follow_path.end());
                }
            }
            
            path_points.x = problem.q_goal[0];
            path_points.y = problem.q_goal[1];
            bug_path.push_back(path_points);

            return bug_path;
        }

        bool goalReached(double x, double y, double step_size, const amp::Problem2D& problem) const {
            double dx = problem.q_goal[0] - x;
            double dy = problem.q_goal[1] - y;
            double distance_to_goal = sqrt(pow(dx,2) + pow(dy,2));
            return distance_to_goal < step_size;
        }
        
        bool inPolygon(double x_bug, double y_bug, const amp::Problem2D& problem) const {
            // Define ints to hold values for the number of obstacles in the environment and the number of 
            // vertices for each of those obstacles
            int num_obstacles = problem.obstacles.size();
            int num_vertices;

            bool inside = false;

            double x1, y1, x2, y2;

            for (int i = 0; i < num_obstacles; i++) {
                num_vertices = problem.obstacles[i].verticesCCW().size();
                for (int j = 0; j < num_vertices; j++) {
                    x1 = problem.obstacles[i].verticesCCW()[j][0];
                    y1 = problem.obstacles[i].verticesCCW()[j][1];
                    x2 = problem.obstacles[i].verticesCCW()[j+1][0];
                    y2 = problem.obstacles[i].verticesCCW()[j+1][1];
                    if (j == num_vertices-1) {
                        x2 = problem.obstacles[i].verticesCCW()[0][0];
                        y2 = problem.obstacles[i].verticesCCW()[0][1];
                    }
                    if (y_bug > std::fmin(y1,y2)) {
                        if (y_bug <= std::fmax(y1,y2)){
                            if (x_bug <= std::fmax(x1,x2)) {
                                // Compute the location of the x-intercept on the line
                                double x_int = (x2-x1) * ((y_bug-y1) / (y2-y1)) + x1;
                                if (x2 == x1 || x_bug <= x_int) {
                                    inside = !inside;
                                }
                            }
                        }
                    }
                } if (inside) return inside;
            }
            return inside;
        }

        bool lineIntersect(amp::Line line, const amp::Problem2D& problem) {
            double x1 = line.p1.x;
            double y1 = line.p1.y;
            double x2 = line.p2.x;
            double y2 = line.p2.y;

            double t, u;
            double x3, y3, x4, y4;

            double slope1, slope2;
            double x1_y3, x1_y4, x2_y3, x2_y4;

            int num_obstacles = problem.obstacles.size();
            int num_vertices;

            for (int i = 0; i < num_obstacles; i++) {
                num_vertices = problem.obstacles[i].verticesCCW().size();
                for (int j = 0; j < num_vertices; j++) {
                    x3 = problem.obstacles[i].verticesCCW()[j][0];
                    y3 = problem.obstacles[i].verticesCCW()[j][1];
                    x4 = problem.obstacles[i].verticesCCW()[j+1][0];
                    y4 = problem.obstacles[i].verticesCCW()[j+1][1];
                    if (j == num_vertices-1) {
                        x4 = problem.obstacles[i].verticesCCW()[0][0];
                        y4 = problem.obstacles[i].verticesCCW()[0][1];
                    }
                    slope1 = (y2-y1)/(x2-x1);
                    slope2 = (y4-y3)/(x4-x3);
                    if (((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)) != 0) {
                        t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / ((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4));
                        u = ((x1-x3)*(y1-y2) - (y1-y3)*(x1-x2)) / ((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4));
                        if ((t >= 0 && t <= 1) && (u >= 0 && u <= 1)) return true;
                    } else if ((slope1 == slope2) || (slope1 == -slope2)) {
                        x1_y3 = slope2 * (x1 - x3) + y3;
                        x1_y4 = slope2 * (x1 - x4) + y4;
                        x2_y3 = slope2 * (x2 - x3) + y3;
                        x2_y4 = slope2 * (x2 - x4) + y4;
                        if ((y1 == x1_y3) || (y1 == x1_y4) || (y2 == x2_y3) || (y2 == x2_y4)) {
                            return true;
                        }
                    } else return false;
                }
            }
            return false;
        }
/*
        bool lineIntersect2(amp::Line line1, amp::Line line2) {
            double x1 = line1.p1.x;
            double y1 = line1.p1.y;
            double x2 = line1.p2.x;
            double y2 = line1.p2.y;

            double x3 = line2.p1.x;
            double y3 = line2.p1.y;
            double x4 = line2.p2.x;
            double y4 = line2.p2.y;

            double t, u;

            double slope1, slope2;
            double x1_y3, x1_y4, x2_y3, x2_y4;

            slope1 = (y2-y1)/(x2-x1);
            slope2 = (y4-y3)/(x4-x3);
           
            if (((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)) != 0) {
                t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / ((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4));
                u = ((x1-x3)*(y1-y2) - (y1-y3)*(x1-x2)) / ((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4));
                if ((t >= 0 && t <= 1) && (u >= 0 && u <= 1)) return true;
            } else if ((slope1 == slope2) || (slope1 == -slope2)) {
                x1_y3 = slope2 * (x1 - x3) + y3;
                x1_y4 = slope2 * (x1 - x4) + y4;
                x2_y3 = slope2 * (x2 - x3) + y3;
                x2_y4 = slope2 * (x2 - x4) + y4;
                if ((y1 == x1_y3) || (y1 == x1_y4) || (y2 == x2_y3) || (y2 == x2_y4)) {
                    return true;
                }
            } else return false;
            return false;
        }
*/
        std::vector<amp::PathPoints> bugFollow(double x_start, double y_start, int algo, const amp::Problem2D& problem) {
            int angle_i = 0;

            double dx, dy;
            double closest_distance;

            double closest_ind;

            // Set the value for sizes of steps to take toward the goal with each move.
            double step_size = 0.03;
            // Set the distance to check ahead of the bug at each step to avoid obstacle collision.
            double check_distance = 0.03;

            int num_angles = 36;
            double angle_step = 2*M_PI / num_angles;

            double x_step;
            double y_step;

            double start_dist = sqrt(pow(x_start-problem.q_goal[0],2) + pow(y_start-problem.q_goal[1],2));
            double x_begin = problem.q_init[0];
            double y_begin = problem.q_init[1];
            double x_goal  = problem.q_goal[0];
            double y_goal  = problem.q_goal[1];

            double m_line_y;
            double m_line_slope = (y_goal-y_begin)/(x_goal-x_begin);

            double angle_prev, input_angle;

            bool inside, crosses, crosses_prev;
            amp::Point point1, point2;
            amp::Line line;

            point1.x = x_start;
            point1.y = y_start;
            line.p1 = point1;

            x_step = x_start + step_size*cos(angle_i*angle_step);
            y_step = y_start + step_size*sin(angle_i*angle_step);
            point2.x = x_step;
            point2.y = y_step;
            line.p2 = point2;
            crosses_prev = lineIntersect(line, problem);

            angle_i++;

            x_step = x_start + step_size*cos(angle_i*angle_step);
            y_step = y_start + step_size*sin(angle_i*angle_step);
            point2.x = x_step;
            point2.y = y_step;
            line.p2 = point2;
            crosses = lineIntersect(line, problem);

            angle_i++;

            while (!((crosses_prev == 1) && (crosses == 0))) {
                crosses_prev = crosses;
                x_step = x_start + step_size*cos(angle_i*angle_step);
                y_step = y_start + step_size*sin(angle_i*angle_step);
                point2.x = x_step;
                point2.y = y_step;
                line.p2 = point2;
                crosses = lineIntersect(line, problem);
                angle_i++;
            }

            x_step = x_start + step_size*cos(angle_i*angle_step);
            y_step = y_start + step_size*sin(angle_i*angle_step);

            dx = x_step - problem.q_goal[0];
            dy = y_step - problem.q_goal[1];

            angle_prev = atan(dy/dx);
            if (dx < 0) {
                angle_prev += M_PI;
            }
            
            std::vector<amp::PathPoints> follow_path;

            // Define the first point in this vector to be start point.
            amp::PathPoints follow_point;
            follow_point.x = x_step;
            follow_point.y = y_step;
            follow_point.step = step_size;
            follow_point.goal_distance = sqrt(pow(dx,2) + pow(dy,2));
            follow_path.push_back(follow_point);

            
            while (true) {
                point1.x = follow_path.back().x;
                point1.y = follow_path.back().y;
                line.p1 = point1;

                angle_i = 0;

                input_angle = angle_i*angle_step + angle_prev+M_PI*1.1;
                x_step = follow_path.back().x + step_size*cos(input_angle);
                y_step = follow_path.back().y + step_size*sin(input_angle);
                point2.x = x_step;
                point2.y = y_step;
                line.p2 = point2;
                crosses_prev = lineIntersect(line, problem);

                angle_i++;

                input_angle = angle_i*angle_step + angle_prev+M_PI*1.1;
                x_step = follow_path.back().x + step_size*cos(input_angle);
                y_step = follow_path.back().y + step_size*sin(input_angle);
                point2.x = x_step;
                point2.y = y_step;
                line.p2 = point2;
                crosses = lineIntersect(line, problem);

                while (!((crosses_prev == 1) && (crosses == 0))) {
                    angle_i++;
                    crosses_prev = crosses;
                    input_angle = angle_i*angle_step + angle_prev+M_PI*1.1;
                    x_step = follow_path.back().x + step_size*cos(input_angle);
                    y_step = follow_path.back().y + step_size*sin(input_angle);
                    point2.x = x_step;
                    point2.y = y_step;
                    line.p2 = point2;
                    crosses = lineIntersect(line, problem);
                    inside = inPolygon(x_step, y_step, problem);
                    if (inside && !crosses) crosses = !crosses;
                    if (angle_i == num_angles) {
                        angle_i = 0;
                        num_angles = 94;
                        while (!((crosses_prev == 1) && (crosses == 0))) {
                            angle_i++;
                            input_angle = angle_i*angle_step + angle_prev+M_PI*1.1;
                            x_step = follow_path.back().x + step_size*cos(input_angle);
                            y_step = follow_path.back().y + step_size*sin(input_angle);
                            point2.x = x_step;
                            point2.y = y_step;
                            line.p2 = point2;
                            crosses = lineIntersect(line, problem);
                        }
                        break;
                        num_angles = 36;
                    };
                }

                x_step = follow_path.back().x + step_size*cos(input_angle);
                y_step = follow_path.back().y + step_size*sin(input_angle);
                angle_prev = input_angle;

                if (!inPolygon(x_step, y_step, problem)) {
                    dx = problem.q_goal[0] - x_step;
                    dy = problem.q_goal[1] - y_step;
                    follow_point.goal_distance = sqrt(pow(dx,2) + pow(dy,2));

                    follow_point.x = x_step;
                    follow_point.y = y_step;
                    follow_point.step = step_size;
                    follow_path.push_back(follow_point);

                    if (follow_path.size() > 5) {
                        if (algo == 1) {
                            double distance_from_start = sqrt(pow(follow_path.back().x-x_start,2) + pow(follow_path.back().y-y_start,2));
                            if (distance_from_start <= 2*step_size) {
                                point1.x = follow_path.back().x;
                                point1.y = follow_path.back().y;
                                line.p1 = point1;
                                point2.x = x_start;
                                point2.y = y_start;
                                line.p2 = point2;
                                crosses = lineIntersect(line, problem);
                                if (crosses == false) {
                                    follow_point.x = x_start;
                                    follow_point.y = y_start;
                                    follow_point.step = distance_from_start;
                                    follow_path.push_back(follow_point);
                                    closest_distance = follow_path[0].goal_distance;
                                    double num_points_followed = follow_path.size();
                                    for (int i = 0; i < num_points_followed; i++) {
                                        if (follow_path[i].goal_distance < closest_distance) {
                                            closest_distance = follow_path[i].goal_distance;
                                            closest_ind = i;
                                        }
                                    }
                                    if (closest_ind/num_points_followed < 0.5) {
                                        for (int i = 0; i < closest_ind+1; i++) {
                                            follow_point.x = follow_path[i].x;
                                            follow_point.y = follow_path[i].y;
                                            follow_point.step = follow_path[i].step;
                                            follow_path.push_back(follow_point);
                                        }
                                    } else {
                                        for (int i = num_points_followed; i > closest_ind-1; i--) {
                                            follow_point.x = follow_path[i].x;
                                            follow_point.y = follow_path[i].y;
                                            follow_point.step = follow_path[i].step;
                                            follow_path.push_back(follow_point);
                                        }
                                    }
                                    break;
                                }
                            }
                        } else if (algo == 2) {
                            m_line_y = m_line_slope * (follow_path.back().x - x_begin) + y_begin;
                            if ((std::abs(follow_path.back().y - m_line_y) <= step_size)) {
                                if (start_dist - follow_path.back().goal_distance > 2*step_size) {
                                    break;
                                }
                            }
                        }
                    }
                }
            }
            return follow_path;
        }

    private:
        // Add any member variables here...
};