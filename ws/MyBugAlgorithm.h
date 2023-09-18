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

        // https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/#
        int direction(amp::Point p, amp::Point q, amp::Point r) {
            double orientation = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
            if (orientation == 0) {
                return 0;
            } else if (orientation > 0) {
                return 1;
            }
            return 2;
        }

        bool onLine(amp::Point p, amp::Point q, amp::Point r)
        {
            if ((q.x <= std::max(p.x, r.x)) && (q.x >= std::min(p.x, r.x)) &&
                (q.y <= std::max(p.y, r.y)) && (q.y >= std::min(p.y, r.y))) {
                return true;
            }
            return false;
        }

        bool cross(amp::Line line, const amp::Problem2D& problem) {
            int o1, o2, o3, o4;
            amp::Point p1, q1, p2, q2;

            p1 = line.p1;
            q1 = line.p2;

            int num_obstacles = problem.obstacles.size();
            int num_vertices;

            bool cross = false;

            for (int i = 0; i < num_obstacles; i++) {
                num_vertices = problem.obstacles[i].verticesCCW().size();
                for (int j = 0; j < num_vertices; j++) {
                    p2.x = problem.obstacles[i].verticesCCW()[j][0];
                    p2.y = problem.obstacles[i].verticesCCW()[j][1];
                    q2.x = problem.obstacles[i].verticesCCW()[j+1][0];
                    q2.y = problem.obstacles[i].verticesCCW()[j+1][1];
                    if (j == num_vertices-1) {
                        q2.x = problem.obstacles[i].verticesCCW()[0][0];
                        q2.y = problem.obstacles[i].verticesCCW()[0][1];
                    }
                    o1 = direction(p1, q1, p2);
                    o2 = direction(p1, q1, q2);
                    o3 = direction(p2, q2, p1);
                    o4 = direction(p2, q2, q1);

                    if ((o1 != o2) && (o3 != o4)) cross = true;
                    if ((o1 == 0) && onLine(p1, p2, q1)) cross = true;
                    if ((o2 == 0) && onLine(p1, q2, q1)) cross = true;
                    if ((o3 == 0) && onLine(p2, p1, q2)) cross = true;
                    if ((o4 == 0) && onLine(p2, q1, q2)) cross = true;
                    if (intersect(p1,q1,p2,q2)) cross = true;
                } if (cross) return cross;
            }
            return cross;
        }

        bool ccw(amp::Point A, amp::Point B, amp::Point C) {
            return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x);
        }

        bool intersect(amp::Point A, amp::Point B, amp::Point C, amp::Point D) {
            return ((ccw(A,C,D) != ccw(B,C,D)) && (ccw(A,B,C) != ccw(A,B,D)));
        }

        std::vector<amp::PathPoints> bugFollow(double x_start, double y_start, int algo, const amp::Problem2D& problem) {
            int angle_i = 0;

            double dx, dy;
            double closest_distance;

            int closest_ind;

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
            crosses_prev = cross(line, problem);


            angle_i++;

            x_step = x_start + step_size*cos(angle_i*angle_step);
            y_step = y_start + step_size*sin(angle_i*angle_step);
            point2.x = x_step;
            point2.y = y_step;
            line.p2 = point2;
            crosses = cross(line, problem);

            angle_i++;

            while (!((crosses_prev == 1) && (crosses == 0))) {
                crosses_prev = crosses;
                x_step = x_start + step_size*cos(angle_i*angle_step);
                y_step = y_start + step_size*sin(angle_i*angle_step);
                point2.x = x_step;
                point2.y = y_step;
                line.p2 = point2;
                crosses = cross(line, problem);
                angle_i++;
            }

            x_step = x_start + step_size*cos(angle_i*angle_step);
            y_step = y_start + step_size*sin(angle_i*angle_step);

            dx = problem.q_goal[0] - x_step;
            dy = problem.q_goal[1] - y_step;

            angle_prev = atan(dy/dx);
            if (dx < 0) {
                angle_prev += M_PI;
            }
            angle_prev += M_PI/2;
            
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

                input_angle = angle_i*angle_step + angle_prev-M_PI/2;
                x_step = follow_path.back().x + step_size*cos(input_angle);
                y_step = follow_path.back().y + step_size*sin(input_angle);
                point2.x = x_step;
                point2.y = y_step;
                line.p2 = point2;
                crosses_prev = cross(line, problem);

                angle_i++;

                input_angle = angle_i*angle_step + angle_prev-M_PI/2.2;
                x_step = follow_path.back().x + step_size*cos(input_angle);
                y_step = follow_path.back().y + step_size*sin(input_angle);
                point2.x = x_step;
                point2.y = y_step;
                line.p2 = point2;
                crosses = cross(line, problem);

                angle_i++;

                while (!((crosses_prev == 1) && (crosses == 0))) {
                    crosses_prev = crosses;
                    input_angle = angle_i*angle_step + angle_prev-M_PI/2.2;
                    x_step = follow_path.back().x + step_size*cos(input_angle);
                    y_step = follow_path.back().y + step_size*sin(input_angle);
                    point2.x = x_step;
                    point2.y = y_step;
                    line.p2 = point2;
                    crosses = cross(line, problem);
                    inside = inPolygon(x_step, y_step, problem);
                    if (inside && !crosses) crosses = !crosses; 
                    angle_i++;
                    if (angle_i == num_angles) {
                        angle_i = 0;
                        num_angles = 94;
                        while (!((crosses_prev == 1) && (crosses == 0))) {
                            input_angle = angle_i*angle_step + angle_prev-M_PI/2.2;
                            x_step = follow_path.back().x + step_size*cos(input_angle);
                            y_step = follow_path.back().y + step_size*sin(input_angle);
                            point2.x = x_step;
                            point2.y = y_step;
                            line.p2 = point2;
                            crosses = cross(line, problem);
                            angle_i++;
                        }
                        num_angles = 36;
                    };
                }

                input_angle = angle_i*angle_step + angle_prev-M_PI/2.2;
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
                            if (sqrt(pow(follow_path.back().x-x_start,2) + pow(follow_path.back().y-y_start,2)) <= 2*step_size) {
                                closest_distance = follow_path[0].goal_distance;
                                int num_points_followed = follow_path.size();
                                for (int i = 0; i < num_points_followed; i++) {
                                    if (follow_path[i].goal_distance < closest_distance) {
                                        closest_distance = follow_path[i].goal_distance;
                                        closest_ind = i;
                                    }
                                }

                                if (closest_ind/num_points_followed < 0.5) {
                                    for (int i = 0; i < closest_ind; i++) {
                                        follow_point.x = follow_path[i].x;
                                        follow_point.y = follow_path[i].y;
                                        follow_point.step = follow_path[i].step;
                                        follow_path.push_back(follow_point);
                                    }
                                } else {
                                    for (int i = num_points_followed; i > closest_ind; i--) {
                                        follow_point.x = follow_path[i].x;
                                        follow_point.y = follow_path[i].y;
                                        follow_point.step = follow_path[i].step;
                                        follow_path.push_back(follow_point);
                                    }
                                }
                                break;
                            }
                        } else if (algo == 2) {
                            m_line_y = m_line_slope * (follow_path.back().x - x_begin) + y_begin;
                            if ((std::abs(follow_path.back().y - m_line_y) < 2*step_size)) {
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