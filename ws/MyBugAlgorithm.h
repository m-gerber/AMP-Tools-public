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

struct PolyInfo {
    bool inside = false;
    int obstacle = -1;
};

struct WallInfo {
    bool isVertex = false;
    double distance;
    double obstacle;
    double x_close = -1;
    double y_close = -1;
    double x0;
    double y0;
    double x1;
    double y1;
    double x2;
    double y2;
    
};

}

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Add any other methods here...

        bool goalReached(double x, double y, double step_size, const amp::Problem2D& problem) const {
            double dx = problem.q_goal[0] - x;
            double dy = problem.q_goal[1] - y;
            double distance_to_goal = sqrt(pow(dx,2) + pow(dy,2));
            return distance_to_goal < step_size;
        }
        
        amp::PolyInfo nearPolygon(double x_bug, double y_bug, const amp::Problem2D& problem) const {
            // Define ints to hold values for the number of obstacles in the environment and the number of 
            // vertices for each of those obstacles
            int num_obstacles = problem.obstacles.size();
            int num_vertices;

            amp::PolyInfo polygon;

            double x1, y1, x2, y2;

            for (int i = 0; i < num_obstacles; i++) {
                num_vertices = problem.obstacles[i].verticesCCW().size();
                polygon.obstacle = i;
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
                                    polygon.inside = !polygon.inside;
                                }
                            }
                        }
                    }
                } if (polygon.inside) return polygon;
            }
            return polygon;
        }
        
        amp::WallInfo findWall(double x_bug, double y_bug, const amp::Problem2D& problem) const {
            // Define ints to hold values for the number of obstacles in the environment and the number of 
            // vertices for each of those obstacles
            int num_obstacles = problem.obstacles.size();
            int num_vertices;

            double x0, y0, x1, y1, x2, y2;

            std::vector<std::vector<amp::WallInfo>> walls;
            walls.resize(num_obstacles);

            amp::WallInfo temp_wall;

            for (int i = 0; i < num_obstacles; i++) {
                num_vertices = problem.obstacles[i].verticesCCW().size();
                walls[i].resize(num_vertices);
                for (int j = 0; j < num_vertices; j++) {
                    x1 = problem.obstacles[i].verticesCCW()[j][0];
                    y1 = problem.obstacles[i].verticesCCW()[j][1];
                    if (j == num_vertices-1) {
                        x2 = problem.obstacles[i].verticesCCW()[0][0];
                        y2 = problem.obstacles[i].verticesCCW()[0][1];
                    } else {
                        x2 = problem.obstacles[i].verticesCCW()[j+1][0];
                        y2 = problem.obstacles[i].verticesCCW()[j+1][1];
                    }
                    if (j == 0) {
                        x0 = problem.obstacles[i].verticesCCW()[num_vertices-1][0];
                        y0 = problem.obstacles[i].verticesCCW()[num_vertices-1][1];
                    } else {
                        x0 = problem.obstacles[i].verticesCCW()[j-1][0];
                        y0 = problem.obstacles[i].verticesCCW()[j-1][1];
                    }

                    // std::cout << "(x ,y ): (" << x_bug << "," << y_bug << ")" << std::endl;
                    // std::cout << "(x0,y0): (" << x0 << "," << y0 << ")" << std::endl;
                    // std::cout << "(x1,y1): (" << x1 << "," << y1 << ")" << std::endl;
                    // std::cout << "(x2,y2): (" << x2 << "," << y2 << ")" << std::endl << std::endl;

                    int num_steps = 101;

                    double x_step = (x2-x1) / (num_steps-1);
                    double y_step = (y2-y1) / (num_steps-1);

                    double x_new, y_new, distance;

                    double closest_vertex = sqrt(pow(x_bug-x1,2) + pow(y_bug-y1,2));
                    temp_wall.x_close = x1;
                    temp_wall.y_close = y1;
                    if (sqrt(pow(x_bug-x2,2) + pow(y_bug-y2,2)) < closest_vertex) {
                        closest_vertex = sqrt(pow(x_bug-x2,2) + pow(y_bug-y2,2));
                        temp_wall.x_close = x2;
                        temp_wall.y_close = y2;
                    }
                    temp_wall.distance = closest_vertex;
                    temp_wall.obstacle = i;
                    temp_wall.x0 = x0;
                    temp_wall.y0 = y0;
                    temp_wall.x1 = x1;
                    temp_wall.y1 = y1;
                    temp_wall.x2 = x2;
                    temp_wall.y2 = y2;

                    for (int k = 1; k < num_steps-2; k++) {
                        x_new = x1 + x_step*k;
                        y_new = y1 + y_step*k;
                        double dist = sqrt(pow(x_bug-x_new,2) + pow(y_bug-y_new,2));
                        if (dist <= temp_wall.distance) {
                            temp_wall.distance = dist;
                            temp_wall.x_close = x_new;
                            temp_wall.y_close = y_new;
                        }
                    }

                    std::cout << "(x,y): (" << x_bug << "," << y_bug << ")" << std::endl;
                    std::cout << "(x,y): (" << temp_wall.x_close << "," << temp_wall.y_close << ")" << std::endl << std::endl;

                    temp_wall.isVertex = false;
                    if (std::abs(temp_wall.distance - closest_vertex) < 0.001) {
                        temp_wall.isVertex = true; 
                    }
                    walls[i][j] = temp_wall;

                    // LOG(temp_wall.distance);
                    // LOG(walls[i][j].distance);
                    // LOG(temp_wall.x_close);
                    // LOG(walls[i][j].x_close);
                    // LOG(temp_wall.y_close);
                    // LOG(walls[i][j].y_close);
                    // LOG(temp_wall.x1);
                    // LOG(walls[i][j].x1);
                    // LOG(temp_wall.y1);
                    // LOG(walls[i][j].y1);
                    // LOG(temp_wall.x2);
                    // LOG(walls[i][j].x2);
                    // LOG(temp_wall.y2);
                    // LOG(walls[i][j].y2);

                    // if (i == 4 && j == 0) {
                    //     std::this_thread::sleep_for(std::chrono::milliseconds(100000));
                    // }
                }
            }
            
            int closest_obstacle = 0;
            int closest_wall = 0;
            double closest_distance = walls[0][0].distance;
            
            for (int i = 0; i < num_obstacles; i++) {
                num_vertices = problem.obstacles[i].verticesCCW().size();
                for (int j = 0; j < num_vertices; j++) {
                    if (walls[i][j].distance < closest_distance) {
                        // LOG(walls[i][j].distance);
                        closest_obstacle = i;
                        closest_wall = j;
                    } else if (walls[i][j].distance == closest_distance && j == num_vertices - 1) {
                        std::cout << "hello" << std::endl;
                        closest_wall = num_vertices - 1;
                    }
                }
            }
            // LOG(walls[closest_obstacle][closest_wall].distance);
            // std::cout << "closest wall: " << walls[0][0].x1 << std::endl;
            // std::cout << "closest wall: " << walls[0][0].y1 << std::endl;
            // std::cout << "closest wall: " << walls[0][0].x2 << std::endl;
            // std::cout << "closest wall: " << walls[0][0].y2 << std::endl;
            // std::cout << "closest wall: " << walls[0][3].distance << std::endl;
            // std::cout << "closest wall: " << walls[0][0].distance << std::endl;
            return walls[closest_obstacle][closest_wall];
        }

    private:
        // Add any member variables here...
};