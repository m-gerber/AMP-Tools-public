#include "MyGridCSpace.h"

amp::MyGridCSpace amp::MyGridCSpace::buildCSpace(amp::MyLinkManipulator links, amp::Environment2D env) {
    environment_ = env;
    links_ = links; 

    amp::MyGridCSpace temp_grid(360, 360, 0 , 2*M_PI, 0, 2*M_PI);

    int grid_discretization = denseArray.size().first;
    double dtheta = 2*M_PI / grid_discretization;

    double x0, x1;

    for (int i = 0; i < grid_discretization; i++) {
        x0 = dtheta*i;
        for (int j = 0; j < grid_discretization; j++) {
            x1 = dtheta*j;
            denseArray(i,j) = inCollision(x0,x1);
            temp_grid(i,j) = inCollision(x0,x1);
        }
    }
    return temp_grid;
}

bool amp::MyGridCSpace::inCollision(double angle0, double angle1) const {
    amp::ManipulatorState state = {angle0,angle1};
    Eigen::Vector2d j1 = links_.getJointLocation(state,1);
    Eigen::Vector2d j2 = links_.getJointLocation(state,2);

    double x0 = 0;
    double y0 = 0;
    double x1 = j1[0];
    double y1 = j1[1];
    double x2 = j2[0];
    double y2 = j2[1];

    double t, u;
    double x3, y3, x4, y4;

    double slope1, slope2;
    double x1_y3, x1_y4, x2_y3, x2_y4;

    int num_obstacles = environment_.obstacles.size();
    int num_vertices;

    bool collision = false;

    for (int i = 0; i < num_obstacles; i++) {
        num_vertices = environment_.obstacles[i].verticesCCW().size();
        for (int j = 0; j < num_vertices; j++) {
            x3 = environment_.obstacles[i].verticesCCW()[j][0];
            y3 = environment_.obstacles[i].verticesCCW()[j][1];
            x4 = environment_.obstacles[i].verticesCCW()[j+1][0];
            y4 = environment_.obstacles[i].verticesCCW()[j+1][1];
            if (j == num_vertices-1) {
                x4 = environment_.obstacles[i].verticesCCW()[0][0];
                y4 = environment_.obstacles[i].verticesCCW()[0][1];
            }
            slope1 = (y2-y1)/(x2-x1);
            slope2 = (y4-y3)/(x4-x3);
            if (((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)) != 0) {
                t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / ((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4));
                u = ((x1-x3)*(y1-y2) - (y1-y3)*(x1-x2)) / ((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4));
                if ((t >= 0 && t <= 1) && (u >= 0 && u <= 1)) collision = true;
            } else if ((slope1 == slope2) || (slope1 == -slope2)) {
                x1_y3 = slope2 * (x1 - x3) + y3;
                x1_y4 = slope2 * (x1 - x4) + y4;
                x2_y3 = slope2 * (x2 - x3) + y3;
                x2_y4 = slope2 * (x2 - x4) + y4;
                if ((y1 == x1_y3) || (y1 == x1_y4) || (y2 == x2_y3) || (y2 == x2_y4)) {
                    collision = true;
                }
            } 
            slope1 = (y1-y0)/(1-x0);
            slope2 = (y4-y3)/(x4-x3);
            if (((x0-x1)*(y3-y4) - (y0-y1)*(x3-x4)) != 0) {
                t = ((x0-x3)*(y3-y4) - (y0-y3)*(x3-x4)) / ((x0-x1)*(y3-y4) - (y0-y1)*(x3-x4));
                u = ((x0-x3)*(y0-y1) - (y0-y3)*(x0-x1)) / ((x0-x1)*(y3-y4) - (y0-y1)*(x3-x4));
                if ((t >= 0 && t <= 1) && (u >= 0 && u <= 1)) collision = true;
            } else if ((slope1 == slope2) || (slope1 == -slope2)) {
                x1_y3 = slope2 * (x0 - x3) + y3;
                x1_y4 = slope2 * (x0 - x4) + y4;
                x2_y3 = slope2 * (x1 - x3) + y3;
                x2_y4 = slope2 * (x1 - x4) + y4;
                if ((y0 == x1_y3) || (y0 == x1_y4) || (y1 == x2_y3) || (y1 == x2_y4)) {
                    collision = true;
                }
            } 
            if (collision) return collision;
        }
    }
    return collision;
}