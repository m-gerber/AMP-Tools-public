#include "MyGridCSpace.h"

void amp::MyGridCSpace::buildPointCSpace(amp::Environment2D env) {
    env_ = env;

    int grid_discretization = size().first;
    double dtheta1 = (env.x_max-env.x_min) / grid_discretization;
    double dtheta2 = (env.y_max-env.y_min) / grid_discretization;

    double x0, x1;
    bool isIn;

    int dx[] = { -1, 0, 1,  0 };
    int dy[] = {  0, 1, 0, -1 };

    for (int i = 0; i < grid_discretization; i++) {
        x0 = env.x_min + dtheta1*i;
        for (int j = 0; j < grid_discretization; j++) {
            x1 = env.y_min + dtheta2*j;
            isIn = inPolygon(x0,x1);
            operator()(i,j) = inPolygon(x0,x1);
            if (isIn) {
                for (int k = 0; k < 4; k++) {
                    operator()(i+dx[k],j+dy[k]) = 1;
                }
            }
        }
    }
}

bool amp::MyGridCSpace::inPolygon(double x_pos, double y_pos) const {
    // Define ints to hold values for the number of obstacles in the environment and the number of 
    // vertices for each of those obstacles
    int num_obstacles = env_.obstacles.size();
    int num_vertices;

    bool inside = false;

    double x1, y1, x2, y2;

    for (int i = 0; i < num_obstacles; i++) {
        num_vertices = env_.obstacles[i].verticesCCW().size();
        for (int j = 0; j < num_vertices; j++) {
            x1 = env_.obstacles[i].verticesCCW()[j][0];
            y1 = env_.obstacles[i].verticesCCW()[j][1];
            x2 = env_.obstacles[i].verticesCCW()[j+1][0];
            y2 = env_.obstacles[i].verticesCCW()[j+1][1];
            if (j == num_vertices-1) {
                x2 = env_.obstacles[i].verticesCCW()[0][0];
                y2 = env_.obstacles[i].verticesCCW()[0][1];
            }
            if (y_pos > std::fmin(y1,y2)) {
                if (y_pos <= std::fmax(y1,y2)){
                    if (x_pos <= std::fmax(x1,x2)) {
                        // Compute the location of the x-intercept on the line
                        double x_int = (x2-x1) * ((y_pos-y1) / (y2-y1)) + x1;
                        if (x2 == x1 || x_pos <= x_int) {
                            inside = !inside;
                        }
                    }
                }
            }
        } if (inside) return inside;
    }
    return inside;
}

void amp::MyGridCSpace::buildLinkCSpace(amp::MyLinkManipulator links, amp::Environment2D env) {
    env_ = env;
    links_ = links; 

    int grid_discretization = size().first;
    double dtheta = 2*M_PI / grid_discretization;

    double x0, x1;

    for (int i = 0; i < grid_discretization; i++) {
        x0 = dtheta*i;
        for (int j = 0; j < grid_discretization; j++) {
            x1 = dtheta*j;
            operator()(i,j) = collisionChecker(x0,x1);
        }
    }
}

bool amp::MyGridCSpace::collisionChecker(double angle0, double angle1) const {
    amp::ManipulatorState state(2);
    state <<  angle0, angle1;

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

    int num_obstacles = env_.obstacles.size();
    int num_vertices;

    bool collision = false;

    for (int i = 0; i < num_obstacles; i++) {
        num_vertices = env_.obstacles[i].verticesCCW().size();
        for (int j = 0; j < num_vertices; j++) {
            x3 = env_.obstacles[i].verticesCCW()[j][0];
            y3 = env_.obstacles[i].verticesCCW()[j][1];
            x4 = env_.obstacles[i].verticesCCW()[j+1][0];
            y4 = env_.obstacles[i].verticesCCW()[j+1][1];
            if (j == num_vertices-1) {
                x4 = env_.obstacles[i].verticesCCW()[0][0];
                y4 = env_.obstacles[i].verticesCCW()[0][1];
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