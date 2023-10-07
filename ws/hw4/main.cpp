// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "HelpfulClass.h"


using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    int plot1 = 1;
    int plot2 = 0;
    int plot3 = 0;

    std::vector<Eigen::Vector2d> vertices;

    vertices.push_back(Eigen::Vector2d(0.0,0.0));
    vertices.push_back(Eigen::Vector2d(1.0,2.0));
    vertices.push_back(Eigen::Vector2d(0.0,2.0));
    
    amp::Polygon obstacle(vertices);

    

    double x_curr, y_curr;
    double x_next, y_next;
    double angle;

    std::vector<double> obstacle_side_angles;

    for (int i = 0; i < obstacle.verticesCCW().size(); i++) {
        x_curr = obstacle.verticesCCW()[i][0];
        y_curr = obstacle.verticesCCW()[i][1];
        if (i == obstacle.verticesCCW().size()-1) {
            x_next = obstacle.verticesCCW()[0][0];
            y_next = obstacle.verticesCCW()[0][1];
        } else {
            x_next = obstacle.verticesCCW()[i+1][0];
            y_next = obstacle.verticesCCW()[i+1][1];
        }
        if (abs(x_next-x_curr) > 0.001) {
            angle = atan2(y_next-y_curr, x_next-x_curr);
            if (angle < 0) angle += 2*M_PI;
        } else if (y_next-y_curr > 0) {
            angle = M_PI/2;
        } else {
            angle = 3*M_PI/2;
        }
        obstacle_side_angles.push_back(angle);
    }

    vertices.clear();
    vertices.push_back(Eigen::Vector2d(0.0,0.0));
    vertices.push_back(Eigen::Vector2d(1.0,2.0));
    vertices.push_back(Eigen::Vector2d(0.0,2.0));

    amp::Polygon robot(vertices);

    double rotation_discretization = 12;
    double rotation_angle = 2*M_PI / (rotation_discretization);

    double vertex_angle, angle_rotated;
    double x_curr_rot, y_curr_rot;
    double dist;

    int robot_iter, obstacle_iter;
    double robot_angle, obstacle_angle;

    double smallest_x, smallest_y;
    int smallest_ind;

    double r_x, r_y, o_x, o_y, c_obs_x, c_obs_y;

    std::vector<amp::Polygon> C_OBS;
    std::vector<double> heights_3d;

    for (int i = 0; i < rotation_discretization+1; i++) {

        amp::Polygon rotated_robot;
        std::vector<double> rotated_robot_side_angles, rotated_robot2_side_angles;
        robot_iter = 0;
        obstacle_iter = 0;

        heights_3d.push_back(rotation_angle*i);

        angle_rotated = rotation_angle*i; 
        for (int j = 0; j < robot.verticesCCW().size(); j++) {
            x_curr = robot.verticesCCW()[j][0];
            y_curr = robot.verticesCCW()[j][1];
            x_curr_rot = x_curr*cos(angle_rotated) - y_curr*sin(angle_rotated);
            y_curr_rot = x_curr*sin(angle_rotated) + y_curr*cos(angle_rotated);
            rotated_robot.verticesCCW().push_back(Eigen::Vector2d(-x_curr_rot, -y_curr_rot));
        }

        for (int j = 0; j < rotated_robot.verticesCCW().size(); j++) {
            x_curr = rotated_robot.verticesCCW()[j][0];
            y_curr = rotated_robot.verticesCCW()[j][1];
            if (j == rotated_robot.verticesCCW().size()-1) {
                x_next = rotated_robot.verticesCCW()[0][0];
                y_next = rotated_robot.verticesCCW()[0][1];
            } else {
                x_next = rotated_robot.verticesCCW()[j+1][0];
                y_next = rotated_robot.verticesCCW()[j+1][1];
            }
             if (x_next-x_curr != 0) {
                angle = atan2(y_next-y_curr, x_next-x_curr);
                if (angle < 0) angle += 2*M_PI;
            } else if (y_next-y_curr > 0) {
                angle = M_PI/2;
            } else {
                angle = 3*M_PI/2;
            }
            
            rotated_robot_side_angles.push_back(angle);
        }

        smallest_x = rotated_robot.verticesCCW()[0][0];
        smallest_y = rotated_robot.verticesCCW()[0][1];
        smallest_ind = 0;
        for (int j = 1; j < rotated_robot.verticesCCW().size(); j++) {
            if (rotated_robot.verticesCCW()[j][1] < smallest_y) {
                smallest_ind = j;
                smallest_x   = rotated_robot.verticesCCW()[j][0];
                smallest_y   = rotated_robot.verticesCCW()[j][1];
            } else if (rotated_robot.verticesCCW()[j][1] == smallest_y) {
                if (rotated_robot.verticesCCW()[j][0] < smallest_x) {
                    smallest_ind = j;
                    smallest_x   = rotated_robot.verticesCCW()[j][0];
                    smallest_y   = rotated_robot.verticesCCW()[j][1];
                }
            }
        }

        vertices.clear();
        int iter = 0;
        for (int j = 0; iter < rotated_robot.verticesCCW().size(); j++) {
            if (j + smallest_ind == rotated_robot.verticesCCW().size()) {
                smallest_ind = 0;
                j = 0;
            }
            // vertices.push_back(Eigen::Vector2d(rotated_robot.verticesCCW()[j+smallest_ind][0]-smallest_x,rotated_robot.verticesCCW()[j+smallest_ind][1]-smallest_y));
            vertices.push_back(Eigen::Vector2d(rotated_robot.verticesCCW()[j+smallest_ind][0],rotated_robot.verticesCCW()[j+smallest_ind][1]));
            rotated_robot2_side_angles.push_back(rotated_robot_side_angles[j+smallest_ind]);
            iter++;
        }
        amp::Polygon rotated_robot2(vertices);

        vertices.clear();
        vertices.push_back(Eigen::Vector2d(0.0,0.0));
        amp::Polygon c_obs(vertices);

        while (robot_iter != rotated_robot2_side_angles.size() || obstacle_iter != obstacle_side_angles.size()) {
            robot_angle    = rotated_robot2_side_angles[robot_iter];
            obstacle_angle = obstacle_side_angles[obstacle_iter];

            if (robot_iter == rotated_robot2_side_angles.size()) {
                robot_angle = 2*M_PI + 1;
            }

            if (obstacle_iter == obstacle_side_angles.size()) {
                obstacle_angle = 2*M_PI + 1;
            }

            if (robot_iter != rotated_robot2_side_angles.size() - 1) {
                r_x = rotated_robot2.verticesCCW()[robot_iter+1][0] - rotated_robot2.verticesCCW()[robot_iter][0];
                r_y = rotated_robot2.verticesCCW()[robot_iter+1][1] - rotated_robot2.verticesCCW()[robot_iter][1];
            } else {
                r_x = rotated_robot2.verticesCCW()[0][0] - rotated_robot2.verticesCCW()[robot_iter][0];
                r_y = rotated_robot2.verticesCCW()[0][1] - rotated_robot2.verticesCCW()[robot_iter][1];
            }

            if (obstacle_iter != obstacle_side_angles.size() - 1) {
                o_x = obstacle.verticesCCW()[obstacle_iter+1][0] - obstacle.verticesCCW()[obstacle_iter][0];
                o_y = obstacle.verticesCCW()[obstacle_iter+1][1] - obstacle.verticesCCW()[obstacle_iter][1];
            } else {
                o_x = obstacle.verticesCCW()[0][0] - obstacle.verticesCCW()[obstacle_iter][0];
                o_y = obstacle.verticesCCW()[0][1] - obstacle.verticesCCW()[obstacle_iter][1];
            }
            
            c_obs_x = vertices.back()[0];
            c_obs_y = vertices.back()[1];

            if (robot_angle == obstacle_angle) {
                vertices.push_back(Eigen::Vector2d(c_obs_x+r_x+o_x, c_obs_y+r_y+o_y));
                robot_iter++;
                obstacle_iter++;
            } else if (robot_angle < obstacle_angle) {
                vertices.push_back(Eigen::Vector2d(c_obs_x+r_x, c_obs_y+r_y));
                robot_iter++;
            } else {
                vertices.push_back(Eigen::Vector2d(c_obs_x+o_x, c_obs_y+o_y));
                obstacle_iter++;
            }
        }
        c_obs.verticesCCW() = vertices;
        C_OBS.push_back(c_obs);
    }

    std::vector<amp::Polygon> slice;
    slice.push_back(C_OBS[0]);

    if (plot1) {
        amp::Visualizer::makeFigure(slice,1);
        amp::Visualizer::makeFigure(C_OBS, heights_3d);
        amp::Visualizer::showFigures();
    }

    amp::ManipulatorState known_angles, IK_angles;
    std::vector<double> link_lengths1 = {0.5,1,0.5};
    std::vector<double> link_lengths2 = {1,0.5,1};
    known_angles = {M_PI/6, M_PI/3,7*M_PI/4};
    amp::MyLinkManipulator q2_links1(link_lengths1);
    amp::MyLinkManipulator q2_links2(link_lengths2);
    
    IK_angles = q2_links2.getConfigurationFromIK(Eigen::Vector2d(2,0));

    if (plot2) {
        amp::Visualizer::makeFigure(q2_links1, known_angles);
        amp::Visualizer::showFigures();
        amp::Visualizer::makeFigure(q2_links2, IK_angles);
        amp::Visualizer::showFigures();
    }

    amp::Environment2D env1, env2, env3;
    std::vector<Obstacle2D> obs1, obs2, obs3;
    amp::Obstacle2D o0, o1, o2, o2p;

    vertices.clear();
    vertices.push_back(Eigen::Vector2d(0.25,0.25));
    vertices.push_back(Eigen::Vector2d(0.0,0.75));
    vertices.push_back(Eigen::Vector2d(-0.25,0.25));
    o0.verticesCCW() = vertices;

    vertices.clear();
    vertices.push_back(Eigen::Vector2d(-0.25,1.1));
    vertices.push_back(Eigen::Vector2d(0.25,1.1));
    vertices.push_back(Eigen::Vector2d(0.25,2));
    vertices.push_back(Eigen::Vector2d(-0.25,2));
     o1.verticesCCW() = vertices;

    vertices.clear();
    vertices.push_back(Eigen::Vector2d(-2,-2));
    vertices.push_back(Eigen::Vector2d(2,-2));
    vertices.push_back(Eigen::Vector2d(2,-1.8));
    vertices.push_back(Eigen::Vector2d(-2,-1.8));
    o2.verticesCCW() = vertices;

    vertices.clear();
    vertices.push_back(Eigen::Vector2d(-2,-0.5));
    vertices.push_back(Eigen::Vector2d(2,-0.5));
    vertices.push_back(Eigen::Vector2d(2,-0.3));
    vertices.push_back(Eigen::Vector2d(-2,-0.3));
    o2p.verticesCCW() = vertices;

    obs1.push_back(o0);
    obs2.push_back(o1);
    obs2.push_back(o2);
    obs3.push_back(o1);
    obs3.push_back(o2p);

    env1.obstacles = obs1;
    env1.x_min = -2.0;
    env1.x_max = 2.0;
    env1.y_min = -2.0;
    env1.y_max = 2.0;
    env2.obstacles = obs2;
    env2.x_min = -2.0;
    env2.x_max = 2.0;
    env2.y_min = -2.0;
    env2.y_max = 2.0;
    env3.obstacles = obs3;
    env3.x_min = -2.0;
    env3.x_max = 2.0;
    env3.y_min = -2.0;
    env3.y_max = 2.0;

    int grid_discretization = 360;
    std::size_t grid_size = sizeof(int);
    double dtheta = 2*M_PI / grid_discretization;

    double x0_min = 0;
    double x0_max = 2*M_PI;
    double x1_min = 0;
    double x1_max = 2*M_PI;

    std::vector<double> q3_link_lengths = {1, 1};
    amp::MyLinkManipulator q3_links(q3_link_lengths);
    amp::MyGridCSpace grid1(grid_discretization,grid_discretization,x0_min,x0_max,x1_min,x1_max);
    amp::MyGridCSpace grid2(grid_discretization,grid_discretization,x0_min,x0_max,x1_min,x1_max);
    amp::MyGridCSpace grid3(grid_discretization,grid_discretization,x0_min,x0_max,x1_min,x1_max);

    amp::MyGridCSpace grid11 = grid1.buildCSpace(q3_links, env1);
    amp::MyGridCSpace grid22 = grid2.buildCSpace(q3_links, env2);
    amp::MyGridCSpace grid33 = grid3.buildCSpace(q3_links, env3);

    if (plot3) {
        amp::Visualizer::makeFigure(env1);
        amp::Visualizer::showFigures();
        amp::Visualizer::makeFigure(env2);
        amp::Visualizer::showFigures();
        amp::Visualizer::makeFigure(env3);
        amp::Visualizer::showFigures();

        amp::Visualizer::makeFigure(grid1);
        amp::Visualizer::showFigures();
        amp::Visualizer::makeFigure(grid2);
        amp::Visualizer::showFigures();
        amp::Visualizer::makeFigure(grid3);
        amp::Visualizer::showFigures();
    }

    // MyGridCSpace2DConstructor constructor;
    // Grade method
    // amp::HW4::grade<MyLinkManipulator>(constructor, "mage7128@colorado.edu", argc, argv);
    return 0;
}