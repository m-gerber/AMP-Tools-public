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
        if (i == obstacle.verticesCCW().size()) {
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
    vertices.push_back(Eigen::Vector2d(-1.0,-2.0));
    vertices.push_back(Eigen::Vector2d(0.0,-2.0));

    amp::Polygon robot(vertices);

    double rotation_discretization = 1;
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
            rotated_robot.verticesCCW().push_back(Eigen::Vector2d(x_curr_rot, y_curr_rot));
        }

        for (int j = 0; j < rotated_robot.verticesCCW().size(); j++) {
            x_curr = rotated_robot.verticesCCW()[j][0];
            y_curr = rotated_robot.verticesCCW()[j][1];
            if (j == rotated_robot.verticesCCW().size()) {
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
            }
        }

        vertices.clear();
        int iter = 0;
        for (int j = 0; iter < rotated_robot.verticesCCW().size(); j++) {
            if (j + smallest_ind == rotated_robot.verticesCCW().size()) {
                smallest_ind = 0;
                j = 0;
            }
            vertices.push_back(Eigen::Vector2d(rotated_robot.verticesCCW()[j+smallest_ind][0]-smallest_x,rotated_robot.verticesCCW()[j+smallest_ind][1]-smallest_y));
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


    int plot = 0;
    if (plot) {
        amp::Visualizer::makeFigure(C_OBS, heights_3d);
        amp::Visualizer::showFigures();
    }

    amp::ManipulatorState angles;
    std::vector<double> link_lengths = {8,4,2};
    amp::MyLinkManipulator test_links(link_lengths);
    // test_links.getJointLocation(angles,1);
    angles = test_links.getConfigurationFromIK(Eigen::Vector2d(0,3));
    
    amp::Visualizer::makeFigure(test_links, angles);
    amp::Visualizer::showFigures();

    // Grade method
    // amp::HW4::grade<MyLinkManipulator>(constructor, "nonhuman.biologic@myspace.edu", argc, argv);
    return 0;
}