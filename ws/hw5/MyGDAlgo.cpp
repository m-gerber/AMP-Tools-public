#include "MyGDAlgo.h"
#include <thread>
#include <cstdlib>

Eigen::Vector2d amp::MyGDAlgo::distance2point(Eigen::Vector2d p1, Eigen::Vector2d p2) {
    double dy = p2[1] - p1[1];
    double dx = p2[0] - p1[0];

    double dist = sqrt( dx*dx + dy*dy );
    double angle = atan2(dy,dx);
    
    return Eigen::Vector2d(dist, angle);
}

Eigen::Vector2d amp::MyGDAlgo::Grad_U_att(const amp::Problem2D& problem, Eigen::Vector2d point) {
    Eigen::Vector2d d_goal = distance2point(point, problem.q_goal);

    double x_step, y_step;

    if (d_goal[0] < m_d_star) {
        x_step = m_xi * d_goal[0] * cos(d_goal[1]);
        y_step = m_xi * d_goal[0] * sin(d_goal[1]);
    } else {
        x_step = (m_d_star / d_goal[0]) * m_xi * cos(d_goal[1]);
        y_step = (m_d_star / d_goal[0]) * m_xi * sin(d_goal[1]);
    }

    return Eigen::Vector2d(x_step,y_step);
}

bool amp::MyGDAlgo::primitiveCheck(Eigen::Vector2d point, double slope, Eigen::Vector2d intercept, int gt) {
    return ( ((gt % 2 == 1) ? -1 : 1) * ( (point[1] - intercept[1]) - slope * (point[0] - intercept[0]) ) <= 0 );
}

double amp::MyGDAlgo::distance2facet(Eigen::Vector2d point, double slope, Eigen::Vector2d intercept) {
    double a = 1;
    double b = -slope;
    double c = -intercept[1] + slope*intercept[0];
    return ( std::abs(a*point[0] + b*point[1] + c) / sqrt(a*a + b*b) );
}

Eigen::Vector2d amp::MyGDAlgo::nearFacet(Eigen::Vector2d point, Eigen::Vector2d p1, Eigen::Vector2d p2) {
    double dy, dx;
    double angle, slope, inv_slope;

    dy = p2[1] - p1[1];
    dx = p2[0] - p1[0];

    angle = atan2(dy,dx);

    slope = dy/dx;
    inv_slope = -1/slope;

    Eigen::Vector2d dist2facet = Eigen::Vector2d(-1,-1);

    if (dx > 0) {
        if (primitiveCheck(point, slope, p1, 0)) {
            if (angle > 0) {
                if (primitiveCheck(point, inv_slope, p1, 1) && primitiveCheck(point, inv_slope, p2, 0)) {
                    return Eigen::Vector2d(distance2facet(point, slope, p1), atan2(-dx,dy));
                }
            } else if (angle < 0) {
                if (primitiveCheck(point, inv_slope, p1, 0) && primitiveCheck(point, inv_slope, p2, 1)) {
                    return Eigen::Vector2d(distance2facet(point, slope, p1), atan2(dx,-dy));
                }
            } else {
                if (point[0] > p1[0] && point[0] < p2[0] && point[1] < p1[1]) {
                    return Eigen::Vector2d(p1[1]-point[1], -M_PI/2);
                }
            }
        }
    } else if (dx < 0) {
        if (primitiveCheck(point, slope, p1, 1)) {
            if (angle < M_PI && angle > 0) {
                if (primitiveCheck(point, inv_slope, p1, 1) && primitiveCheck(point, inv_slope, p2, 0)) {
                    return Eigen::Vector2d(distance2facet(point, slope, p1), atan2(-dx,dy));
                }
            } else if (angle > -M_PI && angle < 0) {
                if (primitiveCheck(point, inv_slope, p1, 0) && primitiveCheck(point, inv_slope, p2, 1)) {
                    return Eigen::Vector2d(distance2facet(point, slope, p1), atan2(dx,-dy));
                }
            } else {
                if (point[0] < p1[0] && point[0] > p2[0] && point[1] > p1[1]) {
                    return Eigen::Vector2d(point[1]-p1[1], M_PI/2);
                }
            }
        }
    } else {
        if (dy > 0) {
            if (point[1] > p1[1] && point[1] < p2[1] && point[0] > p1[0]) {
                return Eigen::Vector2d(point[0]-p1[0], 0);
            }
        } else if (dy < 0) {
            if (point[1] < p1[1] && point[1] > p2[1] && point[0] < p1[0]) {
                return Eigen::Vector2d(p1[0]-point[0], M_PI);
            }
        }
    }
    return dist2facet;
}

std::vector<Eigen::Vector2d> amp::MyGDAlgo::distance2obs(const amp::Problem2D& problem, Eigen::Vector2d point) {

    std::vector<Eigen::Vector2d> d_obs;

    int num_obstacles = problem.obstacles.size();
    int num_vertices;

    Eigen::Vector2d p1, p2;
    Eigen::Vector2d dist2obs, dist2facet, dist2vertex;

    for (int i = 0; i < num_obstacles; i++) {
        num_vertices =  problem.obstacles[i].verticesCCW().size();
        p1 = Eigen::Vector2d(problem.obstacles[i].verticesCCW().back()[0],problem.obstacles[i].verticesCCW().back()[1]);
        dist2facet[0] = -1;
        dist2vertex = distance2point(p1, point);
        for (int j = 0; j < num_vertices; j++) {
            if (j > 0) {
                p1 = Eigen::Vector2d(problem.obstacles[i].verticesCCW()[j-1][0],problem.obstacles[i].verticesCCW()[j-1][1]);
                p2 = Eigen::Vector2d(problem.obstacles[i].verticesCCW()[j][0],problem.obstacles[i].verticesCCW()[j][1]);
            } else {
                p2 = Eigen::Vector2d(problem.obstacles[i].verticesCCW()[0][0],problem.obstacles[i].verticesCCW()[0][1]);
            }
            dist2facet = nearFacet(point,p1,p2);
            if (dist2facet[0] != -1) {
                dist2obs = dist2facet;
                break;
            }
            if (distance2point(p1, point)[0] < dist2vertex[0]) {
                dist2vertex = distance2point(p1, point);
            }
        }
        if (dist2facet[0] == -1) dist2obs = dist2vertex;
        d_obs.push_back(dist2obs);
    }
    return d_obs;
}

Eigen::Vector2d amp::MyGDAlgo::Grad_U_rep(const amp::Problem2D& problem, Eigen::Vector2d point) {
    std::vector<Eigen::Vector2d> d_obs = distance2obs(problem, point);

    double x_step = 0;
    double y_step = 0;

    for (int i = 0; i < d_obs.size(); i++) {
        if (d_obs[i][0] < m_Q_star) {
            // std::cout << "d_obs, angle: " << d_obs[i][0] << "," << d_obs[i][1] << std::endl;
            x_step += -m_eta * (1/m_Q_star - 1/d_obs[i][0]) / d_obs[i][0] / d_obs[i][0] * cos(d_obs[i][1]);
            y_step += -m_eta * (1/m_Q_star - 1/d_obs[i][0]) / d_obs[i][0] / d_obs[i][0] * sin(d_obs[i][1]);
        } else {
            x_step += 0;
            y_step += 0;
        }
    }

    return Eigen::Vector2d(x_step, y_step);
}

amp::Path2D amp::MyGDAlgo::plan(const amp::Problem2D& problem) {

    srand((unsigned) time(NULL));

    amp::Path2D myPlan;

    // Define start and end points
    Eigen::Vector2d start = Eigen::Vector2d(problem.q_init[0], problem.q_init[1]);
    Eigen::Vector2d goal  = Eigen::Vector2d(problem.q_goal[0], problem.q_goal[1]);

    myPlan.waypoints.push_back(start);
    
    double delta = 0.25;
    int iter = 0;

    double x_new, y_new;
    Eigen::Vector2d point_curr, att, rep, pos_new;
    
    double dist2goal = distance2point(start, goal)[0];

    while (dist2goal > delta && iter < 2000) {

        point_curr = myPlan.waypoints.back();

        att = Grad_U_att(problem, point_curr);
        rep = Grad_U_rep(problem, point_curr);

        Eigen::Vector2d tot = Eigen::Vector2d(att[0]+rep[0],att[1]+rep[1]);

        double mag = sqrt(tot[0]*tot[0] + tot[1]*tot[1]);
        tot = Eigen::Vector2d(tot[0]/mag,tot[1]/mag);

        double step_size = 0.01;
        pos_new = point_curr + tot*step_size;

        if ((myPlan.waypoints[myPlan.waypoints.size()-2]-pos_new).norm() == 0) {
            double rand1 = (double(rand() % 200) - 100) / 1000;
            double rand2 = (double(rand() % 200) - 100) / 1000;
            pos_new = point_curr + Eigen::Vector2d(rand1,rand2);
        }

        myPlan.waypoints.push_back(pos_new);

        iter++;
        dist2goal = distance2point(myPlan.waypoints.back(), goal)[0];
    }

    myPlan.waypoints.push_back(problem.q_goal);

    return myPlan;
}