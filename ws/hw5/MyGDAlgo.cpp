#include "MyGDAlgo.h"
#include <thread>

// MyGDAlgo(double d_star, double Q_star) {
//     double m_d_star = d_star;
//     double m_Q_star = Q_star;
// }

Eigen::Vector2d amp::MyGDAlgo::distance2point(Eigen::Vector2d point1, Eigen::Vector2d point2) {
    double x1 = point1[0];
    double y1 = point1[1];
    double x2 = point2[0];
    double y2 = point2[1];
    
    return Eigen::Vector2d(sqrt( (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) ), atan2(y2-y1,x2-x1));
}

Eigen::Vector2d amp::MyGDAlgo::Grad_U_att(Eigen::Vector2d d_goal) {
    return Eigen::Vector2d(0.0,0.0);
}

bool amp::MyGDAlgo::primitiveCheck(Eigen::Vector2d point, double slope, Eigen::Vector2d intercept, int gt) {
    return ((gt % 2 == 1) ? -1 : 1) * ( (point[1] - intercept[1]) - slope * (point[0] - intercept[0]) ) <= 0;
}

double amp::MyGDAlgo::distance2facet(Eigen::Vector2d point, double slope, Eigen::Vector2d intercept) {
    double a = 1;
    double b = -slope;
    double c = -intercept[1] + slope*intercept[0];
    return std::abs(a*point[0] + b*point[1] + c) / sqrt(a*a + b*b);
}

double amp::MyGDAlgo::nearFacet(Eigen::Vector2d point, Eigen::Vector2d p1, Eigen::Vector2d p2) {
    double x_curr, y_curr, x_next, y_next;
    double angle, slope, inv_slope;

    x_curr = p1[0];
    y_curr = p1[1];

    x_next = p2[0];
    y_next = p2[1];

    angle = atan2(y_next-y_curr,x_next-x_curr);
    slope = (y_next-y_curr) / (x_next-x_curr);
    inv_slope = -1/slope;
    if (x_next > x_curr) {
        if (primitiveCheck(point, slope, p1, 0)) {
            if (angle > 0) {
                if (primitiveCheck(point, inv_slope, p1, 1) && primitiveCheck(point, inv_slope, p2, 0)) {
                    return distance2facet(point, slope, p1);
                }
            } else if (angle <= 0) {
                if (primitiveCheck(point, inv_slope, p1, 0) && primitiveCheck(point, inv_slope, p2, 1)) {
                    return distance2facet(point, slope, p1);
                }
            }
        }
    } else if (x_next < x_curr) {
        if (primitiveCheck(point, slope, p1, 1)) {
            if (angle > 0) {
                if (primitiveCheck(point, inv_slope, p1, 1) && primitiveCheck(point, inv_slope, p2, 0)) {
                    return distance2facet(point, slope, p1);
                }
            } else if (angle <= 0) {
                if (primitiveCheck(point, inv_slope, p1, 0) && primitiveCheck(point, inv_slope, p2, 1)) {
                    return distance2facet(point, slope, p1);
                }
            }
        }
    }
    return 0;
}

std::vector<Eigen::Vector2d> amp::MyGDAlgo::distance2obs(const amp::Problem2D& problem, Eigen::Vector2d point) {

    std::vector<Eigen::Vector2d> d_obs;

    double x = point[0];
    double y = point[1];

    int num_obstacles = problem.obstacles.size();
    int num_vertices;

    Eigen::Vector2d p1, p2;

    double dist2obs, dist2facet, dist2vertex;
    double x_curr, y_curr, x_next, y_next;
    double angle, slope, inv_slope;

    for (int i = 0; i < num_obstacles; i++) {
        num_vertices =  problem.obstacles[i].verticesCCW().size();
        p1 = Eigen::Vector2d(problem.obstacles[i].verticesCCW().back()[0],problem.obstacles[i].verticesCCW().back()[1]);
        dist2facet = 0;
        dist2vertex = distance2point(point, p1)[0];
        for (int j = 0; j < num_vertices; j++) {
            if (j > 0) {
                p1 = Eigen::Vector2d(problem.obstacles[i].verticesCCW()[j][0],problem.obstacles[i].verticesCCW()[j][1]);
                p2 = Eigen::Vector2d(problem.obstacles[i].verticesCCW()[j+1][0],problem.obstacles[i].verticesCCW()[j+1][1]);
            } else {
                p2 = Eigen::Vector2d(problem.obstacles[i].verticesCCW()[0][0],problem.obstacles[i].verticesCCW()[0][1]);
            }
            dist2facet = nearFacet(point,p1,p2);
            if (dist2facet != 0) {
                dist2obs = dist2facet;
                break;
            }
            if (distance2point(point, p1)[0] < dist2vertex) {
                dist2vertex = distance2point(point, p1)[0];
            }
        }
        if (dist2facet == 0) dist2obs = dist2vertex;
        d_obs.push_back(Eigen::Vector2d(dist2obs,0));
    }
    return d_obs;
}

Eigen::Vector2d amp::MyGDAlgo::Grad_U_rep(std::vector<Eigen::Vector2d> d_obs) {
    return Eigen::Vector2d(0.0,0.0);
}

amp::Path2D amp::MyGDAlgo::plan(const amp::Problem2D& problem) {

    amp::Path2D myPlan;

    // Define start and end points
    Eigen::Vector2d start = Eigen::Vector2d(problem.q_init[0], problem.q_init[1]);
    Eigen::Vector2d goal  = Eigen::Vector2d(problem.q_goal[0], problem.q_goal[1]);

    myPlan.waypoints.push_back(start);
    
    double delta = 1e-2;
    int iter = 0;
    
    double dist2goal = distance2point(start, goal)[0];

    while (dist2goal > delta && iter < 1000) {






        iter++;
        dist2goal = distance2point(myPlan.waypoints.back(), goal)[0];
    }

    return myPlan;
}