#include "MyGDAlgo.h"
#include <thread>

// MyGDAlgo(double d_star, double Q_star) {
//     double m_d_star = d_star;
//     double m_Q_star = Q_star;
// }

double distance2point(Eigen::Vector2d point1, Eigen::Vector2d point2) {
    double x1 = point1[0];
    double y1 = point1[1];
    double x2 = point2[0];
    double y2 = point2[1];
    
    return sqrt( (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) );
}

double U_att(const amp::Problem2D& problem, double d_goal) {

}

bool nearFacet(Eigen::Vector2d point, Eigen::Vector2d p1, Eigen::Vector2d p2) {
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
        // want to use less than
        if (angle > 0) {
            // want to use greater than x_curr and less than x_next
        } else if (angle <= 0) {
            // want to use less than x_curr and greater than x_next
        }
    } else if (x_next < x_curr) {
        // want to use greater than
    }
}

std::vector<double> distance2obs(const amp::Problem2D& problem, Eigen::Vector2d point) {

    std::vector<double> d_obs;

    double x = point[0];
    double y = point[1];

    int num_obstacles = problem.obstacles.size();
    int num_vertices;

    Eigen::Vector2d p1, p2;

    double dist_x, dist_y, dist2obs;
    double x_curr, y_curr, x_next, y_next;
    double angle, slope, inv_slope;

    double dist2facet = 0;

    for (int i = 0; i < num_obstacles; i++) {
        num_vertices =  problem.obstacles[i].verticesCCW().size();
        p1 = Eigen::Vector2d(problem.obstacles[i].verticesCCW().back()[0],problem.obstacles[i].verticesCCW().back()[1]);
        for (int j = 0; j < num_vertices; j++) {
            if (j > 0) {
                p1 = Eigen::Vector2d(problem.obstacles[i].verticesCCW()[j][0],problem.obstacles[i].verticesCCW()[j][1]);
                p2 = Eigen::Vector2d(problem.obstacles[i].verticesCCW()[j+1][0],problem.obstacles[i].verticesCCW()[j+1][1]);
            } else {
                p2 = Eigen::Vector2d(problem.obstacles[i].verticesCCW()[0][0],problem.obstacles[i].verticesCCW()[0][1]);
            }
            dist2facet = nearFacet(point,p1,p2);
            if (dist2facet != 0) {
                // find distance to facet
                break;
            }
        }

        d_obs.push_back(dist2obs);
    }
    return d_obs;
}

double U_rep(const amp::Problem2D& problem, std::vector<double> d_obs) {

}

amp::Path2D plan(const amp::Problem2D& problem) {

    amp::Path2D myPlan;

    // Define start and end points
    Eigen::Vector2d start = Eigen::Vector2d(problem.q_init[0], problem.q_init[1]);
    Eigen::Vector2d goal  = Eigen::Vector2d(problem.q_goal[0], problem.q_goal[1]);

    myPlan.waypoints.push_back(start);
    
    double delta = 1e-2;
    int iter = 0;
    
    double dist2goal = distance2point(start, goal);

    while (dist2goal > delta && iter < 1000) {






        iter++;
        dist2goal = distance2point(myPlan.waypoints.back(), goal);
    }

    return myPlan;
}