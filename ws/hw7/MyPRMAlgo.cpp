#include "MyAStarAlgo.h"
#include "MyPRMAlgo.h"

#include <vector>
#include <map>
#include <Eigen/Core>

amp::Path amp::GenericPRM::plan_nd(const Eigen::VectorXd& init_state, 
                                   const Eigen::VectorXd& goal_state/*, 
                                   const amp::CollisionSpace& collision_checker*/) 
{
    // Implement the sampling-based planner using
    // only these arguments and other hyper parameters ...
    return amp::Path();
}

amp::Path2D amp::MyPRM2D::plan(const amp::Problem2D& problem) {
    // Make a collsion checker object
    // MyPointCollisionChecker cspace(problem);

    // Call the generic planner
    amp::Path2D path_nd = plan_2D(problem);

    // Convert the ND path to a 2D path and return it...
    // return path_2d;
    return path_nd;
}

amp::Path2D amp::MyPRM2D::plan_2D(const amp::Problem2D& problem) {
    prob_ = problem;

    double x1,y1,x2,y2,dx,dy,dist;
    std::map<amp::Node, Eigen::Vector2d> map;
    std::vector<Eigen::Vector2d> sampled_points;
    amp::LookupSearchHeuristic heuristic;

    //added
    amp::ShortestPathProblem pathProblem;
    pathProblem.graph = std::make_shared<Graph<double>>();

    sampled_points.push_back(problem.q_init);
    sampled_points.push_back(problem.q_goal);

    map.insert({0,problem.q_init});
    map.insert({1,problem.q_goal});

    // amp::Graph<double> graph;

    double x_min = problem.x_min;
    double x_max = problem.x_max;
    double y_min = problem.y_min;
    double y_max = problem.y_max;

    int ind = 2;

    for (int i = 0; i < n_+2; i++) {
        x1 = amp::RNG::randd(x_min,x_max);
        y1 = amp::RNG::randd(y_min,y_max);
        if(!inPolygon(x1, y1)) sampled_points.push_back(Eigen::Vector2d({x1,y1}));
        if(!inPolygon(x1, y1)) {
            map.insert({ind,Eigen::Vector2d({x1,y1})});
            ind++;
        }
    }

    for (int i = 0; i < map.size(); i++) {
        
        x1 = sampled_points[i][0];
        y1 = sampled_points[i][1];

        x1 = map.at(i)[0];
        y1 = map.at(i)[1];
        dx = problem.q_goal[0] - x1;
        dy = problem.q_goal[1] - y1;
        dist = sqrt((dx*dx)+(dy*dy));

        heuristic.heuristic_values.insert(std::pair<Node, double>(i, dist));

        for (int j = i+1; j < sampled_points.size(); j++) {
            x2 = sampled_points[j][0];
            y2 = sampled_points[j][1];

            x2 = map.at(j)[0];
            y2 = map.at(j)[1];

            dx = x2 - x1;
            dy = y2 - y1;
            dist = sqrt((dx*dx)+(dy*dy));
            // if distance is less than threshold
            if (dist < r_) {
                // if connection isnt line intersecting
                if (!lineIntersect(x1,y1,x2,y2,problem)) {
                    // graph.connect(i, j, dist);
                    // graph.connect(j, i, dist);
                    
                    pathProblem.graph->connect(i, j, dist);
                    pathProblem.graph->connect(j, i, dist);
                }
            }
        }
    }

    map_ = map;
    graph_ = *pathProblem.graph;

    amp::MyAStarAlgo aStar;
	pathProblem.init_node = 0;
	pathProblem.goal_node = 1;
    amp::AStar::GraphSearchResult searchResult = aStar.search(pathProblem, amp::SearchHeuristic());

    amp::Path2D path;

    for (const auto& element : searchResult.node_path) {
        path.waypoints.push_back(sampled_points[element]);
    }
    
    return path;
}

bool amp::MyPRM2D::inPolygon(double x_pos, double y_pos) const {
    // Define ints to hold values for the number of obstacles in the environment and the number of 
    // vertices for each of those obstacles
    int num_obstacles = prob_.obstacles.size();
    int num_vertices;

    bool inside = false;

    double x1, y1, x2, y2;

    for (int i = 0; i < num_obstacles; i++) {
        num_vertices = prob_.obstacles[i].verticesCCW().size();
        for (int j = 0; j < num_vertices; j++) {
            x1 = prob_.obstacles[i].verticesCCW()[j][0];
            y1 = prob_.obstacles[i].verticesCCW()[j][1];
            x2 = prob_.obstacles[i].verticesCCW()[j+1][0];
            y2 = prob_.obstacles[i].verticesCCW()[j+1][1];
            if (j == num_vertices-1) {
                x2 = prob_.obstacles[i].verticesCCW()[0][0];
                y2 = prob_.obstacles[i].verticesCCW()[0][1];
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

bool amp::MyPRM2D::lineIntersect(double x1, double y1, double x2, double y2, amp::Problem2D prob) {
    double t, u;
    double x3, y3, x4, y4;

    double slope1, slope2;
    double x1_y3, x1_y4, x2_y3, x2_y4;

    int num_obstacles = prob.obstacles.size();
    int num_vertices;

    for (int i = 0; i < num_obstacles; i++) {
        num_vertices = prob.obstacles[i].verticesCCW().size();
        for (int j = 0; j < num_vertices; j++) {
            x3 = prob.obstacles[i].verticesCCW()[j][0];
            y3 = prob.obstacles[i].verticesCCW()[j][1];
            x4 = prob.obstacles[i].verticesCCW()[j+1][0];
            y4 = prob.obstacles[i].verticesCCW()[j+1][1];
            if (j == num_vertices-1) {
                x4 = prob.obstacles[i].verticesCCW()[0][0];
                y4 = prob.obstacles[i].verticesCCW()[0][1];
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