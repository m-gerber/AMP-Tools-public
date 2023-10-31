#include "MyAStarAlgo.h"
#include "MyGoalBiasRRT2D.h"

#include <vector>
#include <Eigen/Core>

amp::Path2D amp::MyGoalBiasRRT2D::plan(const amp::Problem2D& problem) {
    
    prob_ = problem;

    double x1,y1,x2,y2,dx,dy,dist;
    Eigen::Vector2d q_near, q_rand, q_new, q_goal;
    amp::LookupSearchHeuristic heuristic;
    std::map<amp::Node, Eigen::Vector2d> map;

    amp::ShortestPathProblem pathProblem;
    pathProblem.graph = std::make_shared<Graph<double>>();

    // create tree rooted at problem.q_init
    q_near = problem.q_init;
    map.insert({0, q_near});
    heuristic.heuristic_values.insert({0,distBetween(q_near,q_goal)});

    q_goal = problem.q_goal;

    // LOG("q_init: " << q_near[0] << ", " << q_near[1]);

    double x_min = problem.x_min;
    double x_max = problem.x_max;
    double y_min = problem.y_min;
    double y_max = problem.y_max;

    double closest_node;
    double angle2rand;
    bool goal_found = false;

    int iter = 0;
    int ind  = 0;

    int mod_num = round(1.0 / bias_);

    double min_dist = distBetween(q_near,q_goal);
  
    // while solution not found or below max iterations (n_)...
    while(!goal_found && iter < n_) {

        iter++;
        ind++;

        if (iter % mod_num != 0) {
            x1 = amp::RNG::randd(x_min,x_max);
            y1 = amp::RNG::randd(y_min,y_max);
            if (inPolygon(x1, y1)) {
                ind--;
                continue;
            }
            q_rand = Eigen::Vector2d(x1,y1);
        } else {
            q_rand = problem.q_goal;
        }

        // LOG("q_rand: " << q_rand[0] << ", " << q_rand[1]);

        closest_node = 0;
        dist = distBetween(map.at(0), q_rand);

        for (int i = 1; i < map.size(); i++) { 
            if (distBetween(map.at(i), q_rand) < dist) {
                closest_node = i;
                dist = distBetween(map.at(i), q_rand);
            }
        }

        // LOG("closest point: " << map.at(closest_node)[0] << ", " << map.at(closest_node)[1]);
        // LOG("closest dist node " << closest_node << ": " << dist);

        if (r_ - dist < 0) {
            angle2rand = atan2(q_rand[1] - map.at(closest_node)[1], q_rand[0] - map.at(closest_node)[0]);
            q_new = Eigen::Vector2d(map.at(closest_node)[0]+r_*cos(angle2rand),map.at(closest_node)[1]+r_*sin(angle2rand));
        } else {
            q_new = q_rand;
        }

        if (!lineIntersect(map.at(closest_node), q_new, problem)) {
            map.insert({ind, q_new});
            pathProblem.graph->connect(closest_node, ind, r_);
            heuristic.heuristic_values.insert({ind, distBetween(q_new,q_goal)});
        } else {
            ind--;
            continue;
        }

        

        if (distBetween(q_new, q_goal) < min_dist) min_dist = distBetween(q_new, q_goal);

        if (distBetween(q_new, q_goal) < epsilon_) {
            goal_found = true;
            ind++;
            map.insert({ind,q_goal});
            pathProblem.graph->connect(ind-1, ind, distBetween(q_new, q_goal));
            heuristic.heuristic_values.insert({ind, distBetween(q_new,q_goal)});
        }
        // LOG("q_new " << ind << ": " << q_new[0] << ", " << q_new[1]);
        // LOG("q_goal: " << q_goal[0] << ", " << q_goal[1]);
        // LOG("dist2goal: " << distBetween(q_new, q_goal));
        // LOG("");
        // LOG("");
        // pathProblem.graph->print();
        // if (iter % 10 == 0) {
        //     amp::Visualizer::makeFigure(problem, *pathProblem.graph, [map](amp::Node node) -> Eigen::Vector2d { return map.at(node); });
        //     amp::Visualizer::showFigures();
        // }
    }



    // LOG("min dist: " << min_dist);

    // pathProblem.graph->print();

    if (goal_found) {
        graph_ = *pathProblem.graph;
        map_ = map;

        amp::MyAStarAlgo aStar;
        pathProblem.init_node = 0;
        pathProblem.goal_node = map.size()-1;
        amp::AStar::GraphSearchResult searchResult = aStar.search(pathProblem, heuristic);

        amp::Path2D path;

        for (const auto& element : searchResult.node_path) {
            path.waypoints.push_back(map.at(element));
        }

        // pathProblem.graph = nullptr;
        // map.clear();

        return path;
    }

    // pathProblem.graph = nullptr;
    // map.clear();

    return amp::Path2D();
}

double amp::MyGoalBiasRRT2D::distBetween(Eigen::Vector2d p1, Eigen::Vector2d p2) {
    return sqrt( (p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]) );
}

bool amp::MyGoalBiasRRT2D::inPolygon(double x_pos, double y_pos) const {
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

bool amp::MyGoalBiasRRT2D::lineIntersect(Eigen::Vector2d p1, Eigen::Vector2d p2, amp::Problem2D prob) {
    double x1 = p1[0];
    double y1 = p1[1];
    double x2 = p2[0];
    double y2 = p2[1];

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