#include "MyAStarAlgo.h"
#include "MyDecentralizedMultiAgentRRT.h"

#include <vector>
#include <Eigen/Core>

amp::MultiAgentPath2D amp::MyDecentralizedMultiAgentRRT::plan(const amp::MultiAgentProblem2D& problem) {

    prob_       = problem;
    num_agents_ = problem.numAgents();

    amp::MultiAgentPath2D path(num_agents_);

    amp::MyAStarAlgo aStar;
    amp::AStar::GraphSearchResult searchResult;

    double x1,y1,x2,y2,dx,dy,dist;
    double closest_node, goal_dist;
    bool goal_found = false;
    Eigen::Vector2d q_near, q_rand, q_new, q_goal;
    amp::LookupSearchHeuristic heuristic;
    std::map<amp::Node, Eigen::Vector2d> map;

    amp::ShortestPathProblem pathProblem;
    pathProblem.graph = std::make_shared<Graph<double>>();

    double x_min = problem.x_min;
    double x_max = problem.x_max;
    double y_min = problem.y_min;
    double y_max = problem.y_max;

    int iter, ind;

    int mod_num = round(1.0 / bias_);

    for (int i = 0; i < num_agents_; i++) {
        map.clear();
        pathProblem.graph->clear();

        // create tree rooted at problem.q_init
        q_near = problem.agent_properties[i].q_init;
        q_goal = problem.agent_properties[i].q_goal;

        map.insert({0, q_near});
        heuristic.heuristic_values.insert({0,(q_near-q_goal).norm()});

        root_dist_.insert({0,0});

        iter = 0;
        ind = 0; 

        goal_found = false;

        while(!goal_found && iter < n_) {

            iter++;
            ind++;

            if (iter % mod_num != 0) {
                x1 = amp::RNG::randd(x_min,x_max);
                y1 = amp::RNG::randd(y_min,y_max);
                q_rand = Eigen::Vector2d(x1,y1);
            } else {
                q_rand = q_goal;
            }

            closest_node = 0;
            dist = (map.at(0) - q_rand).norm();

            for (int i = 1; i < map.size(); i++) { 
                if ((map.at(i) - q_rand).norm() < dist) {
                    closest_node = i;
                    dist = (map.at(i) - q_rand).norm();
                }
            }

            if (r_ - dist < 0) {
                q_new = map.at(closest_node) + (q_rand - map.at(closest_node)) / dist * r_;
            } else {
                q_new = q_rand;
            }

            if (!inCollision(q_new, path, i, closest_node)) {
                map.insert({ind, q_new});
                root_dist_.insert({ind,root_dist_.at(closest_node)+1});
                pathProblem.graph->connect(closest_node, ind, r_);
                heuristic.heuristic_values.insert({ind, (q_new-q_goal).norm()});
            } else {
                ind--;
                continue;
            }

            goal_dist = (q_new - q_goal).norm();
            if (goal_dist < epsilon_) {
                goal_found = true;
                ind++;
                map.insert({ind,q_goal});
                pathProblem.graph->connect(ind-1, ind, goal_dist);
                heuristic.heuristic_values.insert({ind, goal_dist});
            }
            // if (iter % 10 == 0) {
            //     amp::Visualizer::makeFigure(problem, *pathProblem.graph, [map](amp::Node node) -> Eigen::Vector2d { return map.at(node); });
            //     amp::Visualizer::showFigures();
            // }
        }

        tree_size_ += ind;

        if (goal_found) {
            pathProblem.init_node = 0;
            pathProblem.goal_node = map.size()-1;
            searchResult = aStar.search(pathProblem, heuristic);

            for (const auto& element : searchResult.node_path) {
                path.agent_paths[i].waypoints.push_back(map.at(element));
            }
        } else {
            return path;
        }
    }

    return path;
}

bool amp::MyDecentralizedMultiAgentRRT::inCollision(Eigen::Vector2d state, amp::MultiAgentPath2D path, int i, double closest_ind) {
    double dist2obs, dist2agent;
    double r_i, r_k;

    Eigen::Vector2d temp_point;

    for (int j = 0; j < i+1; j++) {

        dist2obs = distance2obs(prob_, state);
        r_i = prob_.agent_properties[i].radius;

        // LOG("agent " << i << " dist2obs: " << dist2obs);

        if (dist2obs < r_i * 1.2) return true;
        if (inPolygon(state[0],state[1])) return true;
        for (int k = 0; k < i; k++) {
            temp_point = path.agent_paths[k].waypoints[root_dist_.at(closest_ind)+1];
            dist2agent = (state-temp_point).norm();
            r_k = prob_.agent_properties[k].radius;
            if (dist2agent < (r_i + r_k) * 1.2) return true;

            // LOG("agent " << i << " pos: " << state(2*i) << ", " << state(2*i+1));
            // LOG("agent " << j << " pos: " << state(2*j) << ", " << state(2*j+1));
            // LOG("dist between " << i << " and " << j << ": " << dist2agent);
        }
    }

    // amp::Visualizer::makeFigure(amp::HW8::getWorkspace1(),path);
    // amp::Visualizer::showFigures();

    return false;
}

bool amp::MyDecentralizedMultiAgentRRT::inPolygon(double x_pos, double y_pos) const {
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

bool amp::MyDecentralizedMultiAgentRRT::lineIntersect(Eigen::Vector2d p1, Eigen::Vector2d p2, amp::MultiAgentProblem2D prob) {
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

Eigen::Vector2d amp::MyDecentralizedMultiAgentRRT::distance2point(Eigen::Vector2d p1, Eigen::Vector2d p2) {
    double dy = p2[1] - p1[1];
    double dx = p2[0] - p1[0];

    double dist = sqrt( dx*dx + dy*dy );
    double angle = atan2(dy,dx);
    
    return Eigen::Vector2d(dist, angle);
}

bool amp::MyDecentralizedMultiAgentRRT::primitiveCheck(Eigen::Vector2d point, double slope, Eigen::Vector2d intercept, int gt) {
    return ( ((gt % 2 == 1) ? -1 : 1) * ( (point[1] - intercept[1]) - slope * (point[0] - intercept[0]) ) <= 0 );
}

double amp::MyDecentralizedMultiAgentRRT::distance2facet(Eigen::Vector2d point, double slope, Eigen::Vector2d intercept) {
    double a = 1;
    double b = -slope;
    double c = -intercept[1] + slope*intercept[0];
    return ( std::abs(a*point[0] + b*point[1] + c) / sqrt(a*a + b*b) );
}

Eigen::Vector2d amp::MyDecentralizedMultiAgentRRT::nearFacet(Eigen::Vector2d point, Eigen::Vector2d p1, Eigen::Vector2d p2) {
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

double amp::MyDecentralizedMultiAgentRRT::distance2obs(const amp::MultiAgentProblem2D& problem, Eigen::Vector2d point) {

    double d_obs = 1e9;

    int num_obstacles = problem.obstacles.size();
    int num_vertices;

    Eigen::Vector2d p1, p2;
    double dist2obs, dist2facet, dist2vertex;

    // LOG("point: " << point[0] << ", " << point[1]);

    for (int i = 0; i < num_obstacles; i++) {
        num_vertices = problem.obstacles[i].verticesCCW().size();

        p1 = Eigen::Vector2d(problem.obstacles[i].verticesCCW().back()[0],problem.obstacles[i].verticesCCW().back()[1]);
        dist2facet = -1;
        dist2vertex = distance2point(p1, point)[0];
        for (int j = 0; j < num_vertices; j++) {
            if (j > 0) {
                p1 = Eigen::Vector2d(problem.obstacles[i].verticesCCW()[j-1][0],problem.obstacles[i].verticesCCW()[j-1][1]);
                p2 = Eigen::Vector2d(problem.obstacles[i].verticesCCW()[j][0],problem.obstacles[i].verticesCCW()[j][1]);
            } else {
                p2 = Eigen::Vector2d(problem.obstacles[i].verticesCCW()[0][0],problem.obstacles[i].verticesCCW()[0][1]);
            }

            // LOG("p1: " << p1[0] << ", " << p1[1]);
            // LOG("p2: " << p2[0] << ", " << p2[1]);
            // PAUSE;

            dist2facet = nearFacet(point,p1,p2)[0];
            if (dist2facet != -1) {
                dist2obs = dist2facet;
                break;
            }
            if (distance2point(p1, point)[0] < dist2vertex) {
                dist2vertex = distance2point(p1, point)[0];
            }
        }
        if (dist2facet == -1) dist2obs = dist2vertex;
        if (dist2obs < d_obs) d_obs = dist2obs;
    }
    return d_obs;
}