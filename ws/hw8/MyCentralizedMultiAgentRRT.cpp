#include "MyAStarAlgo.h"
#include "MyCentralizedMultiAgentRRT.h"

#include <vector>
#include <Eigen/Core>

amp::MultiAgentPath2D amp::MyCentralizedMultiAgentRRT::plan(const amp::MultiAgentProblem2D& problem) {
        
    prob_ = problem;
    num_agents_ = problem.numAgents();

    double x1,y1,x2,y2,dx,dy,dist;
    Eigen::VectorXd q_near(2*num_agents_);
    Eigen::VectorXd q_rand(2*num_agents_);
    Eigen::VectorXd  q_new(2*num_agents_);
    Eigen::VectorXd q_goal(2*num_agents_);
    Eigen::VectorXd unit_step(2*num_agents_);

    amp::LookupSearchHeuristic heuristic;
    std::map<amp::Node, Eigen::VectorXd> map;

    amp::ShortestPathProblem pathProblem;
    pathProblem.graph = std::make_shared<Graph<double>>();

    // create tree rooted at problem.q_init
    for (int i = 0; i < num_agents_; i++) {
        q_near(2*i)   = problem.agent_properties[i].q_init[0]; 
        q_near(2*i+1) = problem.agent_properties[i].q_init[1]; 
        q_goal(2*i)   = problem.agent_properties[i].q_goal[0]; 
        q_goal(2*i+1) = problem.agent_properties[i].q_goal[1]; 
    }

    map.insert({0, q_near});
    heuristic.heuristic_values.insert({0,distBetween(q_near,q_goal)});

    double x_min = problem.x_min;
    double x_max = problem.x_max;
    double y_min = problem.y_min;
    double y_max = problem.y_max;

    double closest_node;
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
            for (int i = 0; i < num_agents_; i++) {
                q_rand(2*i)   = amp::RNG::randd(x_min,x_max);
                q_rand(2*i+1) = amp::RNG::randd(y_min,y_max);
            }
        } else {
            q_rand = q_goal;
        }

        closest_node = 0;
        dist = distBetween(map.at(0), q_rand);

        for (int i = 1; i < map.size(); i++) { 
            if (distBetween(map.at(i), q_rand) < dist) {
                closest_node = i;
                dist = distBetween(map.at(i), q_rand);
            }
        }

        q_new = map.at(closest_node) + (q_rand - map.at(closest_node)) / dist * r_;
        unit_step = (q_rand - map.at(closest_node)) / dist;

        // for (int i = 0; i < num_agents_; i++) {
        //     double temp_dist = distBetween(Eigen::Vector2d({q_rand(2*i),q_rand(2*i+1)}), Eigen::Vector2d({map.at(closest_node)(2*i),map.at(closest_node)(2*i+1)}));
        //     // LOG("temp_dist: " << temp_dist);
        //     q_new(2*i)   = map.at(closest_node)(2*i) + (q_rand(2*i) - map.at(closest_node)(2*i)) / temp_dist * r_;
        //     q_new(2*i+1) = map.at(closest_node)(2*i+1) + (q_rand(2*i+1) - map.at(closest_node)(2*i+1)) / temp_dist * r_;
        //     // LOG("d1: " << (q_rand(2*i) - map.at(closest_node)(2*i)) / temp_dist * r_);
        //     // LOG("d2: " << (q_rand(2*i+1) - map.at(closest_node)(2*i+1)) / temp_dist * r_);
        // }

        // LOG("closest node: " << map.at(closest_node));
        // LOG("q_rand: " << q_rand);
        // LOG("q_new: " << q_new);
        // PAUSE;

        double disc = 1;

        for (int i = 0; i < disc+1; i++) {
            if (inCollision(map.at(closest_node) + i * unit_step * r_ / disc)) {
                ind--;
                continue;
            }
        }

        q_new = map.at(closest_node) + (q_rand - map.at(closest_node)) / dist * r_;

        // if (!inCollision(q_new)) {
            // LOG("not");
            map.insert({ind, q_new});
            pathProblem.graph->connect(closest_node, ind, r_);
            heuristic.heuristic_values.insert({ind, distBetween(q_new,q_goal)});
        // } else {
        //     // LOG("colliding");
        //     ind--;
        //     continue;
        // }

        if (distBetween(q_new, q_goal) < min_dist) min_dist = distBetween(q_new, q_goal);

        if (distBetween(q_new, q_goal) < epsilon_) {
            goal_found = true;
            ind++;
            map.insert({ind,q_goal});
            pathProblem.graph->connect(ind-1, ind, distBetween(q_new, q_goal));
            heuristic.heuristic_values.insert({ind, distBetween(q_new,q_goal)});
        }
        // if (iter % 10 == 0) {
        //     amp::Visualizer::makeFigure(problem, *pathProblem.graph, [map](amp::Node node) -> Eigen::Vector2d { return map.at(node); });
        //     amp::Visualizer::showFigures();
        // }
    }

    // LOG("ind: " << ind);
    // LOG("goal found: " << goal_found);

    if (goal_found) {
        graph_ = *pathProblem.graph;
        map_ = map;

        amp::MyAStarAlgo aStar;
        pathProblem.init_node = 0;
        pathProblem.goal_node = map.size()-1;
        amp::AStar::GraphSearchResult searchResult = aStar.search(pathProblem, heuristic);

        amp::MultiAgentPath2D path(num_agents_);

        // LOG("path_size: " << searchResult.node_path.size());

        for (const auto& element : searchResult.node_path) {
            for (int i = 0; i < num_agents_; i++) {
                path.agent_paths[i].waypoints.push_back(Eigen::Vector2d({map.at(element)(2*i),map.at(element)(2*i+1)}));
            }
        }

        return path;
    }

    return amp::MultiAgentPath2D(num_agents_);
}

bool amp::MyCentralizedMultiAgentRRT::inCollision(Eigen::VectorXd state) {
    double dist2obs, dist2agent;
    double r_i, r_j;

    amp::MultiAgentPath2D path(num_agents_);

    for (int i = 0; i < num_agents_; i++) {
        path.agent_paths[i].waypoints.push_back(Eigen::Vector2d(state(2*i),state(2*i+1)));

        dist2obs = distance2obs(prob_, Eigen::Vector2d({state(2*i),state(2*i+1)}));
        r_i = prob_.agent_properties[i].radius;

        // LOG("agent " << i << " dist2obs: " << dist2obs);

        if (dist2obs < r_i) return true;
        for (int j = i+1; j < num_agents_; j++) {
            dist2agent = distBetween(Eigen::Vector2d({state(2*i),state(2*i+1)}), Eigen::Vector2d({state(2*j),state(2*j+1)}));
            r_j = prob_.agent_properties[j].radius;
            if (dist2agent < r_i + r_j) return true;

            // LOG("agent " << i << " pos: " << state(2*i) << ", " << state(2*i+1));
            // LOG("agent " << j << " pos: " << state(2*j) << ", " << state(2*j+1));
            // LOG("dist between " << i << " and " << j << ": " << dist2agent);
        }
    }

    // amp::Visualizer::makeFigure(amp::HW8::getWorkspace1(),path);
    // amp::Visualizer::showFigures();

    return false;
}

double amp::MyCentralizedMultiAgentRRT::distBetween(Eigen::VectorXd state1, Eigen::VectorXd state2) {
    double sum = 0;
    double diff1, diff2;

    for (int i = 0; i < state1.rows()/2; i++) {
        diff1 = state2(2*i)   - state1(2*i);
        diff2 = state2(2*i+1) - state1(2*i+1);
        sum += diff1*diff1 + diff2*diff2;
    }

    return sqrt(sum);
}

bool amp::MyCentralizedMultiAgentRRT::inPolygon(double x_pos, double y_pos) const {
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

bool amp::MyCentralizedMultiAgentRRT::lineIntersect(Eigen::Vector2d p1, Eigen::Vector2d p2, amp::Problem2D prob) {
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

Eigen::Vector2d amp::MyCentralizedMultiAgentRRT::distance2point(Eigen::Vector2d p1, Eigen::Vector2d p2) {
    double dy = p2[1] - p1[1];
    double dx = p2[0] - p1[0];

    double dist = sqrt( dx*dx + dy*dy );
    double angle = atan2(dy,dx);
    
    return Eigen::Vector2d(dist, angle);
}

bool amp::MyCentralizedMultiAgentRRT::primitiveCheck(Eigen::Vector2d point, double slope, Eigen::Vector2d intercept, int gt) {
    return ( ((gt % 2 == 1) ? -1 : 1) * ( (point[1] - intercept[1]) - slope * (point[0] - intercept[0]) ) <= 0 );
}

double amp::MyCentralizedMultiAgentRRT::distance2facet(Eigen::Vector2d point, double slope, Eigen::Vector2d intercept) {
    double a = 1;
    double b = -slope;
    double c = -intercept[1] + slope*intercept[0];
    return ( std::abs(a*point[0] + b*point[1] + c) / sqrt(a*a + b*b) );
}

Eigen::Vector2d amp::MyCentralizedMultiAgentRRT::nearFacet(Eigen::Vector2d point, Eigen::Vector2d p1, Eigen::Vector2d p2) {
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

double amp::MyCentralizedMultiAgentRRT::distance2obs(const amp::MultiAgentProblem2D& problem, Eigen::Vector2d point) {

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

        // LOG("d_obs: " << dist2obs);

        if (dist2obs < d_obs) d_obs = dist2obs;
    }
    return d_obs;
}