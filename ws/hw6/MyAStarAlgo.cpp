#include "MyAStarAlgo.h"
#include <queue>

amp::AStar::GraphSearchResult amp::MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {

    amp::AStar::GraphSearchResult mySearchResult;

    amp::Node init_node = problem.init_node;
    amp::Node goal_node = problem.goal_node;

    // Create vector for open list
    std::vector<amp::Node_Info> open_list;

    amp::Node_Info init_info = amp::Node_Info(init_node, -1, 0, heuristic.operator()(init_node), heuristic.operator()(init_node));
    open_list.push_back(init_info);
    // LOG("init_info: " << init_info.node << " " << init_info.parent << " " << init_info.g << " " << init_info.h << " " << init_info.f);
    
    // Create a vector for the closed list
    std::vector<amp::Node_Info> closed_list(problem.graph->nodes().size());
    closed_list[init_node] = init_info;
    int parent_node;

    double g, h, f;
    amp::Node_Info update_info;
    bool updated;
    bool goal_found = false;
    double goal_dist;

    while(!open_list.empty()) {
        // DEBUG("new loop");
        parent_node = open_list.back().node;
        closed_list[parent_node] = open_list.back();

        if (goal_found) {
            // DEBUG("goal_dist orig: " << goal_dist); 
            // DEBUG("open_list.back().h: " << open_list.back().h); 
            if (open_list.back().f >= goal_dist) break;
        }

        open_list.pop_back();

        const auto& children       = problem.graph->children(parent_node);
        const auto& outgoing_edges = problem.graph->outgoingEdges(parent_node);

        for (int i = 0; i < children.size(); i++) {
            g = closed_list[parent_node].g + outgoing_edges[i];
            h = heuristic.operator()(children[i]);
            f = g + h;
            update_info = amp::Node_Info(children[i], parent_node, g, h, f);

            for (int j = 0; j < open_list.size(); j++) {
                updated = false;
                
                double n1 = open_list[j].node;
                double n2 = update_info.node;
                
                if (std::abs(n1 - n2) < 0.01) {
                    if (open_list[j].f > update_info.f) {
                        open_list[j] = update_info;
                    }
                    updated = true;
                    break;
                }
            }
            if (!updated) open_list.push_back(update_info);
            
            std::sort(open_list.begin(), open_list.end(), [](Node_Info n1, Node_Info n2) { return n1.f > n2.f; });

            if (update_info.node == goal_node && !goal_found) {
                goal_found = true;
                goal_dist = update_info.f;
                // DEBUG("goal_dist fresh: " << goal_dist); 
            }
            // LOG("update_info: " << update_info.node << " " << update_info.parent << " " << update_info.g << " " << update_info.h << " " << update_info.f);
        }
        // for (int i = 0; i < open_list.size(); i++) {
        //     LOG("queue " << i << ": " << open_list[i].node << " " << open_list[i].parent << " " << open_list[i].g << " " << open_list[i].h << " " << open_list[i].f);
        // }
    }

    // for (int i = 0; i < open_list.size(); i++) {
    //     LOG("queue " << i << ": " << open_list[i].node << " " << open_list[i].parent << " " << open_list[i].g << " " << open_list[i].h << " " << open_list[i].f);
    // }

    mySearchResult.success = goal_found;

    if (goal_found) {
        mySearchResult.path_cost = closed_list[goal_node].f;

        std::stack<amp::Node_Info> path_stack;

        amp::Node_Info curr_node = closed_list[goal_node];

        while (curr_node.node != init_node) {
            // std::cout << "curr_node: " << curr_node.node << std::endl;
            path_stack.push(curr_node);
            curr_node = closed_list[curr_node.parent];
        }
        path_stack.push(curr_node);

        // std::cout << "top node: " << static_cast<amp::Node>(path_stack.top().node) << std::endl;

        std::list<amp::Node> myPath;

        while (path_stack.empty() == 0) {
            myPath.push_back(static_cast<amp::Node>(path_stack.top().node));
            // mySearchResult.node_path.push_back(static_cast<amp::Node>(path_stack.top().node));
            // LOG(path_stack.top());
            path_stack.pop();
        }
        mySearchResult.node_path = myPath;
    } 
    std::cout << "algo path: " << std::endl;
    for (const amp::Node& node : mySearchResult.node_path) {
        std::cout << node << " ";
    }
    // std::cout << std::endl;

    std::cout << "Finished!" << std::endl;

    return mySearchResult;
}