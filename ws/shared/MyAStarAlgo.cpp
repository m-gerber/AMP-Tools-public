#include "MyAStarAlgo.h"
#include <queue>

amp::AStar::GraphSearchResult amp::MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    DEBUG("in astar");

    amp::AStar::GraphSearchResult mySearchResult;
    {
    // problem.graph->print();

    amp::Node init_node = problem.init_node;
    amp::Node goal_node = problem.goal_node;

    // Create vector for open list
    std::priority_queue<amp::Node_Info, std::vector<amp::Node_Info>, amp::Comparef> open_list;

    amp::Node_Info init_info = amp::Node_Info(init_node, -1, 0, heuristic.operator()(init_node), heuristic.operator()(init_node));
    open_list.push(init_info);
    
    // Create a vector for the closed list
    std::vector<amp::Node_Info> closed_list(problem.graph->nodes().size());
    closed_list[init_node] = init_info;
    int parent_node;

    double g, h, f;
    amp::Node_Info update_info;
    bool updated;
    bool goal_found = false;
    double goal_dist;

    double prev_parent = -2;

    int iter = 0;
    DEBUG("b4 loop");
    while(!open_list.empty()) {
        iter++;
        
        while (closed_list[open_list.top().node].parent != -1 && !open_list.empty()) {
            open_list.pop(); 
        }
        if (open_list.empty()) break;
        

        // LOG("queue size: " << open_list.size());
        // LOG("top: " << open_list.top().node << " " << open_list.top().parent << " " << open_list.top().g << " " << open_list.top().h << " " << open_list.top().f);

        
        parent_node = open_list.top().node;
        closed_list[parent_node] = open_list.top();

        if (goal_found) {
            if (open_list.top().f >= goal_dist) break;
        }

        open_list.pop();

        const auto& children       = problem.graph->children(parent_node);
        const auto& outgoing_edges = problem.graph->outgoingEdges(parent_node);

        for (int i = 0; i < children.size(); i++) {
            if (children[i] != parent_node && closed_list[children[i]].node == -1) {
                g = closed_list[parent_node].g + outgoing_edges[i];
                h = heuristic.operator()(children[i]);
                f = g + h;
                update_info = amp::Node_Info(children[i], parent_node, g, h, f);
                open_list.push(update_info);
                if (update_info.node == goal_node && !goal_found) {
                    goal_found = true;
                    goal_dist = update_info.f;
                }
                // LOG("update_info: " << update_info.node << " " << update_info.parent << " " << update_info.g << " " << update_info.h << " " << update_info.f);
            }
        }
        // PAUSE;
        prev_parent = parent_node;
    }
    DEBUG("af loop");
    mySearchResult.success = goal_found;
    DEBUG("goal found: " << goal_found);

    if (goal_found) {
        mySearchResult.path_cost = closed_list[goal_node].f;

        std::stack<amp::Node_Info> path_stack;

        amp::Node_Info curr_node = closed_list[goal_node];

        while (curr_node.node != init_node) {
            path_stack.push(curr_node);
            curr_node = closed_list[curr_node.parent];
        }
        path_stack.push(curr_node);

        std::list<amp::Node> myPath;

        while (path_stack.empty() == 0) {
            myPath.push_back(static_cast<amp::Node>(path_stack.top().node));
            path_stack.pop();
        }
        mySearchResult.node_path = myPath;

        // LOG("This path took " << iter << " iterations.");
        // LOG("The length of this path is " << closed_list[goal_node].f << ".");
    }
    DEBUG("returning");
    }
    DEBUG("af scope");
    // mySearchResult.node_path.clear();
    return mySearchResult;
   
    //return amp::AStar::GraphSearchResult{};
}