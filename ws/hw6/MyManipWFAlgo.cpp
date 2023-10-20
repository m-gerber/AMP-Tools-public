#include "MyManipWFAlgo.h"
#include <queue>

int amp::MyManipWFAlgo::isValid(std::pair<int, int> inds, std::vector<std::vector<amp::cell2>> graph, const amp::GridCSpace2D& grid_cspace) {
    //if (inds.first < 0 || inds.second < 0 || inds.first > graph.size()-1 || inds.second > graph[0].size()-1) return 2;
    if (graph[inds.first][inds.second].visited) return 0;
    if (grid_cspace(inds.first,inds.second)) return -1;

    return 1;
}

amp::Path2D amp::MyManipWFAlgo::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) {
    DEBUG("enter");
    std::pair<double, double> x0_bounds = grid_cspace.x0Bounds();
    std::pair<double, double> x1_bounds = grid_cspace.x1Bounds();

    std::pair<int, int> num_cells = grid_cspace.size();
    std::pair<int, int> goal_cell = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);
    std::pair<int, int> init_cell = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);
    DEBUG("bounds: " << x0_bounds.first << ", " << x0_bounds.second);
    DEBUG("goal: " << q_goal[0] << ", " << q_goal[1]);
    DEBUG("af get cell from point");
    double cell_size_x0 = (x0_bounds.second-x0_bounds.first) / static_cast<double>(num_cells.first);
    double cell_size_x1 = (x1_bounds.second-x1_bounds.first) / static_cast<double>(num_cells.second);

    std::pair<int, int> curr_pos = goal_cell;
    DEBUG("goal cell: " << goal_cell.first << ", " << goal_cell.second);
    std::vector<std::vector<amp::cell2>> graph;
    graph.resize(num_cells.first);
    for (int i = 0; i < num_cells.first; i++) graph[i].resize(num_cells.second);

    graph[curr_pos.first][curr_pos.second].visited = 1;
    graph[curr_pos.first][curr_pos.second].val = 2;
    graph[curr_pos.first][curr_pos.second].parent = {-1,-1};

    std::queue<std::pair<int, int>> q;
    q.push(curr_pos);

    int dx[] = { -1, 0, 1,  0 };
    int dy[] = {  0, 1, 0, -1 };
    std::pair<int, int> new_pos;

    int iter1 = 0;

    int wrap_ind;

    while((curr_pos != init_cell) && !q.empty()) {
        iter1++;
        curr_pos = q.front();
        DEBUG("curr pos: " << curr_pos.first << ", " << curr_pos.second);
        q.pop();

        for (int i = 0; i < 4; i++) {
            new_pos.first = curr_pos.first + dx[i];
            new_pos.second = curr_pos.second + dy[i];
            if (new_pos.first >= num_cells.first)
                new_pos.first = 0;
            if (new_pos.first < 0)
                new_pos.first = num_cells.first - 1;
            if (new_pos.second >= num_cells.second)
                new_pos.second = 0;
            if (new_pos.second < 0)
                new_pos.second = num_cells.second - 1;
            DEBUG("b4 is valid");
            if (isValid({new_pos.first, new_pos.second}, graph, grid_cspace) == 1) {
                q.push({new_pos.first, new_pos.second});
                // DEBUG("pushing: " << new_pos.first << ", " << new_pos.second);
                // PAUSE;
                graph[new_pos.first][new_pos.second].visited = true;
                graph[new_pos.first][new_pos.second].val = graph[curr_pos.first][curr_pos.second].val + 1;
                graph[new_pos.first][new_pos.second].parent = curr_pos;
                graph[curr_pos.first][curr_pos.second].child = new_pos;
            } else if (isValid({new_pos.first, new_pos.second}, graph, grid_cspace) == -1) {
                graph[new_pos.first][new_pos.second].visited = true;
                graph[new_pos.first][new_pos.second].val = 1;
            } 
            // else if (isValid({new_pos.first, new_pos.second}, graph, grid_cspace) == 2) {
            //     if (new_pos.first == -1) {
            //         wrap_ind = grid_cspace.size().first-1;
            //         q.push({wrap_ind, new_pos.second});
            //         graph[wrap_ind][new_pos.second].visited = true;
            //         graph[wrap_ind][new_pos.second].val = graph[curr_pos.first][curr_pos.second].val + 1;
            //         graph[wrap_ind][new_pos.second].parent = curr_pos;
            //         graph[wrap_ind][curr_pos.second].child = new_pos;
            //     } else if (new_pos.second == -1) {
            //         wrap_ind = grid_cspace.size().second-1;
            //         q.push({wrap_ind, new_pos.second});
            //         graph[new_pos.first][wrap_ind].visited = true;
            //         graph[new_pos.first][wrap_ind].val = graph[curr_pos.first][curr_pos.second].val + 1;
            //         graph[new_pos.first][wrap_ind].parent = curr_pos;
            //         graph[new_pos.first][wrap_ind].child = new_pos;
            //     } else if (new_pos.first == graph.size()) {
            //         wrap_ind = 0;
            //         q.push({wrap_ind, new_pos.second});
            //         graph[wrap_ind][new_pos.second].visited = true;
            //         graph[wrap_ind][new_pos.second].val = graph[curr_pos.first][curr_pos.second].val + 1;
            //         graph[wrap_ind][new_pos.second].parent = curr_pos;
            //         graph[wrap_ind][curr_pos.second].child = new_pos;
            //     } else if (new_pos.first == graph.size()) {
            //         wrap_ind = 0;
            //         q.push({wrap_ind, new_pos.second});
            //         graph[new_pos.first][wrap_ind].visited = true;
            //         graph[new_pos.first][wrap_ind].val = graph[curr_pos.first][curr_pos.second].val + 1;
            //         graph[new_pos.first][wrap_ind].parent = curr_pos;
            //         graph[new_pos.first][wrap_ind].child = new_pos;
            //     }
            // }
        }
    }

    amp::Path2D path;
    int iter2 = 0;

    path.waypoints.push_back(Eigen::Vector2d(q_init[0], q_init[1]));

    while (curr_pos != goal_cell || iter2 == iter1) {
        iter2++;
        // std::cout << "curr_pos: " << curr_pos.first << "," << curr_pos.second << std::endl;
        // std::cout << "curr_point: " << graph[curr_pos.first][curr_pos.second].val << std::endl;
        // std::cout << "curr_point: " << graph[curr_pos.first][curr_pos.second].parent.first << "," << graph[curr_pos.first][curr_pos.second].parent.second << std::endl;
        //PAUSE;
        path.waypoints.push_back(Eigen::Vector2d(curr_pos.first*cell_size_x0 + x0_bounds.first,curr_pos.second*cell_size_x1 + x1_bounds.first));
        curr_pos = graph[curr_pos.first][curr_pos.second].parent;
    }

    // path.waypoints.push_back(Eigen::Vector2d(q_init[0], q_init[1]));
    path.waypoints.push_back(Eigen::Vector2d(q_goal[0], q_goal[1]));

    return path;
}