#include "MyPointWFAlgo.h"
#include "MyGridCSpace.h"

/// @brief Create a discretized planning space of an environment (point agent). If you abstracted your GridCSpace2DConstructor, you may be
/// able to use that code here to construct a discretized C-space for a point agent.
/// @param environment Workspace and/or C-space (point agent)
/// @return Unique pointer to a GridCSpace2D object (see HW4)
std::unique_ptr<amp::GridCSpace2D> amp::MyPointWFAlgo::constructDiscretizedWorkspace(const amp::Environment2D& env) {
    std::unique_ptr<amp::MyGridCSpace> ptr(new MyGridCSpace(100, 100, env.x_min, env.x_max, env.y_min, env.y_max));
    ptr->buildPointCSpace(env);

    return ptr;
}

amp::Path2D amp::MyPointWFAlgo::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) {

    std::pair<double, double> x0_bounds = grid_cspace.x0Bounds();
    std::pair<double, double> x1_bounds = grid_cspace.x1Bounds();

    std::pair<int, int> num_cells = grid_cspace.size();
    std::pair<int, int> goal_cell = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);
    std::pair<int, int> init_cell = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);

    std::cout << "goal: " << grid_cspace.operator()(goal_cell.first,goal_cell.second) << std::endl;

    std::pair<int, int> curr_pos = goal_cell;

    // Assuming we can loop over items
    while(curr_pos.first != init_cell.first && curr_pos.second != init_cell.second) {

        

    }


    return amp::Path2D();
}