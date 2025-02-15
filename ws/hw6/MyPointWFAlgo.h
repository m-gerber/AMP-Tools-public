#pragma once

#include "AMPCore.h"
#include "hw/HW6.h"
#include "MyGridCSpace.h"

#include <thread>

namespace amp {

struct cell {
    bool visited = 0;
    int val = 0;
    std::pair<int, int> parent;
    std::pair<int, int> child;
};

class MyPointWFAlgo : public amp::PointWaveFrontAlgorithm {
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override;


        int isValid(std::pair<int, int> inds, std::vector<std::vector<cell>> graph, const amp::GridCSpace2D& grid_cspace);

        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override;
};

}