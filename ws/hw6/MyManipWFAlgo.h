#pragma once

#include "AMPCore.h"
#include "hw/HW6.h"
#include "MyGridCSpace.h"
#include "MyGridCSpace2DConstructor.h"

#include <thread>

namespace amp {

struct cell2 {
    bool visited = 0;
    int val = 0;
    std::pair<int, int> parent;
    std::pair<int, int> child;
};

class MyManipWFAlgo : public amp::ManipulatorWaveFrontAlgorithm {
    public:
        // Default ctor
        MyManipWFAlgo()
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyGridCSpace2DConstructor>()) {}

        // You can have custom ctor params for all of these classes
        MyManipWFAlgo(const std::string& beep) 
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyGridCSpace2DConstructor>()) {LOG("constructing... " << beep);}



        int isValid(std::pair<int, int> inds, std::vector<std::vector<cell2>> graph, const amp::GridCSpace2D& grid_cspace);
        
        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override;
};

}