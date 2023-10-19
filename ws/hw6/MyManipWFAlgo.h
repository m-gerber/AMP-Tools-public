#pragma once

#include "AMPCore.h"
#include "hw/HW6.h"
#include "MyGridCSpace.h"
#include "MyGridCSpace2DConstructor.h"

#include <thread>

namespace amp {

class MyManipWFAlgo : public amp::ManipulatorWaveFrontAlgorithm {
    public:
        // Default ctor
        MyManipWFAlgo()
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyGridCSpace2DConstructor>()) {}

        // You can have custom ctor params for all of these classes
        MyManipWFAlgo(const std::string& beep) 
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyGridCSpace2DConstructor>()) {LOG("constructing... " << beep);}

        // This is just to get grade to work, you DO NOT need to override this method
        virtual amp::ManipulatorTrajectory2Link plan(const LinkManipulator2D& link_manipulator_agent, const amp::Problem2D& problem) override {
            return amp::ManipulatorTrajectory2Link();
        }
        
        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override;
};

}