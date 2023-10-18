#pragma once

#include <memory>

#include "tools/Environment.h"
#include "tools/Obstacle.h"
#include "tools/LinkManipulator.h"
#include "tools/ConfigurationSpace.h"

#include "hw/HW4.h"
#include "MyGridCSpace.h"

namespace amp {

class MyGridCSpace2DConstructor : public GridCSpace2DConstructor {
    public:
        std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override;
};

}