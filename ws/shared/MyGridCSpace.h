#pragma once

#include <vector>
#include <Eigen/Core>

#include "tools/ConfigurationSpace.h"
#include "tools/LinkManipulator.h"
#include "tools/Environment.h"
#include "tools/Serializer.h"

#include "MyLinkManipulator.h"

namespace amp {

class MyGridCSpace : public GridCSpace2D {
    public:
        // MyGridCSpace() {}
        MyGridCSpace(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
        : GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) 
        , denseArray(x0_cells, x1_cells) {}

        amp::MyGridCSpace buildCSpace(amp::MyLinkManipulator links, amp::Environment2D env);
        bool inCollision(double angle0, double angle1) const override;
        
    private:
        amp::MyLinkManipulator links_;
        amp::Environment2D environment_;
        amp::DenseArray2D<bool> denseArray;
};

}