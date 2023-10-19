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
        , x0_cells_(x0_cells), x1_cells_(x1_cells), x0_min_(x0_min), x0_max_(x0_max), x1_min_(x1_min), x1_max_(x1_max) {}

        void buildPointCSpace(amp::Environment2D env);
        bool inPolygon(double x_pos, double y_pos) const;
        void buildLinkCSpace(amp::MyLinkManipulator links, amp::Environment2D env);
        bool collisionChecker(double angle0, double angle1) const;
        virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const {
            double dx0 = x0_max_ - x0_min_;
            double dx1 = x1_max_ - x1_min_;

            int ind_x0 = ceil(x0 / dx0 * x0_cells_);
            int ind_x1 = ceil(x1 / dx1 * x1_cells_);

            return {ind_x0, ind_x1};
        }
        
    private:
        amp::MyLinkManipulator links_;
        amp::Environment2D env_;
        std::size_t x0_cells_;
        std::size_t x1_cells_;
        double x0_min_;
        double x0_max_;
        double x1_min_;
        double x1_max_;
};

}