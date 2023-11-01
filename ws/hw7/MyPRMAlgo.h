#pragma once

#include "AMPCore.h"
#include "hw/HW7.h"

#include <thread>

namespace amp {

class GenericPRM {
    public:
        amp::Path plan_nd(const Eigen::VectorXd& init_state, 
                       const Eigen::VectorXd& goal_state/*, 
                       const amp::CollisionSpace& collision_checker*/);
};

class MyPRM2D : public amp::PRM2D, public GenericPRM {
    public:
        MyPRM2D() {}
        MyPRM2D(int n, double r) : n_(n), r_(r) {}
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
        Path2D plan_2D(const amp::Problem2D& problem);
        bool inPolygon(double x_pos, double y_pos) const;
        bool lineIntersect(double x1, double y1, double x2, double y2, amp::Problem2D);
        Graph<double> returnGraph() {return graph_;};
        std::map<amp::Node, Eigen::Vector2d> returnMap() {return map_;};

    private:
        amp::Problem2D prob_;
        Graph<double> graph_;
        std::map<amp::Node, Eigen::Vector2d> map_;
        int n_ = 200;
        double r_ = 2;
};

}