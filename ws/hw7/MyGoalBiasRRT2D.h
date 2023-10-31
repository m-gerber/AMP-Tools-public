#pragma once

#include "AMPCore.h"
#include "hw/HW7.h"

#include <thread>
#include <map>

namespace amp {

class MyGoalBiasRRT2D : public amp::GoalBiasRRT2D {
    public:
        MyGoalBiasRRT2D() {}
        MyGoalBiasRRT2D(int n, double r, double bias, double epsilon) : n_(n), r_(r), bias_(bias), epsilon_(epsilon) {}
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
        double distBetween(Eigen::Vector2d p1, Eigen::Vector2d p2);
        bool inPolygon(double x_pos, double y_pos) const;
        bool lineIntersect(Eigen::Vector2d p1, Eigen::Vector2d p2, amp::Problem2D);
        Graph<double> returnGraph() {return graph_;};
        std::map<amp::Node, Eigen::Vector2d> returnMap() {return map_;};

    private:
        amp::Problem2D prob_;
        Graph<double> graph_;
        std::map<amp::Node, Eigen::Vector2d> map_;
        int n_          = 5000; // Maximum number of iteration
        double r_       = 0.5;  // Step size
        double bias_    = 0.05; // Probability to sample goal
        double epsilon_ = 0.25; // Radius around goal
};

}