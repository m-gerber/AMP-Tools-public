#pragma once

#include "AMPCore.h"
#include "hw/HW8.h"

#include <thread>
#include <map>

namespace amp {

class MyCentralizedMultiAgentRRT : public amp::CentralizedMultiAgentRRT {
    public:
        MyCentralizedMultiAgentRRT() {}
        MyCentralizedMultiAgentRRT(int n, double r, double bias, double epsilon) : n_(n), r_(r), bias_(bias), epsilon_(epsilon) {}
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;
        bool inCollision(Eigen::VectorXd state);
        bool inPolygon(double x_pos, double y_pos) const;
        bool lineIntersect(Eigen::Vector2d p1, Eigen::Vector2d p2, amp::Problem2D);
        Eigen::Vector2d distance2point(Eigen::Vector2d p1, Eigen::Vector2d p2);
        bool primitiveCheck(Eigen::Vector2d point, double slope, Eigen::Vector2d intercept, int gt);
        double distance2facet(Eigen::Vector2d point, double slope, Eigen::Vector2d intercept);
        Eigen::Vector2d nearFacet(Eigen::Vector2d point, Eigen::Vector2d p1, Eigen::Vector2d p2);
        double distance2obs(const amp::MultiAgentProblem2D& problem, Eigen::Vector2d point);
        int returnTreeSize() {return tree_size_;};

    private:
        amp::MultiAgentProblem2D prob_;
        int num_agents_;
        int tree_size_ = 0;
        int n_          = 7500; // Maximum number of iteration
        double r_       = 0.5;  // Step size
        double bias_    = 0.05; // Probability to sample goal
        double epsilon_ = 0.25; // Radius around goal
};

}