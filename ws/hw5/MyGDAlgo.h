#pragma once

#include "AMPCore.h"
#include "hw/HW5.h"

#include <thread>

namespace amp {

class MyGDAlgo : public GDAlgorithm {
    public:
        MyGDAlgo(double d_star, double Q_star) {}

        Eigen::Vector2d distance2point(Eigen::Vector2d point1,      Eigen::Vector2d point2);
        Eigen::Vector2d Grad_U_att(Eigen::Vector2d d_goal);
        bool primitiveCheck(Eigen::Vector2d point, double slope, Eigen::Vector2d intercept, int gt);
        double distance2facet(Eigen::Vector2d point, double slope, Eigen::Vector2d intercept);
        double nearFacet(Eigen::Vector2d point, Eigen::Vector2d p1, Eigen::Vector2d p2);
        std::vector<Eigen::Vector2d> distance2obs(const amp::Problem2D& problem, Eigen::Vector2d point);
        Eigen::Vector2d Grad_U_rep(std::vector<Eigen::Vector2d> d_obs);
        
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

    private:
        double m_d_star;
        double m_Q_star;
};

};