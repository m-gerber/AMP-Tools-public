#pragma once

#include "AMPCore.h"
#include "hw/HW5.h"

#include <thread>

namespace amp {

class MyGDAlgo : public GDAlgorithm {
    public:
        MyGDAlgo(double d_star, double Q_star) {}

        Eigen::Vector2d distance2point(Eigen::Vector2d point1,      Eigen::Vector2d point2);
        Eigen::Vector2d Grad_U_att(const amp::Problem2D& problem, Eigen::Vector2d point, double xi, double d_star);
        bool primitiveCheck(Eigen::Vector2d point, double slope, Eigen::Vector2d intercept, int gt);
        double distance2facet(Eigen::Vector2d point, double slope, Eigen::Vector2d intercept);
        Eigen::Vector2d nearFacet(Eigen::Vector2d point, Eigen::Vector2d p1, Eigen::Vector2d p2);
        std::vector<Eigen::Vector2d> distance2obs(const amp::Problem2D& problem, Eigen::Vector2d point);
        Eigen::Vector2d Grad_U_rep(const amp::Problem2D& problem, Eigen::Vector2d point, double eta, double Q_star);
        
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

    private:
        double m_d_star;
        double m_Q_star;
};

};