#pragma once

#include "AMPCore.h"
#include "hw/HW5.h"

#include <thread>

class MyGDAlgo : public amp::GDAlgorithm {
    public:
        MyGDAlgo(double d_star, double Q_star) {}

        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        double U_att(const amp::Problem2D& problem, double d_goal);
        double U_rep(const amp::Problem2D& problem, double d_obs);

        double distance2point(Eigen::Vector2d point1,      Eigen::Vector2d point2);
        double distance2obs(const amp::Problem2D& problem, Eigen::Vector2d point);

        bool nearFacet(Eigen::Vector2d point, Eigen::Vector2d p1, Eigen::Vector2d p2);

    private:
        double m_d_star;
        double m_Q_star;
};