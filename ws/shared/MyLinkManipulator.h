#pragma once

#include <vector>
#include <Eigen/Core>

#include "tools/Serializer.h"
#include "tools/LinkManipulator.h"

namespace amp {

class MyLinkManipulator : public LinkManipulator2D {
    public:
        MyLinkManipulator() : LinkManipulator2D() {}
        MyLinkManipulator(const std::vector<double>& link_lengths) : LinkManipulator2D(link_lengths) {
            getLinkLengths() = link_lengths;
        }
        Eigen::MatrixXd makeRotationMatrix(double theta, double x, double y) const;
        Eigen::MatrixXd rotate(std::vector<Eigen::MatrixXd> mats);
        Eigen::Vector2d getJointLocation(const ManipulatorState& state, uint32_t joint_index) const override;
        amp::ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override;
        std::vector<Eigen::Vector2d> CircleIntersection(double x0, double y0, double x1, double y1, double r0, double r1) const;
    private:
};

}