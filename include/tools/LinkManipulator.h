#pragma once

#include <vector>
#include <Eigen/Core>

#include "tools/Serializer.h"

namespace amp {

/// @brief Vector of angles (radians) for each joint. The size of the vector should match the 
/// number of links (and hence joints) of the manipulator
using ManipulatorState = std::vector<double>;

/// @brief List of manipulator states in chronological order
using ManipulatorTrajectory = std::list<ManipulatorState>;

class LinkManipulator2D {
    public:
        /// @brief Construct 2-link manipulator. The base location is set to (0.0, 0.0), each link_length is 1.0
        LinkManipulator2D();

        /// @brief Construct from an array of link lengths. The base location is set to (0.0, 0.0)
        /// @param link_lengths Array of link lengths. The number of link lengths dictates the DOF of the manipulator
        LinkManipulator2D(const std::vector<double>& link_lengths);

        /// @brief Construct from an array of link lengths.
        /// @param base_location Custom base location of the manipulator
        /// @param link_lengths Array of link lengths. The number of link lengths dictates the DOF of the manipulator
        LinkManipulator2D(const Eigen::Vector2d& base_location, const std::vector<double>& link_lengths);

        /******* User Implemented Methods ********/

        /// @brief Get the location of the nth joint using the current link attributes using Forward Kinematics
        /// @param state Joint angle state (radians). Must have size() == nLinks()
        /// @param joint_index Joint index in order of base to end effector 
        /// (joint_index = 0 should return the base location, joint_index = nLinks() should return the end effector location)
        /// @return Joint coordinate
        virtual Eigen::Vector2d getJointLocation(const ManipulatorState& state, uint32_t joint_index) const = 0;

        /// @brief Set the configuration (link attributes) give an end effector location using Inverse Kinematics
        /// @param end_effector_location End effector coordinate
        /// @return Joint angle state (radians) in increasing joint index order. Must have size() ==nLinks()
        virtual ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const = 0;

        /*****************************************/

        /// @brief Get the number of links. The number of links is the same as the number of joints
        /// @return Number of links
        inline std::size_t nLinks() const {return m_link_lengths.size();}

        /// @brief Read the link lengths.
        /// @return Const reference access to link lengths
        inline const std::vector<double>& getLinkLengths() const {return m_link_lengths;}

        /// @brief Edit the link lengths.
        /// @return Reference access to link lengths
        inline std::vector<double>& getLinkLengths() {return m_link_lengths;}

        /// @brief Read the base location.
        /// @return Const reference to base location
        inline const Eigen::Vector2d& getBaseLocation() const {return m_base_location;}

        /// @brief Get the maximum radial reach of the manipulator 
        /// @return Sum of all of the link lengths
        double reach() const;

        /// @brief Print the object
        /// @param heading Log what type of object is being printed
        void print(const std::string& heading = "LinkManipulator2D") const;

        /// @brief Virtual dtor
        virtual ~LinkManipulator2D() {}
    protected:
        Eigen::Vector2d m_base_location = Eigen::Vector2d(0.0, 0.0);
        std::vector<double> m_link_lengths;
};

class MyLinkManipulator : public LinkManipulator2D {
    public:
         MyLinkManipulator() : LinkManipulator2D() {
        }

        MyLinkManipulator(const std::vector<double>& link_lengths) : LinkManipulator2D(link_lengths) {
            getLinkLengths() = link_lengths;
        }

        Eigen::MatrixXd makeRotationMatrix(double theta, double x, double y) const {
            Eigen::MatrixXd rot(3,3);
            rot << cos(theta), -sin(theta), x,
                   sin(theta),  cos(theta), y,
                   0,           0,          1;
            return rot;
        }

        Eigen::MatrixXd rotate(std::vector<Eigen::MatrixXd> mats) {
            Eigen::MatrixXd result(3,3), final_inds(3,1);
            final_inds << 0, 0, 1;
            for (int i = 0; i < mats.size()-1; i++) {
                result = mats[i] * mats[i+1];
            }
            return result*final_inds;
        }

        Eigen::Vector2d getJointLocation(const ManipulatorState& state, uint32_t joint_index) const override {
            if (joint_index == 0) return Eigen::Vector2d(0,0);

            Eigen::MatrixXd rot_mat(3,3), temp(3,3), final_inds(3,1);
            final_inds << 0, 0, 1;
            rot_mat = makeRotationMatrix(0,getLinkLengths()[joint_index-1],0);

            for (int i = joint_index-1; i > 0; i--) {
                temp << makeRotationMatrix(state[i],getLinkLengths()[i-1],0);
                rot_mat = temp * rot_mat;
            }

            temp << makeRotationMatrix(state[0],0,0);
            rot_mat = temp * rot_mat;
            rot_mat = rot_mat * final_inds;

            return Eigen::Vector2d(rot_mat(0),rot_mat(1));
        }

        ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override {

            auto t2_cos = [](double x, double y, double a0, double a1) {
                return (1.0 / (2.0 * a0 * a1)) * ((x * x + y * y) - (a0 * a0 + a1 * a1));
            };
            auto t2_sin = [](double x, double y, double a0, double a1) {
                double t2cos = (1.0 / (2.0 * a0 * a1)) * ((x * x + y * y) - (a0 * a0 + a1 * a1));
                return std::sqrt(1.0 - t2cos * t2cos);
            };
            auto t1_cos = [](double x, double y, double a0, double a1, double t2) {
                double denom = x * x + y * y;
                double cos_t2 = std::cos(t2);
                double sqrt_term = std::sqrt(1.0 - cos_t2 * cos_t2);
                return (1.0 / denom) * (x * (a0 + a1 * cos_t2) + y * a1 * sqrt_term);
            };
            auto t1_sin = [](double x, double y, double a0, double a1, double t2) {
                double denom = x * x + y * y;
                double cos_t2 = std::cos(t2);
                double sqrt_term = std::sqrt(1.0 - cos_t2 * cos_t2);
                return (1.0 / denom) * (y * (a0 + a1 * cos_t2) - x * a1 * sqrt_term);
            };

            int num_links = getLinkLengths().size();
            double x_e = end_effector_location[0];
            double y_e = end_effector_location[1];

            double a0 = getLinkLengths()[0];
            double a1, a2;

            double dist = sqrt(x_e*x_e + y_e*y_e);

            double theta1, theta2, theta3;

            amp::ManipulatorState angles;
            std::vector<Eigen::Vector2d> valid_points;
            double x_j1, y_j1, x_j2, y_j2;
            double mag_A, mag_B, dot_AB;
            double angle_A, angle_B;

            switch(num_links) {
                case 1:
                    if (dist == a0) {
                        angles.push_back(atan2(y_e,x_e));
                    }
                    break;
                case 2:
                    a1 = getLinkLengths()[1];
                    theta2 = atan2(t2_sin(x_e,y_e,a0,a1),t2_cos(x_e,y_e,a0,a1));
                    theta1 = atan2(t1_sin(x_e,y_e,a0,a1,theta2),t1_cos(x_e,y_e,a0,a1,theta2));
                    angles.push_back(theta1);
                    angles.push_back(theta2);
                    break;
                case 3:
                    a1 = getLinkLengths()[1];
                    a2 = getLinkLengths()[2];

                    if (dist+a2 < a0+a1 && std::abs(dist-a2) > a0-a1) {
                        valid_points.push_back(Eigen::Vector2d(x_e+a2,y_e));
                    } else if (dist+a2 > a0+a1 && dist-a2 < a0+a1) {
                        valid_points = CircleIntersection(0.0,0.0,x_e,y_e,a0+a1,a2);
                    } else {
                        valid_points = CircleIntersection(0.0,0.0,x_e,y_e,a0-a1,a2);
                    }

                    if (valid_points.empty()) {
                        break;
                    }

                    x_j2 = valid_points[0][0];
                    y_j2 = valid_points[0][1];

                    theta2 = atan2(t2_sin(x_j2,y_j2,a0,a1),t2_cos(x_j2,y_j2,a0,a1));
                    theta1 = atan2(t1_sin(x_j2,y_j2,a0,a1,theta2),t1_cos(x_j2,y_j2,a0,a1,theta2));

                    x_j1 = a0*cos(theta1);
                    y_j1 = a0*sin(theta1);

                    mag_A = std::hypot(x_e - x_j2, y_e - y_j2);
                    mag_B = std::hypot(x_j1 - x_j2, y_j1 - y_j2);

                    dot_AB = (x_e - x_j2) * (x_j1 - x_j2) + (y_e - y_j2) * (y_j1 - y_j2);

                    theta3 = M_PI - acos(dot_AB/(mag_A*mag_B));

                    angle_A = atan2(y_e - y_j2, x_e - x_j2);
                    angle_B = atan2(y_j1 - y_j2, x_j1 - x_j2);
                    if (angle_A < 0) angle_A += 2*M_PI;
                    if (angle_B < 0) angle_B += 2*M_PI;
                    if (angle_B < angle_A) theta3 = -theta3;

                    angles.push_back(theta1);
                    angles.push_back(theta2);
                    angles.push_back(theta3);
                    break;
                default:
                    std::cout << "Too many links." << std::endl;
                    break;
            }

            if (angles.empty()) {
                std::cout << "Target is outside of reachable range." << std::endl;
            }

            return angles;
        }

        std::vector<Eigen::Vector2d> CircleIntersection(double x0, double y0, double x1, double y1, double r0, double r1) const {

            std::vector<Eigen::Vector2d> intersections;

            auto d = [](double x0, double y0, double x1, double y1) {
                return std::hypot(x1 - x0, y1 - y0);
            };
            auto l = [](double r0, double r1, double d) {
                return (r0 * r0 - r1 * r1 + d * d) / (2.0 * d);
            };
            auto h = [](double r0, double l) {
                return std::sqrt(r0 * r0 - l * l);
            };

            auto xp = [](double d, double l, double h, double x0, double y0, double x1, double y1, int pm) {
                return (l / d) * (x1 - x0) + ((pm % 2 == 0) ? 1 : -1) * (h / d) * (y1 - y0) + x0;
            };
            auto yp = [](double d, double l, double h, double x0, double y0, double x1, double y1, int pm) {
                return (l / d) * (y1 - y0) - ((pm % 2 == 0) ? 1 : -1) * (h / d) * (x1 - x0) + y0;
            };

            double d_val = d(x0,y0,x1,y1);
            double l_val = l(r0,r1,d_val);
            double h_val = h(r0,l_val);

            double xp_p = xp(d_val,l_val,h_val,x0,y0,x1,y1,0);
            double yp_p = yp(d_val,l_val,h_val,x0,y0,x1,y1,0);
            double xp_m = xp(d_val,l_val,h_val,x0,y0,x1,y1,1);
            double yp_m = yp(d_val,l_val,h_val,x0,y0,x1,y1,1);

            intersections.push_back(Eigen::Vector2d(xp_p,yp_p));
            intersections.push_back(Eigen::Vector2d(xp_m,yp_m));

            return intersections;
        }

    private:
};

}