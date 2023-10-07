#pragma once

#include <vector>
#include <Eigen/Core>
#include "LinkManipulator.h"
#include "Environment.h"

#include "tools/Serializer.h"

namespace amp {

/// @brief User implemented abstract class that accesses the continuous C-Space (bounded)
class ConfigurationSpace2D {
    public:
        /// @brief Constructor
        /// @param x0_min Lower bound on first configuration dimension
        /// @param x0_max Upper bound on first configuration dimension
        /// @param x1_min Lower bound on second configuration dimension
        /// @param x1_max Upper bound on second configuration dimension
        ConfigurationSpace2D(double x0_min, double x0_max, double x1_min, double x1_max)
            : m_x0_bounds(x0_min, x0_max)
            , m_x1_bounds(x1_min, x1_max)
            {}

        /******* User Implemented Methods ********/

        /// @brief Access the C-space with continuous variables (interpolation between cells)
        /// @param x0 Value of the first configuration variable
        /// @param x1 Value of the second configuration variable
        /// @return `true` if the the point is in collision, `false` if it is not
        virtual bool inCollision(double x0, double x1) const = 0;

        /*****************************************/

        /// @brief Get bounds for the first config dimension
        /// @return Bounds
        inline const std::pair<double, double>& x0Bounds() const {
            return m_x0_bounds;
        }

        /// @brief Get bounds for the second config dimension
        /// @return Bounds
        inline const std::pair<double, double>& x1Bounds() const {
            return m_x1_bounds;
        }

        /// @brief Virtual dtor
        virtual ~ConfigurationSpace2D() {}
    protected:
        std::pair<double, double> m_x0_bounds;
        std::pair<double, double> m_x1_bounds;
};

/* Some of the tools below might help you define your C-Space */

template <typename T = bool>
class DenseArray2D {
    public:
        /// @brief Constructor that initializes data to T{}
        /// @param x0_cells Number of cells along the first configuration dimension
        /// @param x1_cells Number of cells along the second configuration dimension
        DenseArray2D(std::size_t x0_cells, std::size_t x1_cells);

        /// @brief Constructor that initializes data to T{}
        /// @param x0_cells Number of cells along the first configuration dimension
        /// @param x1_cells Number of cells along the second configuration dimension
        /// @param default_element Use custom default element to fill array with
        DenseArray2D(std::size_t x0_cells, std::size_t x1_cells, T default_element);

        /// @brief Get the # of cells along x0 and number of cells along x1
        /// @return 
        inline std::pair<std::size_t, std::size_t> size() const;

        /// @brief Edit/Access an element with bounds checking
        /// @param i x0 index
        /// @param j x1 index
        /// @return Reference to the element (for use with vector<bool>)
        inline typename std::vector<T>::reference operator()(std::size_t i, std::size_t j);

        /// @brief Read an element with bounds checking
        /// @param i x0 index
        /// @param j x1 index
        /// @return Const reference to the element
        inline typename std::vector<T>::const_reference operator()(std::size_t i, std::size_t j) const;

        /// @brief Read the wrapped data
        /// @return Wrapped data
        inline const std::vector<bool>& data() const;

        /// @brief Virtual dtor
        virtual ~DenseArray2D() {}

    private:
        inline std::size_t getWrappedIndex(std::size_t i, std::size_t j) const;

    private:
        std::vector<T> m_data;
        const std::size_t m_x0_cells, m_x1_cells;
};

/// @brief Abstract type that can be used with the Visualizer
class GridCSpace2D : public ConfigurationSpace2D, public DenseArray2D<bool> {
    public:
        GridCSpace2D(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : ConfigurationSpace2D(x0_min, x0_max, x1_min, x1_max)
            , DenseArray2D<bool>(x0_cells, x1_cells)
            {}

        /// @brief Virtual dtor
        virtual ~GridCSpace2D() {}
};

class MyGridCSpace : public GridCSpace2D {
    public:
        MyGridCSpace();
    
        MyGridCSpace(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
        : GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) 
        , denseArray(x0_cells, x1_cells) {}

        amp::MyGridCSpace buildCSpace(amp::MyLinkManipulator links, amp::Environment2D env) {
            environment_ = env;
            links_ = links; 

            amp::MyGridCSpace temp_grid(360,360,0,2*M_PI,0,2*M_PI);

            int grid_discretization = denseArray.size().first;
            double dtheta = 2*M_PI / grid_discretization;

            double x0, x1;

            for (int i = 0; i < grid_discretization; i++) {
                x0 = dtheta*i;
                for (int j = 0; j < grid_discretization; j++) {
                    x1 = dtheta*j;
                    denseArray(i,j) = inCollision(x0,x1);
                    temp_grid(i,j) = inCollision(x0,x1);
                }
            }
            return temp_grid;
        }

        bool inCollision(double angle0, double angle1) const override {
            amp::ManipulatorState state = {angle0,angle1};
            Eigen::Vector2d j1 = links_.getJointLocation(state,1);
            Eigen::Vector2d j2 = links_.getJointLocation(state,2);

            double x0 = 0;
            double y0 = 0;
            double x1 = j1[0];
            double y1 = j1[1];
            double x2 = j2[0];
            double y2 = j2[1];

            double t, u;
            double x3, y3, x4, y4;

            double slope1, slope2;
            double x1_y3, x1_y4, x2_y3, x2_y4;

            int num_obstacles = environment_.obstacles.size();
            int num_vertices;

            bool collision = false;

            for (int i = 0; i < num_obstacles; i++) {
                num_vertices = environment_.obstacles[i].verticesCCW().size();
                for (int j = 0; j < num_vertices; j++) {
                    x3 = environment_.obstacles[i].verticesCCW()[j][0];
                    y3 = environment_.obstacles[i].verticesCCW()[j][1];
                    x4 = environment_.obstacles[i].verticesCCW()[j+1][0];
                    y4 = environment_.obstacles[i].verticesCCW()[j+1][1];
                    if (j == num_vertices-1) {
                        x4 = environment_.obstacles[i].verticesCCW()[0][0];
                        y4 = environment_.obstacles[i].verticesCCW()[0][1];
                    }
                    slope1 = (y2-y1)/(x2-x1);
                    slope2 = (y4-y3)/(x4-x3);
                    if (((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)) != 0) {
                        t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / ((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4));
                        u = ((x1-x3)*(y1-y2) - (y1-y3)*(x1-x2)) / ((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4));
                        if ((t >= 0 && t <= 1) && (u >= 0 && u <= 1)) collision = true;
                    } else if ((slope1 == slope2) || (slope1 == -slope2)) {
                        x1_y3 = slope2 * (x1 - x3) + y3;
                        x1_y4 = slope2 * (x1 - x4) + y4;
                        x2_y3 = slope2 * (x2 - x3) + y3;
                        x2_y4 = slope2 * (x2 - x4) + y4;
                        if ((y1 == x1_y3) || (y1 == x1_y4) || (y2 == x2_y3) || (y2 == x2_y4)) {
                            collision = true;
                        }
                    } 
                    slope1 = (y1-y0)/(1-x0);
                    slope2 = (y4-y3)/(x4-x3);
                    if (((x0-x1)*(y3-y4) - (y0-y1)*(x3-x4)) != 0) {
                        t = ((x0-x3)*(y3-y4) - (y0-y3)*(x3-x4)) / ((x0-x1)*(y3-y4) - (y0-y1)*(x3-x4));
                        u = ((x0-x3)*(y0-y1) - (y0-y3)*(x0-x1)) / ((x0-x1)*(y3-y4) - (y0-y1)*(x3-x4));
                        if ((t >= 0 && t <= 1) && (u >= 0 && u <= 1)) collision = true;
                    } else if ((slope1 == slope2) || (slope1 == -slope2)) {
                        x1_y3 = slope2 * (x0 - x3) + y3;
                        x1_y4 = slope2 * (x0 - x4) + y4;
                        x2_y3 = slope2 * (x1 - x3) + y3;
                        x2_y4 = slope2 * (x1 - x4) + y4;
                        if ((y0 == x1_y3) || (y0 == x1_y4) || (y1 == x2_y3) || (y1 == x2_y4)) {
                            collision = true;
                        }
                    } 
                    if (collision) return collision;
                }
            }
            return collision;
        }

        
    private:
        amp::MyLinkManipulator links_;
        amp::Environment2D environment_;
        amp::DenseArray2D<bool> denseArray;
};

}

#include "public/ConfigurationSpace_impl.h"
