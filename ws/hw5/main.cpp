#include "AMPCore.h"

#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MyGDAlgo.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    double xi  = 0.5;
    double d_star = 2;
    double Q_star = 0.25;
    double eta = 1;
    
    Problem2D problem  = HW5::getWorkspace1();
    Problem2D problem2 = HW2::getWorkspace1();

    MyGDAlgo tester(xi, d_star, eta, Q_star);
    Path2D test_path  = tester.plan(problem);
    Path2D test_path2 = tester.plan(problem2);

    // std::cout << "test: " << tester.distance2point(Eigen::Vector2d(0.85,0.85), Eigen::Vector2d(1,1)) << std::endl;

    // PAUSE;

    Eigen::Vector2d loc = Eigen::Vector2d(0.85,0.85);
    std::vector<Eigen::Vector2d> points = tester.distance2obs(problem2, loc);
    Eigen::Vector2d rep = tester.Grad_U_rep(problem2, loc);


    std::cout << "rep_x, rep_y: " << rep[0] << ", " << rep[1] << std::endl;

    for (int i = 0; i < points.size(); i++) {
        std::cout << "dist, angle: " << points[i][0] << ", " << points[i][1] << std::endl;
    }

    std::cout << std::endl;

    Random2DEnvironmentSpecification spec;
    spec.max_obstacle_region_radius = 5.0;
    spec.n_obstacles = 10;
    spec.path_clearance = 0.01;
    spec.d_sep = 0.01;

    //Randomly generate the environment;
    Problem2D problem3 = EnvironmentTools::generateRandom(spec); // Random environment

    Path2D test_path3 = tester.plan(problem3);

    Visualizer::makeFigure(problem, test_path);
    Visualizer::showFigures();

    Visualizer::makeFigure(problem2, test_path2);
    Visualizer::showFigures();

    MyGDAlgo algo;
    // HW5::grade(algo, "mage7128@colorado.edu", argc, argv);

    return 0;
}