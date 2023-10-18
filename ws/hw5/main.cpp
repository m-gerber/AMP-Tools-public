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
    
    Problem2D problem0 = HW5::getWorkspace1();
    Problem2D problem1 = HW2::getWorkspace1();
    Problem2D problem2 = HW2::getWorkspace2();

    MyGDAlgo tester(xi, d_star, eta, Q_star);
    Path2D test_path0 = tester.plan(problem0);
    Path2D test_path1 = tester.plan(problem1);
    Path2D test_path2 = tester.plan(problem2);

    std::cout << "HW5 workspace 1 path length: " << test_path0.length() << std::endl;
    std::cout << "HW2 workspace 1 path length: " << test_path1.length() << std::endl;
    std::cout << "HW2 workspace 2 path length: " << test_path2.length() << std::endl;

    Visualizer::makeFigure(problem0, test_path0);
    Visualizer::makeFigure(problem1, test_path1);
    Visualizer::makeFigure(problem2, test_path2);
    Visualizer::showFigures();

    MyGDAlgo algo(xi, d_star, eta, Q_star);
    // HW5::grade(algo, "mage7128@colorado.edu", argc, argv);

    return 0;
}