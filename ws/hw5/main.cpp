#include "AMPCore.h"

#include "hw/HW5.h"
#include "MyGDAlgo.h"

using namespace amp;

int main(int argc, char** argv) {

    double d_star = 5;
    double Q_star = 5;

    Problem2D problem = HW5::getWorkspace1();

    MyGDAlgo tester(d_star, Q_star);
    amp::Path2D test_path = tester.plan(problem);

    Eigen::Vector2d ans1, p1, p2;
    std::vector<Eigen::Vector2d> ans2;
    
    p1 = Eigen::Vector2d(5.5,0.0);
    p2 = Eigen::Vector2d(0.0,0.5);

    ans1 = tester.distance2point(p1,p2);
    ans2 = tester.distance2obs(problem, p1);

    std::cout << "\ndist, angle: " << ans1[0] << ", " << ans1[1] << std::endl;

    std::cout << "obstacles size: " << problem.obstacles.size() << std::endl;
    for (int i = 0; i < ans2.size(); i++) {
        std::cout << "dist, theta: " << ans2[i][0] << "," << ans2[i][1] << std::endl;
    }

    std::cout << std::endl;

    Visualizer::makeFigure(problem, test_path);
    Visualizer::showFigures();

    return 0;
}