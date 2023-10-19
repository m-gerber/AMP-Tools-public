#include "AMPCore.h"
#include "hw/HW6.h"
#include "hw/HW2.h"
#include "HelpfulClass.h"
#include "MyGridCSpace.h"
#include "MyGridCSpace2DConstructor.h"
#include "MyPointWFAlgo.h"
#include "MyManipWFAlgo.h"
#include "MyAStarAlgo.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    Problem2D HW2p1 = HW2::getWorkspace1();
    Problem2D HW2p2 = HW2::getWorkspace2();
    Problem2D HW4p1 = HW6::getHW4Problem1();
    Problem2D HW4p2 = HW6::getHW4Problem2();
    Problem2D HW4p3 = HW6::getHW4Problem2();
    MyPointWFAlgo pointAlgo;
    std::unique_ptr<amp::GridCSpace2D> ptr2p1 = pointAlgo.constructDiscretizedWorkspace(HW2p1);
    std::unique_ptr<amp::GridCSpace2D> ptr2p2 = pointAlgo.constructDiscretizedWorkspace(HW2p2);
    amp::Path2D path2p1 = pointAlgo.planInCSpace(HW2p1.q_init, HW2p1.q_goal, *ptr2p1);
    amp::Path2D path2p2 = pointAlgo.planInCSpace(HW2p2.q_init, HW2p2.q_goal, *ptr2p2);

    std::cout << "path: " << path2p1.waypoints[0][0] << ", " << path2p1.waypoints[0][1] << std::endl;
    std::cout << "path: " << path2p1.waypoints.back()[0] << ", " << path2p1.waypoints.back()[1] << std::endl;

    std::cout << "path: " << path2p2.waypoints[0][0] << ", " << path2p2.waypoints[0][1] << std::endl;
    std::cout << "path: " << path2p2.waypoints.back()[0] << ", " << path2p2.waypoints.back()[1] << std::endl;

    Visualizer::makeFigure(HW2p1, path2p1);
    Visualizer::makeFigure(HW2p2, path2p2);
    // Visualizer::makeFigure(*ptr2p1);
    Visualizer::showFigures();

    MyPointWFAlgo gradePointAlgo;
    MyManipWFAlgo gradeManipWFAlgo;
    MyAStarAlgo gradeAStarAlgo;

    HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("mage7128@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple("hey there"), std::make_tuple());
    return 0;

}