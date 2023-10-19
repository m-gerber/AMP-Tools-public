#include "AMPCore.h"
#include "hw/HW6.h"
#include "HelpfulClass.h"
#include "MyGridCSpace.h"
#include "MyGridCSpace2DConstructor.h"
#include "MyPointWFAlgo.h"
#include "MyManipWFAlgo.h"
#include "MyAStarAlgo.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    HW6 HW4p1;
    MyPointWFAlgo pointAlgo;
    std::unique_ptr<amp::GridCSpace2D> ptr = pointAlgo.constructDiscretizedWorkspace(HW4p1.getHW4Problem1());
    amp::Path2D path = pointAlgo.planInCSpace(HW4p1.getHW4Problem1().q_init, HW4p1.getHW4Problem1().q_goal, *ptr);

    Visualizer::makeFigure(HW4p1.getHW4Problem1(), path);
    Visualizer::showFigures();

    // Path2D path = MyPointWFAlgo::planInCSpace(q_init, q_goal, grid_cspace);

    // amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("mage7128@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple("hey there"), std::make_tuple());
    return 0;

}