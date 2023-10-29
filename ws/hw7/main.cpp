#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "hw/HW7.h"
#include "MyPRMAlgo.h"


int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // amp::HW7::hint();

    amp::MyPRM2D prm_tester;

    amp::Problem2D q1a = amp::HW5::getWorkspace1();
    amp::Path2D path_testerq1a = prm_tester.plan(q1a);
    if (path_testerq1a.waypoints.size() > 0) amp::Visualizer::makeFigure(q1a, path_testerq1a);

    // amp::Problem2D q1b1 = amp::HW2::getWorkspace1();
    // amp::Problem2D q1b2 = amp::HW2::getWorkspace2();
    // amp::Path2D path_testerq1b1 = prm_tester.plan(q1b1);
    // if (path_testerq1b1.waypoints.size() > 0) amp::Visualizer::makeFigure(q1b1, path_testerq1b1);
    // amp::Path2D path_testerq1b2 = prm_tester.plan(q1b2);
    // if (path_testerq1b2.waypoints.size() > 0) amp::Visualizer::makeFigure(q1b2, path_testerq1b2);

    amp::Visualizer::showFigures();

    return 0;
}