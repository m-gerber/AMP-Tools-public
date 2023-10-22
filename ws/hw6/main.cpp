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

    //for (uint32_t seed = 1; seed < 20; ++seed) {
        MyPointWFAlgo algo;
        MyManipWFAlgo algoManip;
        MyLinkManipulator link_manipulator_agent;
        Random2DManipulatorEnvironmentSpecification spec;
        MyGridCSpace2DConstructor m_c_space_constructor;
        // amp::Problem2D prob = EnvironmentTools::generateRandomManipulatorProblem(spec, link_manipulator_agent, seed);
        amp::Problem2D prob = HW6::getHW4Problem2();
        amp::ManipulatorState init_state = link_manipulator_agent.getConfigurationFromIK(prob.q_init);
        amp::ManipulatorState goal_state = link_manipulator_agent.getConfigurationFromIK(prob.q_goal);
        std::unique_ptr<amp::GridCSpace2D> grid_cspace = m_c_space_constructor.construct(link_manipulator_agent, prob);
            // Now that we have everything, we can call method to plan in C-space using the WaveFront algorithm
            // Note, we can use the `convert` overloads to easily go between ManipulatorState and ManipulatorState2Link
        amp::Path2D pathC = algoManip.planInCSpace(convert(init_state), convert(goal_state), *grid_cspace);
        amp::Path2D pathWS = algoManip.plan(link_manipulator_agent, prob);
        amp::Path2D init_goal;
        init_goal.waypoints.push_back(init_state); 
        init_goal.waypoints.push_back(goal_state); 
        bool success = HW6::checkLinkManipulatorPlan(pathWS, link_manipulator_agent, prob);
        // if(!success) {
            // LOG("Seed: " << seed);
            Visualizer::makeFigure(*grid_cspace, pathC);
            Visualizer::makeFigure(*grid_cspace, init_goal);
            Visualizer::makeFigure(prob, link_manipulator_agent, pathWS);
            Visualizer::showFigures();
        // }
    //}
    
    HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("mage7128@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple("hey there"), std::make_tuple());
    return 0;

}