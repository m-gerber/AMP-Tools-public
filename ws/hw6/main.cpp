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

    // MyPointWFAlgo pointAlgo;
    // amp::Problem2D pointProb1 = HW2::getWorkspace1();
    // amp::Path2D pointPath1 = pointAlgo.plan(pointProb1);
    // amp::Problem2D pointProb2 = HW2::getWorkspace2();
    // amp::Path2D pointPath2 = pointAlgo.plan(pointProb2);
    // LOG("Path length 1: " << pointPath1.length());
    // LOG("Path length 2: " << pointPath2.length());
    // Visualizer::makeFigure(pointProb1,pointPath1);
    // Visualizer::makeFigure(pointProb2,pointPath2);
    
    // Visualizer::showFigures();
    // PAUSE;

    MyManipWFAlgo manipAlgo;
    MyLinkManipulator link_manipulator_agent;
    MyGridCSpace2DConstructor m_c_space_constructor;
    amp::Problem2D manipProb1 = HW6::getHW4Problem1();
    amp::Problem2D manipProb2 = HW6::getHW4Problem2();
    amp::Problem2D manipProb3 = HW6::getHW4Problem3();
    amp::Path2D manipPathWS1 = manipAlgo.plan(link_manipulator_agent, manipProb1);
    amp::Path2D manipPathWS2 = manipAlgo.plan(link_manipulator_agent, manipProb2);
    amp::Path2D manipPathWS3 = manipAlgo.plan(link_manipulator_agent, manipProb3);
    Visualizer::makeFigure(manipProb1, link_manipulator_agent, manipPathWS1);
    Visualizer::makeFigure(manipProb2, link_manipulator_agent, manipPathWS2);
    Visualizer::makeFigure(manipProb3, link_manipulator_agent, manipPathWS3);
    
    amp::ManipulatorState manipProb1init_state = link_manipulator_agent.getConfigurationFromIK(manipProb1.q_init);
    amp::ManipulatorState manipProb1goal_state = link_manipulator_agent.getConfigurationFromIK(manipProb1.q_goal);
    std::unique_ptr<amp::GridCSpace2D> manipProb1grid_cspace = m_c_space_constructor.construct(link_manipulator_agent, manipProb1);
    amp::Path2D manipProb1pathC = manipAlgo.planInCSpace(convert(manipProb1init_state), convert(manipProb1goal_state), *manipProb1grid_cspace);
    Visualizer::makeFigure(*manipProb1grid_cspace,manipProb1pathC);
    amp::ManipulatorState manipProb2init_state = link_manipulator_agent.getConfigurationFromIK(manipProb2.q_init);
    amp::ManipulatorState manipProb2goal_state = link_manipulator_agent.getConfigurationFromIK(manipProb2.q_goal);
    std::unique_ptr<amp::GridCSpace2D> manipProb2grid_cspace = m_c_space_constructor.construct(link_manipulator_agent, manipProb2);
    amp::Path2D manipProb2pathC = manipAlgo.planInCSpace(convert(manipProb2init_state), convert(manipProb2goal_state), *manipProb2grid_cspace);
    Visualizer::makeFigure(*manipProb2grid_cspace,manipProb2pathC);
    amp::ManipulatorState manipProb3init_state = link_manipulator_agent.getConfigurationFromIK(manipProb3.q_init);
    amp::ManipulatorState manipProb3goal_state = link_manipulator_agent.getConfigurationFromIK(manipProb3.q_goal);
    std::unique_ptr<amp::GridCSpace2D> manipProb3grid_cspace = m_c_space_constructor.construct(link_manipulator_agent, manipProb3);
    amp::Path2D manipProb3pathC = manipAlgo.planInCSpace(convert(manipProb3init_state), convert(manipProb3goal_state), *manipProb3grid_cspace);
    Visualizer::makeFigure(*manipProb3grid_cspace,manipProb3pathC);

    Visualizer::showFigures();
    PAUSE;


    //for (uint32_t seed = 1; seed < 20; ++seed) {
        
        
        // // MyLinkManipulator link_manipulator_agent;
        // // Random2DManipulatorEnvironmentSpecification spec;
        // // MyGridCSpace2DConstructor m_c_space_constructor;
        
        // // amp::Problem2D prob = EnvironmentTools::generateRandomManipulatorProblem(spec, link_manipulator_agent, seed);
        
        // amp::ManipulatorState init_state = link_manipulator_agent.getConfigurationFromIK(prob.q_init);
        // amp::ManipulatorState goal_state = link_manipulator_agent.getConfigurationFromIK(prob.q_goal);
        // std::unique_ptr<amp::GridCSpace2D> grid_cspace = m_c_space_constructor.construct(link_manipulator_agent, prob);
        //     // Now that we have everything, we can call method to plan in C-space using the WaveFront algorithm
        //     // Note, we can use the `convert` overloads to easily go between ManipulatorState and ManipulatorState2Link
        // amp::Path2D pathC = manipAlgo.planInCSpace(convert(init_state), convert(goal_state), *grid_cspace);
        // amp::Path2D pathWS = manipAlgo.plan(link_manipulator_agent, prob);
        // amp::Path2D init_goal;
        // init_goal.waypoints.push_back(init_state); 
        // init_goal.waypoints.push_back(goal_state); 
        // bool success = HW6::checkLinkManipulatorPlan(pathWS, link_manipulator_agent, prob);
        // // if(!success) {
        //     // LOG("Seed: " << seed);
        //     Visualizer::makeFigure(*grid_cspace, pathC);
        //     Visualizer::makeFigure(*grid_cspace, init_goal);
        //     Visualizer::makeFigure(prob, link_manipulator_agent, pathWS);
        //     Visualizer::showFigures();
        // // }
    //}
    
    HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("mage7128@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple("hey there"), std::make_tuple());
    return 0;

}