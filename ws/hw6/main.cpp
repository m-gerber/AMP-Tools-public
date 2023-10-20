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

    // MyManipWFAlgo manipAlgo;

    Problem2D HW4p1 = HW6::getHW4Problem1();
    Problem2D HW4p2 = HW6::getHW4Problem2();
    Problem2D HW4p3 = HW6::getHW4Problem3();

    // int grid_discretization = 100;

    // std::vector<double> q3_link_lengths = {1, 1};
    // amp::MyLinkManipulator q3_links(q3_link_lengths);

    // amp::MyGridCSpace grid1(grid_discretization,grid_discretization,HW4p1.x_min,HW4p1.x_max,HW4p1.y_min,HW4p1.y_max);
    // amp::MyGridCSpace grid2(grid_discretization,grid_discretization,HW4p2.x_min,HW4p2.x_max,HW4p2.y_min,HW4p2.y_max);
    // amp::MyGridCSpace grid3(grid_discretization,grid_discretization,HW4p3.x_min,HW4p3.x_max,HW4p3.y_min,HW4p3.y_max);

    // grid1.buildLinkCSpace(q3_links, HW4p1);
    // grid2.buildLinkCSpace(q3_links, HW4p2);
    // grid3.buildLinkCSpace(q3_links, HW4p3);

    // amp::Visualizer::makeFigure(grid1);
    // amp::Visualizer::makeFigure(grid2);
    // amp::Visualizer::makeFigure(grid3);
    // amp::Visualizer::showFigures();

    // amp::ManipulatorState IK_angles;
    std::vector<double> link_lengths2 = {1,1};
    amp::MyLinkManipulator q2_links2(link_lengths2);

    // MyManipWFAlgo algoManip2;

    // Path2D pathManip = algoManip2.plan(q2_links2,HW4p3);
    // amp::Visualizer::makeFigure(HW4p3, q2_links2, pathManip);
    // Visualizer::showFigures();

    
    // std::cout << "IK_angles: " << IK_angles.size() << std::endl;

    // q2_links2.getConfigurationFromIK(Eigen::Vector2d(2,0));
    // q2_links2.makeRotationMatrix(0,0,0);
    
    // std::cout <<"tesating: "<< q3_links.getConfigurationFromIK(HW4p1.q_init) << std::endl;
    // ManipulatorState HW4p1_goal = q3_links.getConfigurationFromIK(HW4p1.q_goal);
    // ManipulatorState HW4p1_init = q3_links.getConfigurationFromIK(HW4p1.q_init);
    // ManipulatorState HW4p2_goal = q3_links.getConfigurationFromIK(HW4p2.q_goal);
    // ManipulatorState HW4p2_init = q3_links.getConfigurationFromIK(HW4p2.q_init);
    // ManipulatorState HW4p3_goal = q3_links.getConfigurationFromIK(HW4p3.q_goal);
    // ManipulatorState HW4p3_init = q3_links.getConfigurationFromIK(HW4p3.q_init);

    // amp::Path2D path4p1 = manipAlgo.planInCSpace(HW4p1.q_init, HW4p1.q_goal, grid1);
    // amp::Path2D path4p2 = manipAlgo.planInCSpace(HW4p2.q_init, HW4p2.q_goal, grid2);
    // amp::Path2D path4p3 = manipAlgo.planInCSpace(HW4p3.q_init, HW4p3.q_goal, grid3);
    // Visualizer::makeFigure(grid1, path4p1);
    // Visualizer::makeFigure(grid2, path4p2);
    // Visualizer::makeFigure(grid3, path4p3);

    // HW6::checkLinkManipulatorPlan(path4p1, link_manipulator_agent, HW4p1, true);

    //HW6::checkPointAgentPlan(path2p2, HW2p2, true);
    //HW6::checkPointAgentPlan(path2p2, HW2p2, true);

    //for (uint32_t seed = 1; seed < 20; ++seed) {
        MyPointWFAlgo algo;
        MyManipWFAlgo algoManip;
        MyLinkManipulator link_manipulator_agent;
        Random2DManipulatorEnvironmentSpecification spec;
        MyGridCSpace2DConstructor m_c_space_constructor;
        // amp::Problem2D prob = EnvironmentTools::generateRandomManipulatorProblem(spec, link_manipulator_agent, seed);
        amp::Problem2D prob = HW6::getHW4Problem3();
        amp::ManipulatorState init_state = link_manipulator_agent.getConfigurationFromIK(prob.q_init);
        amp::ManipulatorState goal_state = link_manipulator_agent.getConfigurationFromIK(prob.q_goal);
        std::unique_ptr<amp::GridCSpace2D> grid_cspace = m_c_space_constructor.construct(link_manipulator_agent, prob);
            // Now that we have everything, we can call method to plan in C-space using the WaveFront algorithm
            // Note, we can use the `convert` overloads to easily go between ManipulatorState and ManipulatorState2Link
        //amp::Path2D path = algoManip.planInCSpace(convert(init_state), convert(goal_state), *grid_cspace);
        amp::Path2D path = algoManip.plan(link_manipulator_agent, prob);
        bool success = HW6::checkLinkManipulatorPlan(path, link_manipulator_agent, prob);
        if(!success) {
            // LOG("Failed seed: " << seed);
            // Visualizer::makeFigure(*grid_cspace, path);
            Visualizer::makeFigure(prob, link_manipulator_agent, path);
            Visualizer::showFigures();
        }
    //}


    // Visualizer::makeFigure(*ptr2p1);
    // Visualizer::showFigures();

    MyPointWFAlgo gradePointAlgo;
    MyManipWFAlgo gradeManipWFAlgo;
    MyAStarAlgo gradeAStarAlgo;

    // HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("mage7128@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple("hey there"), std::make_tuple());
    return 0;

}