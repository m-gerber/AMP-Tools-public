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

    // MyManipWFAlgo manipAlgo;
    // MyLinkManipulator link_manipulator_agent;
    // MyGridCSpace2DConstructor m_c_space_constructor;
    // amp::Problem2D manipProb1 = HW6::getHW4Problem1();
    // amp::Problem2D manipProb2 = HW6::getHW4Problem2();
    // amp::Problem2D manipProb3 = HW6::getHW4Problem3();
    // amp::Path2D manipPathWS1 = manipAlgo.plan(link_manipulator_agent, manipProb1);
    // amp::Path2D manipPathWS2 = manipAlgo.plan(link_manipulator_agent, manipProb2);
    // amp::Path2D manipPathWS3 = manipAlgo.plan(link_manipulator_agent, manipProb3);
    // Visualizer::makeFigure(manipProb1, link_manipulator_agent, manipPathWS1);
    // Visualizer::makeFigure(manipProb2, link_manipulator_agent, manipPathWS2);
    // Visualizer::makeFigure(manipProb3, link_manipulator_agent, manipPathWS3);
    
    // amp::ManipulatorState manipProb1init_state = link_manipulator_agent.getConfigurationFromIK(manipProb1.q_init);
    // amp::ManipulatorState manipProb1goal_state = link_manipulator_agent.getConfigurationFromIK(manipProb1.q_goal);
    // std::unique_ptr<amp::GridCSpace2D> manipProb1grid_cspace = m_c_space_constructor.construct(link_manipulator_agent, manipProb1);
    // amp::Path2D manipProb1pathC = manipAlgo.planInCSpace(convert(manipProb1init_state), convert(manipProb1goal_state), *manipProb1grid_cspace);
    // Visualizer::makeFigure(*manipProb1grid_cspace,manipProb1pathC);
    // amp::ManipulatorState manipProb2init_state = link_manipulator_agent.getConfigurationFromIK(manipProb2.q_init);
    // amp::ManipulatorState manipProb2goal_state = link_manipulator_agent.getConfigurationFromIK(manipProb2.q_goal);
    // std::unique_ptr<amp::GridCSpace2D> manipProb2grid_cspace = m_c_space_constructor.construct(link_manipulator_agent, manipProb2);
    // amp::Path2D manipProb2pathC = manipAlgo.planInCSpace(convert(manipProb2init_state), convert(manipProb2goal_state), *manipProb2grid_cspace);
    // Visualizer::makeFigure(*manipProb2grid_cspace,manipProb2pathC);
    // amp::ManipulatorState manipProb3init_state = link_manipulator_agent.getConfigurationFromIK(manipProb3.q_init);
    // amp::ManipulatorState manipProb3goal_state = link_manipulator_agent.getConfigurationFromIK(manipProb3.q_goal);
    // std::unique_ptr<amp::GridCSpace2D> manipProb3grid_cspace = m_c_space_constructor.construct(link_manipulator_agent, manipProb3);
    // amp::Path2D manipProb3pathC = manipAlgo.planInCSpace(convert(manipProb3init_state), convert(manipProb3goal_state), *manipProb3grid_cspace);
    // Visualizer::makeFigure(*manipProb3grid_cspace,manipProb3pathC);

    // Visualizer::showFigures();
    // PAUSE;

    amp::MyAStarAlgo aStarAlgo;

    // amp::ShortestPathProblem pathEx3 = HW6::getEx3SPP();
    // amp::LookupSearchHeuristic hueristicEx3 =  HW6::getEx3Heuristic();
    // amp::AStar::GraphSearchResult result = aStarAlgo.search(pathEx3, hueristicEx3);

    // std::cout << "path: " << std::endl;
    // for (const amp::Node& node : result.node_path) {
    //     std::cout << node << " ";
    // }
    // std::cout << std::endl;
    // PAUSE;

    // amp::HW6::checkGraphSearchResult(result, pathEx3, hueristicEx3, true);
    // PAUSE;

    // uint32_t n_nodes = 20;
    // double min_edge_weight = 0.0;
    // double max_edge_weight = 10.0;
    // uint32_t max_outgoing_edges_per_node = 5;
    // double connectedness = 0.5;

    // int iter = 0;

    // for (int i = 0; i < 20; i++) {
    //     std::cout << "iter: " << ++iter <<std::endl;
    //     uint32_t seed = i;
    //     amp::ShortestPathProblem randPath = amp::GraphTools::generateRandomSPP(n_nodes, min_edge_weight, max_edge_weight, max_outgoing_edges_per_node, connectedness, seed);
    //     amp::AStar::GraphSearchResult result1 = aStarAlgo.search(randPath, amp::SearchHeuristic());
    //     bool success = amp::HW6::checkGraphSearchResult(result1, randPath, amp::SearchHeuristic(), true);
    //     PAUSE;
    // }
    // uint32_t seed = 0u;
    // amp::ShortestPathProblem randPath = amp::GraphTools::generateRandomSPP(n_nodes, min_edge_weight, max_edge_weight, max_outgoing_edges_per_node, connectedness, seed);
    // amp::AStar::GraphSearchResult result1 = aStarAlgo.search(randPath, amp::SearchHeuristic());
    // amp::HW6::checkGraphSearchResult(result1, randPath, amp::SearchHeuristic(), true);
    // // HW6::generateAndCheck(aStarAlgo, result1, randPath, true, seed);
    // // std::cout << "path: " << std::endl;
    // // for (const amp::Node& node : result.node_path) {
    // //     std::cout << node << " ";
    // // }
    // // std::cout << std::endl;
    // PAUSE;
    
    HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("mage7128@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple(), std::make_tuple());
    return 0;

}