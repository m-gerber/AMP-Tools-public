#include "main_helper.h"

int main(int argc, char** argv) {

    // Eigen::Vector2d point = Eigen::Vector2d({5,2});
    // double dist = central_checker.distance2obs(ws1, point);
    // // LOG("dist: " << dist);
    // amp::Visualizer::makeFigure(ws1);
    // amp::Visualizer::showFigures();
    // // PAUSE;

    // amp::MultiAgentProblem2D prob = amp::HW8::getWorkspace1();
    // amp::MultiAgentPath2D path;

    // prob = amp::HW8::getWorkspace1(3);
    // // std::vector<std::vector<Eigen::Vector2d>> collision_states;
    // do{
    //     amp::MyCentralizedMultiAgentRRT pln;
    //     path = pln.plan(prob);
    //     // collision_states.clear();
    // }while(amp::HW8::check(path, prob, true));
    // amp::HW8::check(path, prob, false);
    // amp::Visualizer::makeFigure(prob, path);
    // // amp::Visualizer::makeFigure(prob, path);
    // amp::Visualizer::showFigures();

    // int num_successes = 0;

    // for (int i = 0; i < 50; i++) {
    //     path = central_checker.plan(ws1);
    //     num_successes += amp::HW8::check(path,ws1,true);
    //     if (!amp::HW8::check(path,ws1,false)) {
    //         amp::Visualizer::makeFigure(ws1, path);
    //         amp::Visualizer::showFigures();
    //     }
    // }

    // LOG("num_successes: " << num_successes);
    // path = central_checker.plan(ws1);
    // amp::HW8::check(path,ws1,true);

    // while (1) {
    // amp::MyCentralizedMultiAgentRRT pln;
    // amp::MultiAgentPath2D ma_path;
    // amp::MultiAgentProblem2D ma_prob;
    // std::vector<std::vector<Eigen::Vector2d>> collision_states;
    // amp::HW8::generateAndCheck(central_checker, ma_path, ma_prob, collision_states, true, 0u);
    // Eigen::Vector2d point = Eigen::Vector2d({3,3});
    // double dist = central_checker.distance2obs(ma_prob, point);
    // LOG("dist: " << dist);
    // LOG("in poly: " << central_checker.inPolygon(point[0],point[1]));
    // // do{
    // //     ma_path = pln.plan(ma_prob);
    // //     collision_states.clear();
    // // }while(amp::HW8::check(ma_path, ma_prob, true));
    // amp::Visualizer::makeFigure(ma_prob, ma_path);
    // amp::Visualizer::makeFigure(ma_prob);
    // amp::Visualizer::showFigures();
    // }


    // const std::vector<std::vector<Eigen::Vector2d>> ma_collision_states(3);

    // amp::Visualizer::makeFigure(ws1, path);
    // amp::Visualizer::showFigures();

    amp::RNG::seed(amp::RNG::randiUnbounded());

    amp::main_helper helper;

    int num_trials = 100; // Set the number of trials to run

    bool plotE1    = 0;   // Plot exercise 1 questions
    bool trialsE1  = 1;   // Run XX trials for benchmark
    bool boxE1     = 1;   // Plot performance statistics
    helper.runE1(plotE1, trialsE1, boxE1, num_trials);

    bool plotE2    = 0;   // Plot exercise 1 questions
    bool trialsE2  = 1;   // Run XX trials for benchmark
    bool boxE2     = 1;   // Plot performance statistics
    helper.runE2(plotE2, trialsE2, boxE2, num_trials);

    // amp::HW8::grade<amp::MyCentralizedMultiAgentRRT, amp::MyDecentralizedMultiAgentRRT>("mage7128@colorado.edu", argc, argv);

    return 0;
}