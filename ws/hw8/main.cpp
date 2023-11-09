#include "MyCentralizedMultiAgentRRT.h"
#include "MyDecentralizedMultiAgentRRT.h"

int main(int argc, char** argv) {

    amp::RNG::seed(amp::RNG::randiUnbounded());

    amp::MyCentralizedMultiAgentRRT central_checker;
    amp::MyDecentralizedMultiAgentRRT decentral_checker;

    // Eigen::Vector2d point = Eigen::Vector2d({5,2});
    // double dist = central_checker.distance2obs(ws1, point);
    // // LOG("dist: " << dist);
    // amp::Visualizer::makeFigure(ws1);
    // amp::Visualizer::showFigures();
    // // PAUSE;

    amp::MultiAgentProblem2D prob = amp::HW8::getWorkspace1();
    amp::MultiAgentPath2D path;

    amp::MyCentralizedMultiAgentRRT pln;
    prob = amp::HW8::getWorkspace1(6);
    std::vector<std::vector<Eigen::Vector2d>> collision_states;
    do{
        path = pln.plan(prob);
        collision_states.clear();
    }while(!amp::HW8::check(path, prob, collision_states, true));
    amp::HW8::check(path, prob, collision_states, false);
    amp::Visualizer::makeFigure(prob, path, collision_states);
    // amp::Visualizer::makeFigure(prob, path);
    amp::Visualizer::showFigures();

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
    // amp::HW8::generateAndCheck(central_checker, true, 0u);

    // const std::vector<std::vector<Eigen::Vector2d>> ma_collision_states(3);

    // amp::Visualizer::makeFigure(ws1, path);
    // amp::Visualizer::showFigures();

    // amp::HW8::grade<amp::MyCentralizedMultiAgentRRT, amp::MyDecentralizedMultiAgentRRT>("mage7128@colorado.edu", argc, argv);

    return 0;
}