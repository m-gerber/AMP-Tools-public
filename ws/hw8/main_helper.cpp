#include "main_helper.h"

#include <iostream>
#include <string>
#include <iomanip>

void amp::main_helper::runE1(bool verbose, bool verbose2, bool verbose3, int num_trials) {

    amp::MultiAgentProblem2D prob;
    amp::MultiAgentPath2D path;
    
    if (verbose) {

        prob = amp::HW8::getWorkspace1(2);
        do {
            amp::MyCentralizedMultiAgentRRT central_checker;
            path = central_checker.plan(prob);
        } while(!amp::HW8::check(path, prob, true));
        amp::HW8::check(path, prob, false);
        amp::Visualizer::makeFigure(prob, path);
        amp::Visualizer::showFigures();

    }

    if (verbose2) {

        int m_test_vals[] = {2, 3, 4, 5, 6};
        int m;

        std::vector<std::string> labels;

        std::string title_tree;
        std::string title_comp;
        
        std::string xlabel = "m robots";
        std::string ylabel;

        int num_successes[] = {0, 0, 0, 0, 0};
        std::vector<double> vec_comp_time, vec_tree_size;
        std::list<std::vector<double>> comp_time, tree_size;

        for (int i = 0; i < 5; i++) {

            m = m_test_vals[i];

            prob = amp::HW8::getWorkspace1(m);

            labels.push_back(std::to_string(m));

            vec_tree_size.clear();
            vec_comp_time.clear();

            for (int j = 0; j < num_trials; j++) {

                amp::MyCentralizedMultiAgentRRT central_checker;

                auto start = std::chrono::high_resolution_clock::now();
                path = central_checker.plan(prob);
                auto stop = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
                if (path.agent_paths[m-1].waypoints.size() > 0) {
                    num_successes[i]++;
                    vec_tree_size.push_back(central_checker.returnTreeSize());
                    vec_comp_time.push_back(duration.count()/1000.0);
                }

            }
            comp_time.push_back(vec_comp_time);
            tree_size.push_back(vec_tree_size);
        }

        if (verbose3) {
            title_comp = "Computation Time vs. m Robots (Centralized";
            title_tree = "Tree Size vs. m Robots (Centralized";
            ylabel = "Computation Time (ms)";
            amp::Visualizer::makeBoxPlot(comp_time, labels, title_comp, xlabel, ylabel);
            ylabel = "Number of Elements in Tree";
            amp::Visualizer::makeBoxPlot(tree_size, labels, title_tree, xlabel, ylabel);

            amp::Visualizer::showFigures();
        }
    }
}

void amp::main_helper::runE2(bool verbose, bool verbose2, bool verbose3, int num_trials) {

    amp::MultiAgentProblem2D prob;
    amp::MultiAgentPath2D path;

    if (verbose) {

        amp::MultiAgentProblem2D prob;
        amp::MultiAgentPath2D path;

        prob = amp::HW8::getWorkspace1(2);
        do {
            amp::MyDecentralizedMultiAgentRRT decentral_checker;
            path = decentral_checker.plan(prob);
        } while(!amp::HW8::check(path, prob, true));
        amp::HW8::check(path, prob, false);
        amp::Visualizer::makeFigure(prob, path);
        amp::Visualizer::showFigures();

    }

    if (verbose2) {

        int m_test_vals[] = {2, 3, 4, 5, 6};
        int m;

        std::vector<std::string> labels;

        std::string title_tree;
        std::string title_comp;
        
        std::string xlabel = "m robots";
        std::string ylabel;

        int num_successes[] = {0, 0, 0, 0, 0};
        std::vector<double> vec_comp_time, vec_tree_size;
        std::list<std::vector<double>> comp_time, tree_size;

        for (int i = 0; i < 5; i++) {

            m = m_test_vals[i];

            prob = amp::HW8::getWorkspace1(m);

            labels.push_back(std::to_string(m));

            vec_tree_size.clear();
            vec_comp_time.clear();

            for (int j = 0; j < num_trials; j++) {

                amp::MyDecentralizedMultiAgentRRT decentral_checker;

                auto start = std::chrono::high_resolution_clock::now();
                path = decentral_checker.plan(prob);
                auto stop = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
                if (path.agent_paths[m-1].waypoints.size() > 0) {
                    num_successes[i]++;
                    vec_tree_size.push_back(decentral_checker.returnTreeSize());
                    vec_comp_time.push_back(duration.count()/1000.0);
                }

            }
            comp_time.push_back(vec_comp_time);
            tree_size.push_back(vec_tree_size);
        }

        if (verbose3) {
            title_comp = "Computation Time vs. m Robots (Decentralized";
            title_tree = "Tree Size vs. m Robots (Decentralized)";
            ylabel = "Computation Time (ms)";
            amp::Visualizer::makeBoxPlot(comp_time, labels, title_comp, xlabel, ylabel);
            ylabel = "Number of Elements in Tree";
            amp::Visualizer::makeBoxPlot(tree_size, labels, title_tree, xlabel, ylabel);

            amp::Visualizer::showFigures();
        }
    }
}