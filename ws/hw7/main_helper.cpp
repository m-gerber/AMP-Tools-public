#include "main_helper.h"

amp::Path2D amp::main_helper::pathSmoothing(amp::Path2D path, amp::Problem2D problem) {

    int num_waypoints, inda, ind1, indb, ind2;
    amp::MyPRM2D checker;

    for (int i = 0; i < 500; i++) {
        num_waypoints = path.waypoints.size();

        inda = amp::RNG::randi(0,num_waypoints);
        indb = amp::RNG::randi(0,num_waypoints);

        if (indb - inda > 1) {
            ind1 = inda;
            ind2 = indb;
        } else if (inda - indb > 1) {
            ind1 = indb;
            ind2 = inda;
        } else {
            continue;
        }

        if (!checker.lineIntersect(path.waypoints[ind1][0], path.waypoints[ind1][1], path.waypoints[ind2][0], path.waypoints[ind2][1], problem)) {
            path.waypoints.erase(path.waypoints.begin()+ind1+1,path.waypoints.begin()+ind2);
        }
    }
    return path;
}

void amp::main_helper::runE1(bool verbose, bool verbose2, bool verbose3, bool smoothing, int num_trials) {

    amp::Problem2D q1a  = amp::HW5::getWorkspace1();
    amp::Problem2D q1b1 = amp::HW2::getWorkspace1();
    amp::Problem2D q1b2 = amp::HW2::getWorkspace2();

    if (verbose) {

        amp::MyPRM2D prm_tester;

        amp::Path2D prm_path_testerq1a = prm_tester.plan(q1a);
        if (prm_path_testerq1a.waypoints.size() > 0) {
            if (smoothing) prm_path_testerq1a = pathSmoothing(prm_path_testerq1a, q1a);
            amp::Visualizer::makeFigure(q1a, prm_path_testerq1a);
        }
        
        amp::Path2D prm_path_testerq1b1 = prm_tester.plan(q1b1);
        if (prm_path_testerq1b1.waypoints.size() > 0) {
            if (smoothing) prm_path_testerq1b1 = pathSmoothing(prm_path_testerq1b1, q1b1);
            amp::Visualizer::makeFigure(q1b1, prm_path_testerq1b1);
        }

        amp::Path2D prm_path_testerq1b2 = prm_tester.plan(q1b2);
        if (prm_path_testerq1b2.waypoints.size() > 0) {
            if (smoothing) prm_path_testerq1b2 = pathSmoothing(prm_path_testerq1b2, q1b2);
            amp::Visualizer::makeFigure(q1b2, prm_path_testerq1b2);
        }
        
        amp::Visualizer::showFigures();

    }

    if (verbose2) {

        int    n_test_vals[] = {200, 200, 200, 200, 500, 500, 500, 500};
        double r_test_vals[] = {0.5,   1, 1.5,   2, 0.5,   1, 1.5,   2};

        std::vector<std::string> labels;

        std::string title_success = "Number of Successes (PRM)";
        std::string title_path = "Path Length (PRM)";
        std::string title_comp = "Computation Time (PRM) (ms)";
        
        std::string xlabel = "Trial";
        std::string ylabel = "Value";

        std::vector<double> vec_num_successesq1a, vec_path_lengthq1a, vec_comp_timeq1a;
        std::list<std::vector<double>> num_successesq1a, path_lengthq1a, comp_timeq1a;

        for (int i = 0; i < 8; i++) {

            labels.push_back("n = " + std::to_string(n_test_vals[i]) + ", r = " + std::to_string(r_test_vals[i]));

            vec_num_successesq1a.clear();
            vec_path_lengthq1a.clear();
            vec_comp_timeq1a.clear();
            
            amp::MyPRM2D q1_tester(n_test_vals[i], r_test_vals[i]);

            for (int j = 0; j < num_trials; j++) {
                auto start = std::chrono::high_resolution_clock::now();
                amp::Path2D prm_path_testerq1a = q1_tester.plan(q1a);
                if (prm_path_testerq1a.waypoints.size() > 0) {
                    if (smoothing) prm_path_testerq1a = pathSmoothing(prm_path_testerq1a, q1a);
                    vec_num_successesq1a.push_back(1);
                    vec_path_lengthq1a.push_back(prm_path_testerq1a.length());
                } else {
                    vec_num_successesq1a.push_back(0);
                    vec_path_lengthq1a.push_back(0);
                }
                auto stop = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
                vec_comp_timeq1a.push_back(duration.count()/1000.0);
            }
            num_successesq1a.push_back(vec_num_successesq1a);
            path_lengthq1a.push_back(vec_path_lengthq1a);
            comp_timeq1a.push_back(vec_comp_timeq1a);


            LOG("Round " << i << " for Exercise 2.(a) of Homework 5 with n = " << n_test_vals[i] << " and r = " << r_test_vals[i] <<" for " << num_trials << " trials " << (smoothing ? "with " : "without ") << "path smoothing:");
            // LOG("");
            // LOG(" ----- SUCCESSES ------");
            // LOG("num: " << num_successes << " (" << static_cast<double>(num_successes)/static_cast<double>(num_trials)*100.0 << "%)");
            // LOG("");
            // LOG(" ---- PATH LENGTH -----");
            // LOG("total:     " << path_length);
            // LOG("per trial: " << path_length/static_cast<double>(num_successes));
            // LOG("");
            // LOG(" -- COMPUTATION TIME --");
            // LOG("total:     " << duration.count()/1000.0 << " ms");
            // LOG("per trial: " << duration.count()/static_cast<double>(num_trials)/1000.0 << " ms");
            // LOG("");
            // LOG("");

        }

        std::vector<double> vec_num_successesq1b1, vec_path_lengthq1b1, vec_comp_timeq1b1;
        std::list<std::vector<double>> num_successesq1b1, path_lengthq1b1, comp_timeq1b1;

        for (int i = 0; i < 8; i++) {

            vec_num_successesq1b1.clear();
            vec_path_lengthq1b1.clear();
            vec_comp_timeq1b1.clear();
            
            amp::MyPRM2D q1_tester(n_test_vals[i], r_test_vals[i]);

            for (int j = 0; j < num_trials; j++) {
                auto start = std::chrono::high_resolution_clock::now();
                amp::Path2D prm_path_testerq1a = q1_tester.plan(q1a);
                if (prm_path_testerq1a.waypoints.size() > 0) {
                    if (smoothing) prm_path_testerq1a = pathSmoothing(prm_path_testerq1a, q1a);
                    vec_num_successesq1b1.push_back(1);
                    vec_path_lengthq1b1.push_back(prm_path_testerq1a.length());
                } else {
                    vec_num_successesq1b1.push_back(0);
                    vec_path_lengthq1b1.push_back(0);
                }
                auto stop = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
                vec_comp_timeq1b1.push_back(duration.count()/1000.0);
            }
            num_successesq1b1.push_back(vec_num_successesq1b1);
            path_lengthq1b1.push_back(vec_path_lengthq1b1);
            comp_timeq1b1.push_back(vec_comp_timeq1b1);

            LOG("Round " << i << " for Exercise 2.(b) of Homework 2 with n = " << n_test_vals[i] << " and r = " << r_test_vals[i] <<" for " << num_trials << " trials " << (smoothing ? "with " : "without ") << "path smoothing:");
            // LOG("");
            // LOG(" ----- SUCCESSES ------");
            // LOG("num: " << num_successes << " (" << static_cast<double>(num_successes)/static_cast<double>(num_trials)*100.0 << "%)");
            // LOG("");
            // LOG(" ---- PATH LENGTH -----");
            // LOG("total:     " << path_length);
            // LOG("per trial: " << path_length/static_cast<double>(num_successes));
            // LOG("");
            // LOG(" -- COMPUTATION TIME --");
            // LOG("total:     " << duration.count()/1000.0 << " ms");
            // LOG("per trial: " << duration.count()/static_cast<double>(num_trials)/1000.0 << " ms");
            // LOG("");
            // LOG("");

        }

        std::vector<double> vec_num_successesq1b2, vec_path_lengthq1b2, vec_comp_timeq1b2;
        std::list<std::vector<double>> num_successesq1b2, path_lengthq1b2, comp_timeq1b2;

        for (int i = 0; i < 8; i++) {

            vec_num_successesq1b2.clear();
            vec_path_lengthq1b2.clear();
            vec_comp_timeq1b2.clear();
            
            amp::MyPRM2D q1_tester(n_test_vals[i], r_test_vals[i]);

            for (int j = 0; j < num_trials; j++) {
                auto start = std::chrono::high_resolution_clock::now();
                amp::Path2D prm_path_testerq1a = q1_tester.plan(q1a);
                if (prm_path_testerq1a.waypoints.size() > 0) {
                    if (smoothing) prm_path_testerq1a = pathSmoothing(prm_path_testerq1a, q1a);
                    vec_num_successesq1b2.push_back(1);
                    vec_path_lengthq1b2.push_back(prm_path_testerq1a.length());
                } else {
                    vec_num_successesq1b2.push_back(0);
                    vec_path_lengthq1b2.push_back(0);
                }
                auto stop = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
                vec_comp_timeq1b2.push_back(duration.count()/1000.0);
            }
            num_successesq1b2.push_back(vec_num_successesq1b2);
            path_lengthq1b2.push_back(vec_path_lengthq1b2);
            comp_timeq1b2.push_back(vec_comp_timeq1b2);

            LOG("Round " << i << " for Exercise 2.(b) of Homework 2 with n = " << n_test_vals[i] << " and r = " << r_test_vals[i] <<" for " << num_trials << " trials " << (smoothing ? "with " : "without ") << "path smoothing:");
            // LOG("");
            // LOG(" ----- SUCCESSES ------");
            // LOG("num: " << num_successes << " (" << static_cast<double>(num_successes)/static_cast<double>(num_trials)*100.0 << "%)");
            // LOG("");
            // LOG(" ---- PATH LENGTH -----");
            // LOG("total:     " << path_length);
            // LOG("per trial: " << path_length/static_cast<double>(num_successes));
            // LOG("");
            // LOG(" -- COMPUTATION TIME --");
            // LOG("total:     " << duration.count()/1000.0 << " ms");
            // LOG("per trial: " << duration.count()/static_cast<double>(num_trials)/1000.0 << " ms");
            // LOG("");
            // LOG("");

        }

        if (verbose3) {
            amp::Visualizer::makeBoxPlot(num_successesq1a,  labels, title_success, xlabel, ylabel);
            amp::Visualizer::makeBoxPlot(path_lengthq1a,    labels, title_path,    xlabel, ylabel);
            amp::Visualizer::makeBoxPlot(comp_timeq1a,      labels, title_comp,    xlabel, ylabel);

            amp::Visualizer::makeBoxPlot(num_successesq1b1, labels, title_success, xlabel, ylabel);
            amp::Visualizer::makeBoxPlot(path_lengthq1b1,   labels, title_path,    xlabel, ylabel);
            amp::Visualizer::makeBoxPlot(comp_timeq1b1,     labels, title_comp,    xlabel, ylabel);

            amp::Visualizer::makeBoxPlot(num_successesq1b2, labels, title_success, xlabel, ylabel);
            amp::Visualizer::makeBoxPlot(path_lengthq1b2,   labels, title_path,    xlabel, ylabel);
            amp::Visualizer::makeBoxPlot(comp_timeq1b2,     labels, title_comp,    xlabel, ylabel);

            amp::Visualizer::showFigures();
        }

    }

}

void amp::main_helper::runE2(bool verbose, bool verbose2, bool verbose3, bool smoothing, int num_trials) {

    amp::Problem2D q1a  = amp::HW5::getWorkspace1();
    amp::Problem2D q1b1 = amp::HW2::getWorkspace1();
    amp::Problem2D q1b2 = amp::HW2::getWorkspace2();

    if (verbose) {

        amp::MyGoalBiasRRT2D rrt_tester;

        amp::Path2D rrt_path_testerq1a = rrt_tester.plan(q1a);
        if (rrt_path_testerq1a.waypoints.size() > 0) {
            if (smoothing) rrt_path_testerq1a = pathSmoothing(rrt_path_testerq1a, q1a);
            amp::Visualizer::makeFigure(q1a, rrt_path_testerq1a);
        }
        
        amp::Path2D rrt_path_testerq1b1 = rrt_tester.plan(q1b1);
        if (rrt_path_testerq1b1.waypoints.size() > 0) {
            if (smoothing) rrt_path_testerq1b1 = pathSmoothing(rrt_path_testerq1b1, q1b1);
            amp::Visualizer::makeFigure(q1b1, rrt_path_testerq1b1);
        }

        amp::Path2D rrt_path_testerq1b2 = rrt_tester.plan(q1b2);
        if (rrt_path_testerq1b2.waypoints.size() > 0) {
            if (smoothing) rrt_path_testerq1b2 = pathSmoothing(rrt_path_testerq1b2, q1b2);
            amp::Visualizer::makeFigure(q1b2, rrt_path_testerq1b2);
        }
        
        amp::Visualizer::showFigures();

    }

    if (verbose2) {

        std::vector<std::string> labels;

        std::string title_success;
        std::string title_path;
        std::string title_comp;
        labels.push_back("Default Case");
        
        std::string xlabel = "Trial";
        std::string ylabel = "Value";

        std::vector<double> vec_num_successesq1a, vec_path_lengthq1a, vec_comp_timeq1a;
        std::list<std::vector<double>> num_successesq1a, path_lengthq1a, comp_timeq1a;
        
        amp::MyGoalBiasRRT2D q2_tester;

        for (int j = 0; j < num_trials; j++) {
            auto start = std::chrono::high_resolution_clock::now();
            amp::Path2D prm_path_testerq1a = q2_tester.plan(q1a);
            if (prm_path_testerq1a.waypoints.size() > 0) {
                if (smoothing) prm_path_testerq1a = pathSmoothing(prm_path_testerq1a, q1a);
                vec_num_successesq1a.push_back(1);
                vec_path_lengthq1a.push_back(prm_path_testerq1a.length());
            } else {
                vec_num_successesq1a.push_back(0);
                vec_path_lengthq1a.push_back(0);
            }
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
            vec_comp_timeq1a.push_back(duration.count()/1000.0);
        }
        num_successesq1a.push_back(vec_num_successesq1a);
        path_lengthq1a.push_back(vec_path_lengthq1a);
        comp_timeq1a.push_back(vec_comp_timeq1a);

        std::vector<double> vec_num_successesq1b1, vec_path_lengthq1b1, vec_comp_timeq1b1;
        std::list<std::vector<double>> num_successesq1b1, path_lengthq1b1, comp_timeq1b1;

        for (int j = 0; j < num_trials; j++) {
            auto start = std::chrono::high_resolution_clock::now();
            amp::Path2D prm_path_testerq1b1 = q2_tester.plan(q1b1);
            if (prm_path_testerq1b1.waypoints.size() > 0) {
                if (smoothing) prm_path_testerq1b1 = pathSmoothing(prm_path_testerq1b1, q1b1);
                vec_num_successesq1b1.push_back(1);
                vec_path_lengthq1b1.push_back(prm_path_testerq1b1.length());
            } else {
                vec_num_successesq1b1.push_back(0);
                vec_path_lengthq1b1.push_back(0);
            }
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
            vec_comp_timeq1b1.push_back(duration.count()/1000.0);
        }
        num_successesq1b1.push_back(vec_num_successesq1b1);
        path_lengthq1b1.push_back(vec_path_lengthq1b1);
        comp_timeq1b1.push_back(vec_comp_timeq1b1);

        std::vector<double> vec_num_successesq1b2, vec_path_lengthq1b2, vec_comp_timeq1b2;
        std::list<std::vector<double>> num_successesq1b2, path_lengthq1b2, comp_timeq1b2;

        for (int j = 0; j < num_trials; j++) {
            auto start = std::chrono::high_resolution_clock::now();
            amp::Path2D prm_path_testerq1b2 = q2_tester.plan(q1b2);
            if (prm_path_testerq1b2.waypoints.size() > 0) {
                if (smoothing) prm_path_testerq1b2 = pathSmoothing(prm_path_testerq1b2, q1b2);
                vec_num_successesq1b2.push_back(1);
                vec_path_lengthq1b2.push_back(prm_path_testerq1b2.length());
            } else {
                vec_num_successesq1b2.push_back(0);
                vec_path_lengthq1b2.push_back(0);
            }
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
            vec_comp_timeq1b2.push_back(duration.count()/1000.0);
        }
        num_successesq1b2.push_back(vec_num_successesq1b2);
        path_lengthq1b2.push_back(vec_path_lengthq1b2);
        comp_timeq1b2.push_back(vec_comp_timeq1b2);

        if (verbose3) {
            title_success = "Number of Successes Env. 1 (RRT)";
            title_path = "Path Length Env. 1 (RRT)";
            title_comp = "Computation Time Env. 1 (RRT) (ms)";
            amp::Visualizer::makeBoxPlot(num_successesq1a,  labels, title_success, xlabel, ylabel);
            amp::Visualizer::makeBoxPlot(path_lengthq1a,    labels, title_path,    xlabel, ylabel);
            amp::Visualizer::makeBoxPlot(comp_timeq1a,      labels, title_comp,    xlabel, ylabel);

            title_success = "Number of Successes Env. 2 (RRT)";
            title_path = "Path Length Env. 2 (RRT)";
            title_comp = "Computation Time Env. 2 (RRT) (ms)";
            amp::Visualizer::makeBoxPlot(num_successesq1b1, labels, title_success, xlabel, ylabel);
            amp::Visualizer::makeBoxPlot(path_lengthq1b1,   labels, title_path,    xlabel, ylabel);
            amp::Visualizer::makeBoxPlot(comp_timeq1b1,     labels, title_comp,    xlabel, ylabel);

            title_success = "Number of Successes Env. 3 (RRT)";
            title_path = "Path Length Env. 3 (RRT)";
            title_comp = "Computation Time Env. 3 (RRT) (ms)";
            amp::Visualizer::makeBoxPlot(num_successesq1b2, labels, title_success, xlabel, ylabel);
            amp::Visualizer::makeBoxPlot(path_lengthq1b2,   labels, title_path,    xlabel, ylabel);
            amp::Visualizer::makeBoxPlot(comp_timeq1b2,     labels, title_comp,    xlabel, ylabel);

            amp::Visualizer::showFigures();
        }

    }

}