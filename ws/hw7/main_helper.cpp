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

void amp::main_helper::runE1(bool verbose, bool verbose2, bool smoothing) {

    amp::Problem2D q1a  = amp::HW5::getWorkspace1();
    amp::Problem2D q1b1 = amp::HW2::getWorkspace1();
    amp::Problem2D q1b2 = amp::HW2::getWorkspace2();

    if (verbose) {

        amp::MyPRM2D prm_tester;

        amp::Path2D path_testerq1a = prm_tester.plan(q1a);
        if (path_testerq1a.waypoints.size() > 0) {
            if (smoothing) path_testerq1a = pathSmoothing(path_testerq1a, q1a);
            amp::Visualizer::makeFigure(q1a, path_testerq1a);
        }
        
        amp::Path2D path_testerq1b1 = prm_tester.plan(q1b1);
        if (path_testerq1b1.waypoints.size() > 0) {
            if (smoothing) path_testerq1b1 = pathSmoothing(path_testerq1b1, q1b1);
            amp::Visualizer::makeFigure(q1b1, path_testerq1b1);
        }

        amp::Path2D path_testerq1b2 = prm_tester.plan(q1b2);
        if (path_testerq1b2.waypoints.size() > 0) {
            if (smoothing) path_testerq1b2 = pathSmoothing(path_testerq1b2, q1b2);
            amp::Visualizer::makeFigure(q1b2, path_testerq1b2);
        }
        
        amp::Visualizer::showFigures();

    }

    if (verbose2) {

        int    n_test_vals[] = {200, 200, 200, 200, 500, 500, 500, 500};
        double r_test_vals[] = {0.5,   1, 1.5,   2, 0.5,   1, 1.5,   2};

        int num_trials = 100;
        int num_successes;
        double path_length, comp_time;

        // std::list<std::vector<double>> num_successes, path_length, comp_time;
        // static void makeBoxPlot(const std::list<std::vector<double>>& data_sets, const std::vector<std::string>& labels, 
        //                         const std::string& title = std::string(), const std::string& xlabel = std::string(), const std::string& ylabel = std::string());

        for (int i = 0; i < 8; i++) {

            num_successes = 0;
            path_length = 0;
            amp::MyPRM2D q1_tester(n_test_vals[i], r_test_vals[i]);
            amp::Path2D path_testerq1a = q1_tester.plan(q1a);

            auto start = std::chrono::high_resolution_clock::now();

            for (int j = 0; j < num_trials; j++) {
                amp::Path2D path_testerq1a = q1_tester.plan(q1a);
                if (path_testerq1a.waypoints.size() > 0) {
                    if (smoothing) path_testerq1a = pathSmoothing(path_testerq1a, q1a);
                    num_successes++;
                    path_length += path_testerq1a.length();
                }
            }

            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

            LOG("Round " << i << " for Exercise 2.(a) of Homework 5 with n = " << n_test_vals[i] << " and r = " << r_test_vals[i] <<" for " << num_trials << " trials " << (smoothing ? "with " : "without ") << "path smoothing:");
            LOG("");
            LOG(" ----- SUCCESSES ------");
            LOG("num: " << num_successes << " (" << static_cast<double>(num_successes)/static_cast<double>(num_trials)*100.0 << "%)");
            LOG("");
            LOG(" ---- PATH LENGTH -----");
            LOG("total:     " << path_length);
            LOG("per trial: " << path_length/static_cast<double>(num_successes));
            LOG("");
            LOG(" -- COMPUTATION TIME --");
            LOG("total:     " << duration.count()/1000.0 << " ms");
            LOG("per trial: " << duration.count()/static_cast<double>(num_trials)/1000.0 << " ms");
            LOG("");
            LOG("");

        }

        for (int i = 0; i < 8; i++) {

            num_successes = 0;
            path_length = 0;
            amp::MyPRM2D q1_tester(n_test_vals[i], r_test_vals[i]);
            amp::Path2D path_testerq1b1 = q1_tester.plan(q1b1);

            auto start = std::chrono::high_resolution_clock::now();

            for (int j = 0; j < num_trials; j++) {
                amp::Path2D path_testerq1b1 = q1_tester.plan(q1b1);
                if (path_testerq1b1.waypoints.size() > 0) {
                    if (smoothing) path_testerq1b1 = pathSmoothing(path_testerq1b1, q1b1);
                    num_successes++;
                    path_length += path_testerq1b1.length();
                }
            }

            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

            LOG("Round " << i << " for Exercise 2.(b) of Homework 5 with n = " << n_test_vals[i] << " and r = " << r_test_vals[i] <<" for " << num_trials << " trials " << (smoothing ? "with " : "without ") << "path smoothing:");
            LOG("");
            LOG(" ----- SUCCESSES ------");
            LOG("num: " << num_successes << " (" << static_cast<double>(num_successes)/static_cast<double>(num_trials)*100.0 << "%)");
            LOG("");
            LOG(" ---- PATH LENGTH -----");
            LOG("total:     " << path_length);
            LOG("per trial: " << path_length/static_cast<double>(num_successes));
            LOG("");
            LOG(" -- COMPUTATION TIME --");
            LOG("total:     " << duration.count()/1000.0 << " ms");
            LOG("per trial: " << duration.count()/static_cast<double>(num_trials)/1000.0 << " ms");
            LOG("");
            LOG("");

        }

        for (int i = 0; i < 8; i++) {

            num_successes = 0;
            path_length = 0;
            amp::MyPRM2D q1_tester(n_test_vals[i], r_test_vals[i]);
            amp::Path2D path_testerq1b2 = q1_tester.plan(q1b2);

            auto start = std::chrono::high_resolution_clock::now();

            for (int j = 0; j < num_trials; j++) {
                amp::Path2D path_testerq1b2 = q1_tester.plan(q1b2);
                if (path_testerq1b2.waypoints.size() > 0) {
                    if (smoothing) path_testerq1b2 = pathSmoothing(path_testerq1b2, q1b2);
                    num_successes++;
                    path_length += path_testerq1b2.length();
                }
            }

            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

            LOG("Round " << i << " for Exercise 2.(b) of Homework 2 with n = " << n_test_vals[i] << " and r = " << r_test_vals[i] <<" for " << num_trials << " trials " << (smoothing ? "with " : "without ") << "path smoothing:");
            LOG("");
            LOG(" ----- SUCCESSES ------");
            LOG("num: " << num_successes << " (" << static_cast<double>(num_successes)/static_cast<double>(num_trials)*100.0 << "%)");
            LOG("");
            LOG(" ---- PATH LENGTH -----");
            LOG("total:     " << path_length);
            LOG("per trial: " << path_length/static_cast<double>(num_successes));
            LOG("");
            LOG(" -- COMPUTATION TIME --");
            LOG("total:     " << duration.count()/1000.0 << " ms");
            LOG("per trial: " << duration.count()/static_cast<double>(num_trials)/1000.0 << " ms");
            LOG("");
            LOG("");

        }

    }

}