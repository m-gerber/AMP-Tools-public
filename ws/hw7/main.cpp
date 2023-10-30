#include "main_helper.h"

int main(int argc, char** argv) {

    amp::RNG::seed(amp::RNG::randiUnbounded());

    amp::main_helper helper;

    bool plotE1    = 0;   // Plot exercise 1 questions
    bool trialsE1  = 0;   // Run XX trials for benchmark
    bool boxE1     = 0;   // Plot the boxplots for the trials
    bool smoothing = 0;   // Run smoothing on the path results
    int num_trials = 100; // Set the number of trials to run
    helper.runE1(plotE1, trialsE1, boxE1, smoothing, num_trials);

    bool plotE2 = 1;
    helper.runE2(plotE2);

    return 0;
}