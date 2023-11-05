#include "main_helper.h"
#include "MyPRMAlgo.h"
#include "MyGoalBiasRRT2D.h"

int main(int argc, char** argv) {

    amp::RNG::seed(amp::RNG::randiUnbounded());

    amp::HW7::hint();

    // amp::main_helper helper;

    // bool smoothing = 1;   // Run smoothing on the path results
    // int num_trials = 100; // Set the number of trials to run

    // bool plotE1    = 0;   // Plot exercise 1 questions
    // bool trialsE1  = 1;   // Run XX trials for benchmark
    // bool printE1   = 0;   // Print performance statistics
    // bool boxE1     = 1;   // Plot the boxplots for the trials
    // helper.runE1(plotE1, trialsE1, printE1, boxE1, smoothing, num_trials);

    // bool plotE2    = 0;   // Plot exercise 1 questions
    // bool trialsE2  = 0;   // Run XX trials for benchmark
    // bool boxE2     = 0;   // Plot the boxplots for the trials
    // helper.runE2(plotE2, trialsE2, boxE2, smoothing, num_trials);

    // amp::HW7::grade<amp::MyPRM2D, amp::MyGoalBiasRRT2D>("mage7128@colorado.edu", argc, argv);

    return 0;
}