#include "main_helper.h"

int main(int argc, char** argv) {

    amp::RNG::seed(amp::RNG::randiUnbounded());

    amp::main_helper helper;

    bool plotE1 = 1;
    bool printE1 = 0;
    bool smoothing = 1;
    helper.runE1(plotE1, printE1, smoothing);

    bool plotE2 = 0;

    bool plotE3 = 0;

    

    return 0;
}