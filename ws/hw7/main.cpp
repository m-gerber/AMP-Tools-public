#include "AMPCore.h"
#include "hw/HW7.h"


int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    amp::HW7::hint();

    return 0;
}