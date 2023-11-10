#pragma once

#include "MyCentralizedMultiAgentRRT.h"
#include "MyDecentralizedMultiAgentRRT.h"

#include <iostream>
#include <chrono>

#include <thread>

namespace amp {

class main_helper {
    public:
        void runE1(bool verbose, bool verbose2, bool verbose3, int num_trials);
        void runE2(bool verbose, bool verbose2, bool verbose3, int num_trials);
};

}