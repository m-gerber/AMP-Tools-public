#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "hw/HW7.h"
#include "MyPRMAlgo.h"
#include "MyGoalBiasRRT2D.h"

#include <iostream>
#include <chrono>

#include <thread>

namespace amp {

class main_helper {
    public:
        amp::Path2D pathSmoothing(amp::Path2D path, amp::Problem2D problem);
        void runE1(bool verbose, bool verbose2, bool verbose3, bool smoothing, int num_trials);
        void runE2(bool verbose);
};

}