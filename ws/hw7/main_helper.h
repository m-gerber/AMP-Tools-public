#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "hw/HW7.h"
#include "MyPRMAlgo.h"

#include <iostream>
#include <chrono>

#include <thread>

namespace amp {

class main_helper {
    public:
        amp::Path2D pathSmoothing(amp::Path2D path, amp::Problem2D problem);
        void runE1(bool verbose, bool verbose2, bool smoothing);
};

}