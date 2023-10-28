#pragma once

#include "AMPCore.h"

namespace amp {

class MyPRMAlgo : : public PRM {

    public:
        amp::Path2D plan(const amp::Problem2D& problem) override;

        amp::Path planND(const:: amp::Problem) {

        }


};

}