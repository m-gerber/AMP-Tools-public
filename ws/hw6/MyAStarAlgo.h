#pragma once

#include "AMPCore.h"
#include "hw/HW6.h"
#include "MyGridCSpace.h"

#include <thread>

namespace amp {

class MyAStarAlgo : public amp::AStar {
    public:
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;
};

}