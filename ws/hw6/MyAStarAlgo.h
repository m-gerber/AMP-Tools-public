#pragma once

#include "AMPCore.h"
#include "hw/HW6.h"
#include "MyGridCSpace.h"

#include <thread>

namespace amp {

struct Node_Info {
    double node = -1;
    double parent = -1;
    double g;
    double h;
    double f;

    Node_Info() {}

    Node_Info(double node, double parent, double g, double h, double f)
        : node(node)
        , parent(parent)
        , g(g)
        , h(h)
        , f(f) {}
};

struct Comparef {
    bool operator()(amp::Node_Info& n1, amp::Node_Info& n2) {
        return n1.f > n2.f;
    }
}; 

class MyAStarAlgo : public amp::AStar {
    public:
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;
};

}