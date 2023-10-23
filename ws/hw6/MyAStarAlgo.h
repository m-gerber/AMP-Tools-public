#pragma once

#include "AMPCore.h"
#include "hw/HW6.h"
#include "MyGridCSpace.h"

#include <thread>

namespace amp {

struct Node_Info {
    amp::Node node = -1;
    amp::Node parent = -1;
    double g;
    double h;
    double f;

    Node_Info() {}

    Node_Info(amp::Node node, amp::Node parent, double g, double h, double f)
        : node(node)
        , parent(parent)
        , g(g)
        , h(h)
        , f(f) {}
};
struct Compare_f {
    bool operator()(Node_Info const& n1, Node_Info const& n2) {
        return n1.f > n2.f;
    }
};

class MyAStarAlgo : public amp::AStar {
    public:
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;
};

}