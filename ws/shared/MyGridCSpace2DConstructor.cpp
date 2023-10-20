#include "MyGridCSpace2DConstructor.h"

std::unique_ptr<amp::GridCSpace2D> amp::MyGridCSpace2DConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    std::unique_ptr<amp::MyGridCSpace> ptr(new MyGridCSpace(100,100,-M_PI,M_PI,-M_PI,M_PI));
    ptr->buildLinkCSpace(manipulator.getLinkLengths(), env);

    return ptr;
}