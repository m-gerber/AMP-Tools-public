#include "AMPCore.h"
#include "hw/HW6.h"
#include "HelpfulClass.h"
#include "MyGridCSpace.h"
#include "MyGridCSpace2DConstructor.h"
#include "MyPointWFAlgo.h"
#include "MyManipWFAlgo.h"
#include "MyAStarAlgo.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("mage7128@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple("hey there"), std::make_tuple());
    return 0;

}