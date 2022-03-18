#pragma once

#include <random>
#include <vector>
#include <string>

#include "Agent.h"

#define TIMESTEP 0.02
#define EVTIME 20.0

using namespace std;

namespace xeres {

    struct simbox {
        double pos[3];
        double rot[4];
        double siz[3];
        string id;
    };

    simbox geomToBox(dGeomID t, const char* id="unnamed");

    std::string vectorToPythonListString(std::vector<int> theVector);

    double randomdouble();
    int random(int maximum);
    double randomGaussianDouble();

    extern std::random_device rd;
    extern std::mt19937 g;
    extern std::uniform_real_distribution<> dis;

} // namespace lwp
