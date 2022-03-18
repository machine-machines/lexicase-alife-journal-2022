#pragma once

#include <string>

#include "Agent.h"
#include "WallTaskWorld.hpp"

using namespace std;

namespace xeres {

    class Task {

        public:

            unsigned int NUM_ENVIRONMENTS;
            string name;

            Task();
            ~Task();

            virtual TaskWorld* launchTask(Agent* agent, int environment) = 0;
    };

}
