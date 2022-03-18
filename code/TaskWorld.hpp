#pragma once

#include <vector>

#include "Agent.h"
#include "Util.hpp"

namespace xeres {

    class TaskWorld {
        public:

            TaskWorld();
            virtual ~TaskWorld();

            virtual double getFitness();
            virtual vector<simbox> getActiveBoxes() = 0;
            virtual void update() = 0;

            std::vector<Agent*> agents;

    };

}
