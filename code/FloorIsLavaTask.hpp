#pragma once

#include "Agent.h"
#include "FloorIsLavaTaskWorld.hpp"
#include "Task.hpp"

namespace xeres {

class FloorIsLavaTask : public Task {

    public:
        FloorIsLavaTask();
        ~FloorIsLavaTask();

        TaskWorld* launchTask(Agent* agent, int environment) override;
};

}