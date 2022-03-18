#pragma once

#include "Agent.h"
#include "TurnTaskWorld.hpp"
#include "Task.hpp"

namespace xeres {

class TurnTask : public Task {

    public:
        TurnTask();
        ~TurnTask();

        TaskWorld* launchTask(Agent* agent, int environment) override;
};

}
