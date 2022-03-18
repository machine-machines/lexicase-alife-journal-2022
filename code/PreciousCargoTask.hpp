#pragma once

#include "Agent.h"
#include "PreciousCargoTaskWorld.hpp"
#include "Task.hpp"

namespace xeres {

class PreciousCargoTask : public Task {

    public:
        PreciousCargoTask();
        ~PreciousCargoTask();

        TaskWorld* launchTask(Agent* agent, int environment) override;
};

}