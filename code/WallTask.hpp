#pragma once

#include "Agent.h"
#include "WallTaskWorld.hpp"
#include "Task.hpp"

namespace xeres {

    class WallTask : public Task {

        public:
            WallTask();
            ~WallTask();

            TaskWorld* launchTask(Agent* agent, int environment) override;
    };

}
