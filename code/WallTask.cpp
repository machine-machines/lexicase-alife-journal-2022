#include "WallTask.hpp"

namespace xeres {

    WallTask::WallTask() {
        NUM_ENVIRONMENTS = 100;
        name = "wall";
    }

    WallTask::~WallTask() {
    }

    TaskWorld* WallTask::launchTask(Agent* agent, int environment) {
        return new WallTaskWorld(agent, environment);

    }

}
