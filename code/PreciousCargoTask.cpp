#include "PreciousCargoTask.hpp"

namespace xeres {

    PreciousCargoTask::PreciousCargoTask() {
        NUM_ENVIRONMENTS = 100;
        name = "precious_cargo";
    }

    PreciousCargoTask::~PreciousCargoTask() {
    }

    TaskWorld* PreciousCargoTask::launchTask(Agent* agent, int environment) {
        return new PreciousCargoTaskWorld(agent, environment);

    }

}