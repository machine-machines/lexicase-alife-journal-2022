#include "FloorIsLavaTask.hpp"

namespace xeres {

    FloorIsLavaTask::FloorIsLavaTask() {
        NUM_ENVIRONMENTS = 100;
        name = "lava";
    }

    FloorIsLavaTask::~FloorIsLavaTask() {
    }

    TaskWorld* FloorIsLavaTask::launchTask(Agent* agent, int environment) {
        return new FloorIsLavaTaskWorld(agent, environment);

    }

}