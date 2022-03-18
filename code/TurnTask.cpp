#include "TurnTask.hpp"

namespace xeres {

    TurnTask::TurnTask() {
        NUM_ENVIRONMENTS = 100;
        name = "turn";
    }

    TurnTask::~TurnTask() {
    }

    TaskWorld* TurnTask::launchTask(Agent* agent, int environment) {
        return new TurnTaskWorld(agent, environment);

    }

}
