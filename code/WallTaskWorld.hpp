#pragma once

#include "ode/ode.h"

#include "TaskWorld.hpp"

namespace xeres {

    class WallTaskWorld : public TaskWorld {

        public:

            WallTaskWorld(Agent* a, int environment);
            ~WallTaskWorld();

            void update() override;
            void callback(dGeomID o1, dGeomID o2);

            double getFitness() override;
            vector<simbox> getActiveBoxes() override;

            dWorldID d_world;
            dSpaceID d_space;
            dJointGroupID d_cngrp;

            dGeomID ground;
            dGeomID wall;

            double target[2];
            double simTime;
    };

}
