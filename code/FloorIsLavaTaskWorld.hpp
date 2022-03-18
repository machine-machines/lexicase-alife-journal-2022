#pragma once

#include "ode/ode.h"

#include "TaskWorld.hpp"
#include "Util.hpp"

namespace xeres {

    class FloorIsLavaTaskWorld : public TaskWorld {

        public:

            FloorIsLavaTaskWorld(Agent* a, int environment);
            ~FloorIsLavaTaskWorld();

            void update() override;
            void callback(dGeomID o1, dGeomID o2);

            double getFitness() override;
            vector<simbox> getActiveBoxes() override;
            dWorldID d_world;
            dSpaceID d_space;
            dJointGroupID d_cngrp;

            dGeomID ground;

            double target[2];
            double simTime;

            int total_ticks_in_the_air;
            int total_ticks;
    };

}
