#pragma once

#include "ode/ode.h"

#include "TaskWorld.hpp"
#include "Util.hpp"

namespace xeres {

    class PreciousCargoTaskWorld : public TaskWorld {

        public:

            PreciousCargoTaskWorld(Agent* a, int environment);
            ~PreciousCargoTaskWorld();

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

            unsigned int done;
            unsigned int cargo_is_free;
            double time_elapsed;

            dJointID cargo_joint;
            dBodyID cargo;
            dGeomID cargo_box;

    };

}
