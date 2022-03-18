#include "PreciousCargoTaskWorld.hpp"

#include <vector>

#include "ode/ode.h"

#include "Agent.h"
#include "Util.hpp"

namespace xeres {

    PreciousCargoTaskWorld::PreciousCargoTaskWorld(Agent* a, int environment) {

        double angle = 0.0;

        target[0] = 2.0 * cos(angle); target[1] = 2.0 * sin(angle);

        simTime = 0.0;
        done = 0;
        cargo_is_free = 0;
        time_elapsed = 0.0;

        dInitODE2(0);

        d_world = dWorldCreate();
        d_space = dSimpleSpaceCreate(0);
        d_cngrp = dJointGroupCreate(0);

        dWorldSetERP(d_world, 0.2);
        dWorldSetCFM(d_world, 0.00005);
        dWorldSetGravity(d_world, 0, 0, -1.2);    

        ground = dCreatePlane(d_space, 0, 0, 1, 0);

        // create the cargo and cargo_box as per
        // ancient cargo experiment
        // mass = 0.01 * environment (0.0 to 1.0kg)
        //
        cargo = dBodyCreate(d_world);
        cargo_box = dCreateBox(d_space, 0.1, 0.1, 0.1);
        dMass m;
        dMassSetBoxTotal(&m, 0.01*environment, 0.1, 0.1, 0.1);
        dBodySetMass( cargo, &m );
	    dGeomSetBody( cargo_box, cargo );
	    dBodySetPosition( cargo, 0, 0, 0.55 );

        a->reset(d_world, d_space);
        
       cargo_joint = dJointCreateFixed(d_world, 0);
       dJointAttach(cargo_joint, cargo, a->body->bodies[0]);
       dJointSetFixed(cargo_joint);

        a->target[0] = target[0]; a->target[1] = target[1];

        agents.push_back(a);
    }

    PreciousCargoTaskWorld::~PreciousCargoTaskWorld() {

        for(auto a: agents) { a->deactivate(); }

        if(ground) dGeomDestroy(ground);
        if(cargo_joint) dJointDestroy(cargo_joint);
        if(cargo_box) dGeomDestroy(cargo_box);
        if(cargo) dBodyDestroy(cargo);

        if(d_space) dSpaceDestroy(d_space);
        if(d_cngrp) dJointGroupDestroy(d_cngrp);
        if(d_world) dWorldDestroy(d_world);

        dCloseODE();

    }
    void PreciousCargoTaskWorld::callback(dGeomID o1, dGeomID o2) {

        this->done = this->done || ((o1==cargo_box || o2==cargo_box) && (o1==ground || o2==ground));
        dBodyID b1 = dGeomGetBody(o1), b2 = dGeomGetBody(o2);
        if (b1 && b2 && dAreConnected(b1,b2) ) return;
        const int MAX_CONTACTS = 8; dContact contacts[MAX_CONTACTS];
        int numCollisions = dCollide(o1,o2,MAX_CONTACTS,&contacts[0].geom,sizeof(dContact));
        for (int c=0;c<numCollisions;c++) {
            dSurfaceParameters& s=contacts[c].surface;
            s.mode = dContactApprox1;// | dContactFDir1;

            s.mu = 2.0; //treatment.world_mu;

            dJointID cj = dJointCreateContact (d_world,d_cngrp,&(contacts[c]));
            dJointAttach (cj,b1,b2);

        }
    }

    void pctw_nearCallback (void *data, dGeomID o1, dGeomID o2)
    {
        ((PreciousCargoTaskWorld*)data)->callback(o1, o2);
    }

    void PreciousCargoTaskWorld::update() {

        agents[0]->tick(TIMESTEP);
        time_elapsed += TIMESTEP;

        if(!cargo_is_free && time_elapsed >= 1.0) {
            cargo_is_free = true;
            dJointDestroy(cargo_joint);
            cargo_joint = 0;
        }
        
        dSpaceCollide(d_space,this,&pctw_nearCallback);
                
        dWorldStep(d_world,TIMESTEP);
        dJointGroupEmpty(d_cngrp);
    }

    double PreciousCargoTaskWorld::getFitness() {

        while( simTime < EVTIME && !done ) {
            update();
            simTime += TIMESTEP;
        }

        agents[0]->deactivate();

        // Fitness for precious cargo is
        // the amount of time before the box touches the floor
        return simTime;
    }

    vector<simbox> PreciousCargoTaskWorld::getActiveBoxes() {
        std::vector<simbox> r;
        r.push_back(geomToBox(cargo_box, "cargo"));

        for( int b=0; b<agents[0]->body->numBodies; b++) {
            dGeomID t = dBodyGetFirstGeom(agents[0]->body->bodies[b]);
            r.push_back(geomToBox(t,"body"));
        }
        return r;
    }

} // namespace
