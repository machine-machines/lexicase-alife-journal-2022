#include "TurnTaskWorld.hpp"

#include <vector>

#include "ode/ode.h"

#include "Agent.h"
#include "Util.hpp"

namespace xeres {

    TurnTaskWorld::TurnTaskWorld(Agent* a, int environment) {

        double angle = (0.01 * environment * M_PI) - (M_PI / 2.0);

        target[0] = 2.0 * cos(angle); target[1] = 2.0 * sin(angle);

        simTime = 0.0;

        dInitODE2(0);

        d_world = dWorldCreate();
        d_space = dSimpleSpaceCreate(0);
        d_cngrp = dJointGroupCreate(0);

        dWorldSetERP(d_world, 0.2);
        dWorldSetCFM(d_world, 0.00005);
        dWorldSetGravity(d_world, 0, 0, -1.2);

        ground = dCreatePlane(d_space, 0, 0, 1, 0);

        a->reset(d_world, d_space);

        a->target[0] = target[0]; a->target[1] = target[1];

        agents.push_back(a);
    }

    TurnTaskWorld::~TurnTaskWorld() {

        for(auto a: agents) { a->deactivate(); }

        if(ground) dGeomDestroy(ground);

        if(d_space) dSpaceDestroy(d_space);
        if(d_cngrp) dJointGroupDestroy(d_cngrp);
        if(d_world) dWorldDestroy(d_world);

        dCloseODE();

    }
    void TurnTaskWorld::callback(dGeomID o1, dGeomID o2) {
        if( o1!=ground && o2!=ground ) return;

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

    void ttw_nearCallback (void *data, dGeomID o1, dGeomID o2)
    {
        ((TurnTaskWorld*)data)->callback(o1, o2);
    }

    void TurnTaskWorld::update() {

        agents[0]->tick(TIMESTEP);

        dSpaceCollide(d_space,this,&ttw_nearCallback);
        dWorldStep(d_world,TIMESTEP);
        dJointGroupEmpty(d_cngrp);
    }

    double TurnTaskWorld::getFitness() {
        bool reached_target = false;
        double dist = 0.0;
        while( !reached_target && simTime < EVTIME ) {
            update();
            double dx = (target[0]-agents[0]->position[0]);
            double dy = (target[1]-agents[0]->position[1]);

            dist = 0.0 - sqrt( dx*dx + dy*dy );

            reached_target = (dist > -0.2);
            simTime += TIMESTEP;
        }
        agents[0]->deactivate();
        return min(0.0, dist - (-0.2)); // Fitness
    }

    vector<simbox> TurnTaskWorld::getActiveBoxes() {
        std::vector<simbox> r;

        for( int b=0; b<agents[0]->body->numBodies; b++) {
            dGeomID t = dBodyGetFirstGeom(agents[0]->body->bodies[b]);
            r.push_back(geomToBox(t,"body"));
        }

        simbox targ;
        targ.pos[0]=target[0];
        targ.pos[1]=target[1];
        targ.pos[2]=0.2;
        dQuaternion q; dQSetIdentity(q);
        targ.rot[0]=q[0]; targ.rot[1]=q[1]; targ.rot[2]=q[2]; targ.rot[3]=q[3];
        targ.siz[0] = targ.siz[1] = targ.siz[2] = 0.2;
        targ.id=string("target");
        r.push_back(targ);
        return r;
    }


} // namespace
