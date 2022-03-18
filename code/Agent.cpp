#include "Agent.h"
#include "Util.hpp"

namespace xeres {

#define MUTRATE 2.0

void GeneticString::init()
{	
}

std::ostream& operator<<(std::ostream& os, const GeneticString& gs)
{
        char buf[42];
        for( int locus=0; locus<gs.length; locus++ )
        {
                sprintf(buf, "%a ", gs.data[locus]);
                os << buf;
        }
        os << std::endl;
        return os;
}
std::istream& operator>>(std::istream& is, GeneticString& gs)
{
	for( int locus=0; locus<gs.length; locus++ )
	{
		std::string s;
		double d;
		is >> s;
		if( !sscanf(s.c_str(), "%la ", &d ) )
			std::cerr << "Error reading numeric data for genetic string." << std::endl;

		gs.data[locus] = d;
	}
	return is;
}
	bool operator==(const GeneticString& gs1, const GeneticString& gs2 )
	{
		for( int c=0; c<gs1.length; c++ )
		{
			if( gs1.data[c] != gs2.data[c] )
			   return false;
		}
		return true;
	}
	bool operator!=(const GeneticString& gs1, const GeneticString& gs2 ) { return !operator==(gs1, gs2); }

GeneticString::GeneticString()
{
	mutation_rate = MUTRATE;
}

/* Generation-zero contructor */
GeneticString::GeneticString( ControllerType type )
{
	mutation_rate = MUTRATE;

	if( type==FeedForward )
	{
		int locus=0;
		for( ; locus<length; locus++ )
			data[locus] = -1.0 + randomdouble() * 2.0;
	}
	else
	{
		printf("Unknown controller!\n");
		exit(-1);
	}
}

/* Two-parent single-point crossover with mutation */
GeneticString::GeneticString( GeneticString *parent1, GeneticString *parent2 )
{
	mutation_rate = MUTRATE;
	int x_point = random(length);
	
	for( int locus=0; locus<length; locus++ )
	{
		if( locus < x_point ) data[locus] = parent1->data[locus];
		else data[locus] = parent2->data[locus];
		
		if( randomdouble() < mutation_rate/length ) data[locus] += randomGaussianDouble();		
	}
}

/* Parthenogenesis (mutation only) */
GeneticString::GeneticString( GeneticString *parent1 )
{	
	mutation_rate = MUTRATE;
	for( int locus=0; locus<length; locus++ )
	{
		data[locus] = parent1->data[locus];
		if( randomdouble() < mutation_rate/length ) data[locus] += randomGaussianDouble();
	}
}

GeneticString::~GeneticString()
{
}

double GeneticString::distance( GeneticString& comp ) {
	double sum=0.0;
	for( int locus=0; locus<length; locus++ ) {
		sum += (data[locus] - comp.data[locus]) * (data[locus] - comp.data[locus]);
	}
	return sqrt(sum);
}

Controller::~Controller() 
{
}

FFPController::FFPController( GeneticString genetic_template )
{
	int min_length = (NUM_INPUTS*NETWORK_SIZE)+(NETWORK_SIZE*NETWORK_SIZE);
	
	if( genetic_template.length < min_length )
	{
		printf("ERROR: gene string not long enough for controller. Exiting...\n");
		exit(-1);
	}
	
	int locus = 0;
	
	for( int input=0; input < NUM_INPUTS; input++ )
	{
		for( int hidden=0; hidden < NETWORK_SIZE; hidden++ )
		{
			locus = (input*NETWORK_SIZE)+hidden;
			input_hidden_weight[input][hidden] = genetic_template.data[ locus ];
		}
	}
	for( int hidden=0; hidden < NETWORK_SIZE; hidden++ )
	{
		for( int output=0; output < NETWORK_SIZE; output++ )
		{
			locus = NUM_INPUTS*NETWORK_SIZE + (hidden*NETWORK_SIZE)+output;
			hidden_output_weight[hidden][output] = genetic_template.data[ locus ];
		}
	}
	for( int c=0; c<NETWORK_SIZE; c++ )
		outputs[c] = 0.0;
	
	for( int c=0; c<NUM_INPUTS; c++ )
		inputs[c] = 0.0;	
}

FFPController::~FFPController()
{
}

void FFPController::step()
{
	/* Weighted sum inputs to each hidden. Apply tanh to each hidden.
	 * Weighted sum hidden to output. Apply sigmoid [0,1] to each output. Done. */
	 
	 double hidden_nodes[ NETWORK_SIZE ];

	 double sum = 0.0;
	 
	 for( int i=0; i<NETWORK_SIZE; i++ ) // For each hidden...
	 {
		sum = 0.0;
		for( int j=0; j<NUM_INPUTS; j++ ) // For each input...
		{
			sum += inputs[j] * input_hidden_weight[j][i];
		}
		/* Hyperbolic tan keeps negative values in hidden layer */
		hidden_nodes[i] = tanh( sum );
	}
	
	for( int i=0; i<NETWORK_SIZE; i++ ) // For each output...
	{
		sum = 0.0;
		for( int j=0; j<NETWORK_SIZE; j++ ) // For each hidden...
		{
			sum += hidden_nodes[j] * hidden_output_weight[j][i];
		}
		/* 1/(1+e^-x) squashes sigmoid output to [0,1] for outputs */
		outputs[i] = 1.0 / (1.0 + exp( -sum ) );
	}
}

Body::Body( dWorldID physics_handle, dSpaceID space_handle )
{
	/* Zero variables */
	for( int joint=0; joint<8; joint++ )
	{
		joint_current_angles[joint][0] = joint_current_angles[joint][1] = 0.0;
		joint_target_angles[joint][0] = joint_target_angles[joint][1] = 0.0;
		joint_torques[joint][0] = joint_torques[joint][1] = 0.0;
	}
	
	dMass mass; dMatrix3 rotation;
	
	dBodyID thigh0, thigh1, thigh2, thigh3;
	
	dGeomID thigh0_g, thigh1_g, thigh2_g, thigh3_g;
	dGeomID shin0_g, shin1_g, shin2_g, shin3_g;
	
	dJointID hip0, hip1, hip2, hip3, knee0, knee1, knee2, knee3, earjoint1, earjoint2;
	
	double headMass = 2.0, thighMass = .5, shinMass = .5;
	
	double headSize[3]	= {0.2, 0.2, 0.2};
	double thighSize[3]	= {0.075, 0.05, 0.05};
	double shinSize[3]	= {0.075, 0.05, 0.05};
	
	double headHeight = 0.4;
	double anchorHeight = headHeight - (headSize[2] / 2.0);
	
	/* Anchor points clockwise in XY from 3pm */
	double legAnchors[4][3] = { 
								{ headSize[0]/2, -headSize[1]/2, -headSize[2]/2},
								{-headSize[0]/2, -headSize[1]/2, -headSize[2]/2},
								{-headSize[0]/2,  headSize[1]/2, -headSize[2]/2},
								{ headSize[0]/2,  headSize[1]/2, -headSize[2]/2} 
						   };
	
	/* Rotations clockwise from 3pm */
	double legXYRotations[4] = { M_PI/4.0, 3.0*M_PI/4.0, 5.0*M_PI/4.0, 7.0*M_PI/4.0 };
	
	double kneeAnchors[4][3] = {
							   { legAnchors[0][0]+thighSize[0]*cos(legXYRotations[0]), legAnchors[0][1]-thighSize[0]*sin(legXYRotations[0]), anchorHeight },
							   { legAnchors[1][0]+thighSize[0]*cos(legXYRotations[1]), legAnchors[1][1]-thighSize[0]*sin(legXYRotations[1]), anchorHeight},
							   { legAnchors[2][0]+thighSize[0]*cos(legXYRotations[2]), legAnchors[2][1]-thighSize[0]*sin(legXYRotations[2]), anchorHeight},
							   { legAnchors[3][0]+thighSize[0]*cos(legXYRotations[3]), legAnchors[3][1]-thighSize[0]*sin(legXYRotations[3]), anchorHeight} };

	double thighCOMs[4][3] = {   
							{ legAnchors[0][0]+thighSize[0]/2.0*cos(legXYRotations[0]), legAnchors[0][1]-thighSize[0]/2.0*sin(legXYRotations[0]), anchorHeight},
							{ legAnchors[1][0]+thighSize[0]/2.0*cos(legXYRotations[1]), legAnchors[1][1]-thighSize[0]/2.0*sin(legXYRotations[1]), anchorHeight},
							{ legAnchors[2][0]+thighSize[0]/2.0*cos(legXYRotations[2]), legAnchors[2][1]-thighSize[0]/2.0*sin(legXYRotations[2]), anchorHeight},
							{ legAnchors[3][0]+thighSize[0]/2.0*cos(legXYRotations[3]), legAnchors[3][1]-thighSize[0]/2.0*sin(legXYRotations[3]), anchorHeight}};
	
	double shinCOMs[4][3] = {
							{ legAnchors[0][0]+(thighSize[0]+shinSize[0]/2)*cos(legXYRotations[0]), legAnchors[0][1]-(thighSize[0]+shinSize[0]/2)*sin(legXYRotations[0]), anchorHeight},
							{ legAnchors[1][0]+(thighSize[0]+shinSize[0]/2)*cos(legXYRotations[1]), legAnchors[1][1]-(thighSize[0]+shinSize[0]/2)*sin(legXYRotations[1]), anchorHeight},
							{ legAnchors[2][0]+(thighSize[0]+shinSize[0]/2)*cos(legXYRotations[2]), legAnchors[2][1]-(thighSize[0]+shinSize[0]/2)*sin(legXYRotations[2]), anchorHeight},
							{ legAnchors[3][0]+(thighSize[0]+shinSize[0]/2)*cos(legXYRotations[3]), legAnchors[3][1]-(thighSize[0]+shinSize[0]/2)*sin(legXYRotations[3]), anchorHeight}};
							
	universe = physics_handle;
	u_space = space_handle;
	numBodies = 0;
	
	head   = dBodyCreate( universe );
	dMassSetBoxTotal( &mass, headMass, headSize[0], headSize[1], headSize[2] );
	head_g = dCreateBox( u_space, headSize[0], headSize[1], headSize[2] );
	dBodySetMass( head, &mass );
	dGeomSetBody( head_g, head );
	dBodySetPosition( head, 0, 0, .4 );
	geoms[numBodies] = head_g;
	bodies[numBodies++] = head;

	ear1 = dBodyCreate( universe );
	dMassSetBoxTotal( &mass, 0.01, 0.05, 0.05, 0.05 );
	ear1_g = dCreateBox( u_space, 0.05, 0.05, 0.05 );
	dBodySetMass( ear1, &mass );
	dGeomSetBody( ear1_g, ear1 );
	dBodySetPosition( ear1, 0, -headSize[1]/2.0, .4 );
	geoms[numBodies] = ear1_g;
	bodies[numBodies++] = ear1;

	ear2 = dBodyCreate( universe );
	dMassSetBoxTotal( &mass, 0.01, 0.05, 0.05, 0.05 );
	ear2_g = dCreateBox( u_space, 0.05, 0.05, 0.05 );
	dBodySetMass( ear2, &mass );
	dGeomSetBody( ear2_g, ear2 );
	dBodySetPosition( ear2, 0, headSize[1]/2.0, .4 );
	geoms[numBodies] = ear2_g;
	bodies[numBodies++] = ear2;

	thigh0 = dBodyCreate( universe );
	dMassSetBoxTotal( &mass, thighMass, thighSize[0], thighSize[1], thighSize[2] );
	thigh0_g = dCreateBox( u_space, thighSize[0], thighSize[1], thighSize[2] );
	dBodySetMass( thigh0, &mass );
	dGeomSetBody( thigh0_g, thigh0 );
	dBodySetPosition( thigh0, thighCOMs[0][0], thighCOMs[0][1], thighCOMs[0][2] );
	dRFromAxisAndAngle( rotation, 0, 0, -1, legXYRotations[0] );
	dBodySetRotation( thigh0, rotation );
	geoms[numBodies] = thigh0_g;
	bodies[numBodies++] = thigh0;

	thigh1 = dBodyCreate( universe );
	dMassSetBoxTotal( &mass, thighMass, thighSize[0], thighSize[1], thighSize[2] );
	thigh1_g = dCreateBox( u_space, thighSize[0], thighSize[1], thighSize[2] );
	dBodySetMass( thigh1, &mass );
	dGeomSetBody( thigh1_g, thigh1 );
	dBodySetPosition( thigh1, thighCOMs[1][0], thighCOMs[1][1], thighCOMs[1][2] );
	dRFromAxisAndAngle( rotation, 0, 0, -1, legXYRotations[1] );
	dBodySetRotation( thigh1, rotation );
	geoms[numBodies] = thigh1_g;
	bodies[numBodies++] = thigh1;
		
	thigh2 = dBodyCreate( universe );
	dMassSetBoxTotal( &mass, thighMass, thighSize[0], thighSize[1], thighSize[2] );
	thigh2_g = dCreateBox( u_space, thighSize[0], thighSize[1], thighSize[2] );
	dBodySetMass( thigh2, &mass );
	dGeomSetBody( thigh2_g, thigh2 );
	dBodySetPosition( thigh2, thighCOMs[2][0], thighCOMs[2][1], thighCOMs[2][2] );
	dRFromAxisAndAngle( rotation, 0, 0, -1, legXYRotations[2] );
	dBodySetRotation( thigh2, rotation );
	geoms[numBodies] = thigh2_g;
	bodies[numBodies++] = thigh2;
		
	thigh3 = dBodyCreate( universe );
	dMassSetBoxTotal( &mass, thighMass, thighSize[0], thighSize[1], thighSize[2] );
	thigh3_g = dCreateBox( u_space, thighSize[0], thighSize[1], thighSize[2] );
	dBodySetMass( thigh3, &mass );
	dGeomSetBody( thigh3_g, thigh3 );
	dBodySetPosition( thigh3, thighCOMs[3][0], thighCOMs[3][1], thighCOMs[3][2] );
	dRFromAxisAndAngle( rotation, 0, 0, -1, legXYRotations[3] );
	dBodySetRotation( thigh3, rotation );
	geoms[numBodies] = thigh3_g;
	bodies[numBodies++] = thigh3;

	shin0 = dBodyCreate( universe );
	dMassSetBoxTotal( &mass, shinMass, shinSize[0], shinSize[1], shinSize[2] );
	shin0_g = dCreateBox( u_space, shinSize[0], shinSize[1], shinSize[2] );
	dBodySetMass( shin0, &mass );
	dGeomSetBody( shin0_g, shin0 );
	dBodySetPosition( shin0, shinCOMs[0][0], shinCOMs[0][1], shinCOMs[0][2] );
	dRFromAxisAndAngle( rotation, 0, 0, -1, legXYRotations[0] );
	dBodySetRotation( shin0, rotation );
	geoms[numBodies] = shin0_g;
	bodies[numBodies++] = shin0;
			
	shin1 = dBodyCreate( universe );
	dMassSetBoxTotal( &mass, shinMass, shinSize[0], shinSize[1], shinSize[2] );
	shin1_g = dCreateBox( u_space, shinSize[0], shinSize[1], shinSize[2] );
	dBodySetMass( shin1, &mass );
	dGeomSetBody( shin1_g, shin1 );
	dBodySetPosition( shin1, shinCOMs[1][0], shinCOMs[1][1], shinCOMs[1][2] );
	dRFromAxisAndAngle( rotation, 0, 0, -1, legXYRotations[1] );
	dBodySetRotation( shin1, rotation );
	geoms[numBodies] = shin1_g;
	bodies[numBodies++] = shin1;

	shin2 = dBodyCreate( universe );
	dMassSetBoxTotal( &mass, shinMass, shinSize[0], shinSize[1], shinSize[2] );
	shin2_g = dCreateBox( u_space, shinSize[0], shinSize[1], shinSize[2] );
	dBodySetMass( shin2, &mass );
	dGeomSetBody( shin2_g, shin2 );
	dBodySetPosition( shin2, shinCOMs[2][0], shinCOMs[2][1], shinCOMs[2][2] );
	dRFromAxisAndAngle( rotation, 0, 0, -1, legXYRotations[2] );
	dBodySetRotation( shin2, rotation );
	geoms[numBodies] = shin2_g;
	bodies[numBodies++] = shin2;	
	
	shin3 = dBodyCreate( universe );
	dMassSetBoxTotal( &mass, shinMass, shinSize[0], shinSize[1], shinSize[2] );
	shin3_g = dCreateBox( u_space, shinSize[0], shinSize[1], shinSize[2] );
	dBodySetMass( shin3, &mass );
	dGeomSetBody( shin3_g, shin3 );
	dBodySetPosition( shin3, shinCOMs[3][0], shinCOMs[3][1], shinCOMs[3][2] );
	dRFromAxisAndAngle( rotation, 0, 0, -1, legXYRotations[3] );
	dBodySetRotation( shin3, rotation );
	geoms[numBodies] = shin3_g;
	bodies[numBodies++] = shin3;	
	
	
	/* Build the joints between bodies */
	numJoints = 0;
	
	/* Knees are hinges, hips are universal */
	hip0 = dJointCreateUniversal(universe,0);
	dJointAttach( hip0, head, thigh0 );
	dJointSetUniversalAnchor( hip0, legAnchors[0][0], legAnchors[0][1], legAnchors[0][2]+.4 );
	dJointSetUniversalAxis1( hip0, 0, 0, 1 ); // Joint rotates on X-Y plane
	dJointSetUniversalAxis2( hip0, -cos(legXYRotations[0]), -sin(legXYRotations[0]),0 ); // Up and down
	dJointSetUniversalParam( hip0, dParamLoStop, -M_PI/4.0 );
	dJointSetUniversalParam( hip0, dParamHiStop, M_PI/4.0 );
	dJointSetUniversalParam( hip0, dParamLoStop2, 0);
	dJointSetUniversalParam( hip0, dParamHiStop2, M_PI/2.0 );
	joints[numJoints++] = hip0;
	
	hip1 = dJointCreateUniversal(universe,0);
	dJointAttach( hip1, head, thigh1 );
	dJointSetUniversalAnchor( hip1, legAnchors[1][0], legAnchors[1][1], legAnchors[1][2]+.4 );
	dJointSetUniversalAxis1( hip1, 0, 0, 1 ); // Joint rotates on X-Y plane
	dJointSetUniversalAxis2( hip1, cos(legXYRotations[1]), sin(legXYRotations[1]),0 ); // Up and down	
	dJointSetUniversalParam( hip1, dParamLoStop, -M_PI/4.0 );
	dJointSetUniversalParam( hip1, dParamHiStop, M_PI/4.0 );
	dJointSetUniversalParam( hip1, dParamLoStop2, 0 );
	dJointSetUniversalParam( hip1, dParamHiStop2, M_PI/2.0 );
	joints[numJoints++] = hip1;

	hip2 = dJointCreateUniversal(universe,0);
	dJointAttach( hip2, head, thigh2 );
	dJointSetUniversalAnchor( hip2, legAnchors[2][0], legAnchors[2][1], legAnchors[2][2]+.4 );
	dJointSetUniversalAxis1( hip2, 0, 0, 1 ); // Joint rotates on X-Y plane
	dJointSetUniversalAxis2( hip2, -cos(legXYRotations[2]), -sin(legXYRotations[2]),0 ); // Up and down		
	dJointSetUniversalParam( hip2, dParamLoStop, -M_PI/2.0 );
	dJointSetUniversalParam( hip2, dParamHiStop, M_PI/2.0 );
	dJointSetUniversalParam( hip2, dParamLoStop2, -M_PI/2.0 );
	dJointSetUniversalParam( hip2, dParamHiStop2, M_PI/2.0 );
	joints[numJoints++] = hip2;

	hip3 = dJointCreateUniversal(universe,0);
	dJointAttach( hip3, head, thigh3 );
	dJointSetUniversalAnchor( hip3, legAnchors[3][0], legAnchors[3][1], legAnchors[3][2]+.4 );
	dJointSetUniversalAxis1( hip3, 0, 0, 1 ); // Joint rotates on X-Y plane
	dJointSetUniversalAxis2( hip3, cos(legXYRotations[3]), sin(legXYRotations[3]),0 ); // Up and down		
	dJointSetUniversalParam( hip3, dParamLoStop, -M_PI/2.0 );
	dJointSetUniversalParam( hip3, dParamHiStop, M_PI/2.0 );
	dJointSetUniversalParam( hip3, dParamLoStop2, -M_PI/2.0 );
	dJointSetUniversalParam( hip3, dParamHiStop2, M_PI/2.0 );	
	joints[numJoints++] = hip3;

	knee0 = dJointCreateHinge(universe,0);
	dJointAttach( knee0, thigh0, shin0 );
	dJointSetHingeAnchor( knee0, kneeAnchors[0][0], kneeAnchors[0][1], kneeAnchors[0][2] );
	dJointSetHingeAxis( knee0, -cos(legXYRotations[0]), -sin(legXYRotations[0]),0 );
	dJointSetHingeParam( knee0, dParamLoStop, 0);
	dJointSetHingeParam( knee0, dParamHiStop, M_PI/2.0);
	joints[numJoints++] = knee0;

	knee1 = dJointCreateHinge(universe,0);
	dJointAttach( knee1, thigh1, shin1 );
	dJointSetHingeAnchor( knee1, kneeAnchors[1][0], kneeAnchors[1][1], kneeAnchors[1][2] );
	dJointSetHingeAxis( knee1, cos(legXYRotations[1]), sin(legXYRotations[1]),0 );	
	dJointSetHingeParam( knee1, dParamLoStop, 0);
	dJointSetHingeParam( knee1, dParamHiStop, M_PI/2.0);
	joints[numJoints++] = knee1;

	knee2 = dJointCreateHinge(universe,0);
	dJointAttach( knee2, thigh2, shin2 );
	dJointSetHingeAnchor( knee2, kneeAnchors[2][0], kneeAnchors[2][1], kneeAnchors[2][2] );
	dJointSetHingeAxis( knee2, -cos(legXYRotations[2]), -sin(legXYRotations[2]),0 );	
	dJointSetHingeParam( knee2, dParamLoStop, 0);
	dJointSetHingeParam( knee2, dParamHiStop, M_PI/2.0);
	joints[numJoints++] = knee2;

	knee3 = dJointCreateHinge(universe,0);
	dJointAttach( knee3, thigh3, shin3 );
	dJointSetHingeAnchor( knee3, kneeAnchors[3][0], kneeAnchors[3][1], kneeAnchors[3][2] );
	dJointSetHingeAxis( knee3, cos(legXYRotations[3]), sin(legXYRotations[3]),0 );		
	dJointSetHingeParam( knee3, dParamLoStop, 0);
	dJointSetHingeParam( knee3, dParamHiStop, M_PI/2.0);	
	joints[numJoints++] = knee3;			
	
	// Ears
	earjoint1 = dJointCreateFixed( universe,0);
	dJointAttach( earjoint1, ear1, head );
	earjoint2 = dJointCreateFixed( universe,0);
	dJointAttach(earjoint2, ear2, head);
	dJointSetFixed(earjoint1);
	dJointSetFixed(earjoint2);
	joints[numJoints++] = earjoint1;
	joints[numJoints++] = earjoint2;


}

Body::~Body()
{
	/* Destroy all geoms and bodies */
	for( int joint=0; joint<numJoints; joint++ )
		dJointDestroy( joints[joint] );
		
	for( int body=0; body<numBodies; body++ )
	{
		dGeomDestroy( geoms[body] );
		dBodyDestroy( bodies[body] );
	}
}

void Body::collectJointAngles()
{
	double joint_old_angles[8][2];
	for( int joint=0; joint<8; joint++ )
	{
		joint_old_angles[joint][0] = joint_current_angles[joint][0];
		joint_old_angles[joint][1] = joint_current_angles[joint][1];
	}
	
	for( int joint=0; joint<4; joint++ )
	{
		joint_current_angles[joint][0] = dJointGetUniversalAngle1(joints[joint]);
		joint_current_angles[joint][1] = dJointGetUniversalAngle2(joints[joint]);				
	}
	for( int joint=4; joint<8; joint++ )
	{
		joint_current_angles[joint][0] = dJointGetHingeAngle(joints[joint]);
	}
	for( int joint=0; joint<8; joint++ )
	{
		joint_deltas[joint][0] = joint_current_angles[joint][0] - joint_old_angles[joint][0];
		joint_deltas[joint][1] = joint_current_angles[joint][1] - joint_old_angles[joint][1];
	}
}

void Body::applyTorques()
{	
	
	for( int j=0; j<4; j++ )
		dJointAddUniversalTorques(joints[j], joint_torques[j][0], joint_torques[j][1]);

	for( int j=4; j<8; j++ )
		dJointAddHingeTorque(joints[j], joint_torques[j][0]);
		
	for( int t=0; t<8; t++ )
		joint_torques[t][0] = joint_torques[t][1] = 0.0;
		
	return;
}

void Body::applyPDControlledTorques()
{
	//double ks = 1.2, kd = 0.2; // good for leaky integrator
	double ks = 0.5, kd = 0.5; // good for feedforward
	for( int joint=0; joint<8; joint++ )
	{
		joint_torques[joint][0] = ks * ( joint_target_angles[joint][0] - joint_current_angles[joint][0]) - kd*joint_deltas[joint][0];
		joint_torques[joint][1] = ks * ( joint_target_angles[joint][1] - joint_current_angles[joint][1]) - kd*joint_deltas[joint][1];		

//#define MAX_TORQUE 0.5
#define MAX_TORQUE 0.125
//#warning MAX_TORQUE is 0.125 - this is the "hard" version
        #warning this version incompatible with any saved population pre 2020-04-18
        if(joint_torques[joint][0] < -MAX_TORQUE ) { joint_torques[joint][0] = -MAX_TORQUE; }
        if(joint_torques[joint][1] < -MAX_TORQUE ) { joint_torques[joint][1] = -MAX_TORQUE; }

		if( joint_torques[joint][0] > MAX_TORQUE )
		{
			//printf("Error: controller demanded huge torque of %f!!! Reset to %f.\n", joint_torques[joint][0], MAX_TORQUE );
			joint_torques[joint][0] = MAX_TORQUE;
		}
		if( joint_torques[joint][1] > MAX_TORQUE )
		{
			//printf("Error: controller demanded huge torque of %f!!! Reset to %f.\n", joint_torques[joint][1], MAX_TORQUE );
			joint_torques[joint][1] = MAX_TORQUE;
		}
	}
	
	applyTorques();
	
}

double Body::getHeadBalance()
{
	double angle;
	
	const dReal *rot = dBodyGetRotation(head);	

	angle = acos( rot[10] );
	return angle;
}

Agent::Agent( dWorldID physics_handle, dSpaceID space_handle )
{
	universe = physics_handle; u_space = space_handle;
	
	/* Select appropriate generation-zero genome for controller in use: FeedForward, LeakyIntegrator */
	controller_definition = GeneticString( FeedForward );
	init();
}

Agent::Agent( Agent *parent1, Agent *parent2, dWorldID physics_handle, dSpaceID space_handle  )
{
	universe = physics_handle; u_space = space_handle;
	
	controller_definition = GeneticString( &(parent1->controller_definition), &(parent2->controller_definition) );
	init();
}

Agent::Agent( Agent *parent1, dWorldID physics_handle, dSpaceID space_handle  )
{
	universe = physics_handle; u_space = space_handle;
	
	controller_definition = GeneticString( &(parent1->controller_definition) );
	init();

}

Agent::~Agent()
{
	readyToSimulate = false;
	
	if( controller ) { delete controller; controller = 0; } 
	
	if(!deactivated)
		if( body ) delete body;
}

void Agent::tick(double timestep)
{
	if( deactivated ) return;
	
	timesteps++;
	age += timestep;
	
	body->collectJointAngles();
	
	const double* headPos = dBodyGetPosition( body->head );
	position[0] = headPos[0]; position[1] = headPos[1];
	
	double balance = body->getHeadBalance();

	const double* ep = dBodyGetPosition( body->ear1 );
	double point_to_ear1 = sqrt(((target[0]-ep[0])*(target[0]-ep[0]))+((target[1]-ep[1])*(target[1]-ep[1])));
	ep = dBodyGetPosition(body->ear2);
	double point_to_ear2 = sqrt(((target[0]-ep[0])*(target[0]-ep[0]))+((target[1]-ep[1])*(target[1]-ep[1])));

	double ear_separation = 0.2; /* Check that this value is ok edit think it is */
	double point_input = (point_to_ear1 - point_to_ear2) / ear_separation;

	/* Collect new inputs for controller step */
	/* Unset inputs are zero and have no effect */
	
	/* Standard inputs [0-2] */
	controller->inputs[0] = sin( age*2*M_PI ); // input 0 - sin
	controller->inputs[1] = cos( age*2*M_PI ); // input 1 - cos

	controller->inputs[2] = balance; // input 2 - balance sensor

	controller->inputs[3] = point_input;

	/* inputs [4,5,6] unused */

	/* 2-axis joint angle inputs [7-14] */
	for( int joint=0; joint<4; joint++ )
	{
		controller->inputs[7+joint*2] = body->joint_current_angles[joint][0];
		controller->inputs[8+joint*2] = body->joint_current_angles[joint][1];		
	}
	
	/* 1-axis joint angle inputs [15-18] */
	for( int joint=4; joint<8; joint++ )
	{
		controller->inputs[11+joint] = body->joint_current_angles[joint][0];	
	}
	
	/* Step controller */
	controller->step();
	
	/* Collect outputs from controller for physical step */

	/* 2-axis joints (consume two neural outputs per joint) */
	for( int joint=0; joint<4; joint++ )
	{
		body->joint_target_angles[joint][0] = (controller->outputs[joint*2]   * M_PI / 2.0) - (M_PI / 4.0);
		body->joint_target_angles[joint][1] = (controller->outputs[joint*2+1] * M_PI / 2.0);
	}	
		
	/* 1-axis joints (consume one neural output per joint) */
	for( int joint=4; joint<8; joint++ )
	{	
		body->joint_target_angles[joint][0] = (controller->outputs[joint+4] * M_PI / 2.0);	
	}

	// Uncomment below to test individual joints (comment out two for loops above)
	//body->joint_target_angles[2][0] = (controller->outputs[6] * M_PI / 2.0) - (M_PI / 4.0);
	//body->joint_target_angles[3][1] = (controller->outputs[7] * M_PI / 2.0);
	//body->joint_target_angles[7][0] = (controller->outputs[7+4] * M_PI / 2.0);

	body->applyPDControlledTorques();
}

void Agent::reset(dWorldID w, dSpaceID s)
{	
	universe = w; u_space = s;
	if( controller ) { delete controller; controller = 0; }
	if( body )       { delete body; body = 0; }
	
	init();

	controller = new FFPController( controller_definition );
	
	body = new Body( universe, u_space );	
	deactivated = false;
}

void Agent::deactivate()
{
	if( deactivated ) return;
	
	if( body )		 { delete body; body = 0; }
	if( controller ) { delete controller; controller = 0; }
	deactivated = true;
}

void Agent::init()
{
	readyToSimulate = false;
	deactivated = false;
	timesteps = 0;
	age = 0.0;

	position[0] = position[1] = 0.0;

	controller = NULL; /* Controller gets built on agent reset only */
	body = NULL; /* Body gets built on agent reset only */
	deactivated = true;
	readyToSimulate = true;	
}

} // namespace lwp
