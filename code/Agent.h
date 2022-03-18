#include "ode/ode.h"
#include <iostream>
#include <sstream>
#include <cstdio>

#ifndef AGENT_H 
#define AGENT_H

#define NETWORK_SIZE 12
#define NUM_INPUTS 19
#define MIN_OUTPUTS 12

namespace xeres {

enum ControllerType { FeedForward };

#define GENOTYPE_LENGTH NETWORK_SIZE*(NETWORK_SIZE+2+NUM_INPUTS)

class GeneticString
{
public:
	
	double mutation_rate;
	static const int		length = NETWORK_SIZE*(NETWORK_SIZE+2+NUM_INPUTS); // tc, bias + inputs
	double					data[length];

			GeneticString();
			GeneticString( ControllerType type ); /* 1=Random or 0=zero */
			GeneticString( GeneticString *parent1, GeneticString *parent2 ); /* One-point crossover */
			GeneticString( GeneticString *parent1 ); /* Parthenogenesis */
			~GeneticString();
	double  distance( GeneticString& comp );
			void init();

			 friend std::ostream& operator<<(std::ostream& os, const GeneticString& gs);
			 friend std::istream& operator>>(std::istream& is, GeneticString& gs);

			 friend bool operator==(const GeneticString& gs1, const GeneticString& gs2 );
			 friend bool operator!=(const GeneticString& gs1, const GeneticString& gs2 );
};
/* Abstract superclass for controllers */
class Controller
{
public:
	virtual ~Controller();
	virtual void step() = 0;
	double inputs[NUM_INPUTS];
	double outputs[NETWORK_SIZE];
};

/* Feedforward perceptron controller */
/* NUM_INPUTS inputs, NETWORK_SIZE hidden, NETWORK_SIZE outputs */
class FFPController : public Controller
{
public:
	
	double input_hidden_weight[NUM_INPUTS][NETWORK_SIZE];
	double hidden_output_weight[NETWORK_SIZE][NETWORK_SIZE];
	
	FFPController( GeneticString genetic_template );
	~FFPController();
	
	void step();
};

class Body
{
public:
	dWorldID	universe;
	dSpaceID	u_space;
	
				Body( dWorldID physics_handle, dSpaceID space_handle );
				~Body();
	
	dBodyID		head;
	dGeomID		head_g;
	dBodyID		shin0, shin1, shin2, shin3;	

	dBodyID		ear1, ear2;
	dGeomID		ear1_g, ear2_g;


	double		joint_current_angles[8][2];
	double		joint_target_angles[8][2]; /* For PD controller joints */
	double		joint_torques[8][2]; /* Torques to apply next timestep for non-PD */
	double		joint_deltas[8][2];  /* For damping on PD controlled joints */
	
	void		collectJointAngles();
	void		applyTorques();
	void		applyPDControlledTorques();
	
	double		getHeadBalance();

	int			numBodies, numJoints;
	dBodyID		bodies[11];
	dGeomID		geoms[11];
	dJointID	joints[10];
};

class Agent
{
public:
	Controller	*controller;
	Body		*body;

	bool		readyToSimulate;

				Agent( dWorldID physics_handle, dSpaceID space_handle ); /* Random or zero  genotypes */
				Agent( Agent *parent1, Agent *parent2, dWorldID physics_handle, dSpaceID space_handle  ); /* One-point crossover */
				Agent( Agent *parent1, dWorldID physics_handle, dSpaceID space_handle  ); /* Parthenogenesis */

				~Agent();
	
	void		tick(double timestep); /* Update controller by timestep */
	void		reset( dWorldID w, dSpaceID s);
	void		deactivate(); /* Delete the body */
	
	double		position[2];
	double		target[2];
	bool		deactivated;
	
	dWorldID	universe;
	dSpaceID	u_space;
	
	GeneticString	controller_definition;
	GeneticString	body_definition;

	int			timesteps;
	double		age;
	
private:
	void		init();
};

} // namespace
#endif

