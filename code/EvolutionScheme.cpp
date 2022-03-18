#include "EvolutionScheme.hpp"

#include <vector>
#include <iterator>
#include <algorithm>
#include <limits>

#include "Agent.h"
#include "Util.hpp"
#include "Task.hpp"

// LOGGING
// Gen, SelEvent, Depth, NumInds, InIDList

namespace xeres {

    using namespace std;

    EvolutionScheme::EvolutionScheme() {

    }

    EvolutionScheme::~EvolutionScheme() {

    }

    LexicaseEvolutionScheme::LexicaseEvolutionScheme(vector<Task*> tasks, int generation, string log_file) {

        int case_id = 0;
		int num_envs = 10;

        this->generation = generation;
        this->log_file = log_file;

//#define NAIVE
//#define EVEN
#define FLIP

#ifdef NAIVE
		// Naive scheme:
        // create a test_case for every task/environment combination
        for( auto t: tasks ) {
            for( unsigned int e=0; e<t->NUM_ENVIRONMENTS; e++ ) {
                test_cases.emplace_back( t,e,case_id );
				// should this be here?
            	case_id++;
            }
        }

        num_cases = test_cases.size();
        max_cases = case_id;

        // Shuffle task,parameter combinations
        shuffle(test_cases.begin(), test_cases.end(), g);

        // resize to the desired number of tests per selection event
        test_cases.erase(test_cases.begin()+num_envs, test_cases.end());

        // resize() requires a default constructor, so not using
        // test_cases.resize(num_envs); 
#endif
#ifdef EVEN
		// Even sampling scheme:
		//
		int num_tasks = tasks.size();
	
		int envs_per_task = num_envs / num_tasks;

		// For every task, get envs_per_task random environments and
		// add to the final test_cases vector:
		for( auto t: tasks ) {
			vector<test_case> tmp;
			for(unsigned int e=0; e<t->NUM_ENVIRONMENTS; e++ ) {
        		tmp.emplace_back( t,e,case_id );
				case_id++;
       		}
			shuffle(tmp.begin(), tmp.end(), g);
			copy(tmp.begin(), tmp.begin()+envs_per_task, back_inserter(test_cases));
    	}
        
        shuffle(test_cases.begin(), test_cases.end(), g);

		num_cases = test_cases.size();
        max_cases = case_id;
#endif
#ifdef FLIP
        int flip_every = 1;

		int tasknum = (generation / flip_every) % tasks.size();
        case_id = 100 * tasknum;

		Task* t = tasks.at(tasknum);
		for( unsigned int e=0; e<t->NUM_ENVIRONMENTS; e++ ) {
       		test_cases.emplace_back( t,e,case_id );
			case_id++;
		}
        // Shuffle task,parameter combinations
        shuffle(test_cases.begin(), test_cases.end(), g);

        // resize to the desired number of tests per selection event
        test_cases.erase(test_cases.begin()+num_envs, test_cases.end());

		num_cases = test_cases.size();
        max_cases = case_id;
#endif
	}

    LexicaseEvolutionScheme::~LexicaseEvolutionScheme() {

    }

    Agent* LexicaseEvolutionScheme::produceAgent( vector<Agent*>& population ) {

        vector<int> parent_indicies;

        for(unsigned int i=0; i<population.size(); ++i)
            parent_indicies.push_back(i);


        shuffle(parent_indicies.begin(),parent_indicies.end(),g);
        vector<int> par_subset(parent_indicies.begin(),parent_indicies.begin()+5); //pool of 5
        Agent* p1 = selectParent( population, par_subset );

        shuffle(parent_indicies.begin(),parent_indicies.end(),g);
        vector<int> par_subset2(parent_indicies.begin(),parent_indicies.begin()+5); //pool of 5
        Agent* p2 = selectParent( population, par_subset2 );

        return new Agent(p1, p2, NULL, NULL);
    }

    Agent* LexicaseEvolutionScheme::selectParent( vector<Agent*>& population, vector<int>& individuals ) {

        double max_fit = -numeric_limits<double>::max();
        vector<double> fitness(population.size());

        int depth = 0;

        // Iterate over every environment, one by one.
        for( auto e: test_cases ) {

            // gen,depth,env_id,num_inds,ind_ids
            std::cout << generation << "," << depth << "," << e.task->name << "," << e.environment << "," << e.id << "," << individuals.size() << "," << vectorToPythonListString(individuals) << std::endl;
            
            // No need to keep evaluating if there is only one individual left.
            if(individuals.size() == 1) break;

            max_fit = -numeric_limits<double>::max();
            // Check each individual in the pool in this environment.
            // The cache location is a combination of individual and task and environment
            for( auto i: individuals ) {
                int cache_location = i*max_cases + e.id;
                if(!cache_hit[cache_location]) {
                    TaskWorld* tw = e.task->launchTask(population[i], e.environment);
                    cache[cache_location] = tw->getFitness();
                    cache_hit[cache_location] = true;
                    delete tw;
                }
                // Store the individuals fitness in this environment.
                fitness[i] = cache[cache_location];
                //cout << e.task->name << "/" << e.environment << ": " << fitness[i] << endl;
                max_fit = max(max_fit, fitness[i]);
            }
            // Remove individuals from the pool if they have poor fitness.
            for(vector<int>::iterator ind_itr = individuals.begin(); ind_itr < individuals.end();) {
                if(abs(max_fit-fitness[*ind_itr]) > abs(max_fit*0.1) ) { // fuzz facotr / epsilon
                    ind_itr = individuals.erase(ind_itr);
                } else {
                    ++ind_itr;
                }
            }
            depth++;
        } // next environment

        if(individuals.size()>1) {
            // Return a random choice from the remaining (tied) individuals
            shuffle(individuals.begin(), individuals.end(), g);

            individuals.erase(individuals.begin()+1, individuals.end());
            
            std::cout << generation << "," << depth << "," << "*" << "," << individuals.size() << "," << vectorToPythonListString(individuals) << std::endl;
        }

        return population[individuals[0]];
    }

} // namespace
