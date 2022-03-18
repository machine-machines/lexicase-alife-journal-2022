#pragma once

#include <vector>
#include <string>

#include "Agent.h"
#include "EvolutionScheme.hpp"

using namespace std;

namespace xeres {

    class Species {

        public:

            unsigned int population_size;

            Species(unsigned int psize);
            ~Species();

            static Species* loadPopulation( string filename );
            bool     savePopulation( string filename );

            vector<Agent*> population;

            Agent* getMember(unsigned int i);

            void newGeneration(EvolutionScheme* e);
            void evaluate(vector<Task*> tasks, string filename, int generation);

        private:

            void deletePopulation();
    };
}
