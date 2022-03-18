#pragma once

#include <vector>
#include <map>

#include "Agent.h"
#include "Task.hpp"

using namespace std;
namespace xeres {

    class EvolutionScheme {

        public:

            EvolutionScheme();
            virtual ~EvolutionScheme();

            virtual Agent* produceAgent( vector<Agent*>& population ) = 0;

            int generation;
            string log_file;

    };

    class LexicaseEvolutionScheme : public EvolutionScheme {

        public:
            LexicaseEvolutionScheme(vector<Task*> tasks, int generation, string log_file);
            ~LexicaseEvolutionScheme();

            Agent* produceAgent( vector<Agent*>& population ) override;

        private:
        
            struct test_case {
                Task* task;
                int environment;
                int id;
                test_case(Task* t, int e, int i) : task(t), environment(e), id(i) {}
            };

            vector<test_case> test_cases;

            Agent* selectParent( vector<Agent*>& population, vector<int>& individuals );

            int num_cases, max_cases;

            map<int,bool> cache_hit;
            map<int,double> cache;

    };

} // namespace
