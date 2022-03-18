#include "Species.hpp"

#include <fstream>

#include "Agent.h"

using namespace std;

namespace xeres {

    Species::Species(unsigned int psize) {

        population_size = psize;

        //    population = new Agent[population_size];

        for(unsigned int c=0; c<population_size; c++) {
            // normally would have (world, space)
            population.push_back(new Agent(NULL, NULL));
        }
    }

    Species::~Species() {

        deletePopulation();
    }

    void Species::deletePopulation() {
        for(unsigned int c=0; c<population_size; c++) {
            if(population[c]) delete population[c];
            population[c] = nullptr;
        }

        population.clear();
    }

    Species* Species::loadPopulation( string filename ) {
        Species* s = new Species(0);
        s->deletePopulation();

        ifstream fin(filename);

        if(!fin) { cerr << "Error opening file: " << filename << endl; return nullptr; }

        int count = 0;

        while(fin && count < 2000) {
            char ch = fin.peek();
            if(ch==EOF) break;
            if(ch==32||ch==10) { fin.ignore(1); continue; }
            Agent* a = new Agent(NULL,NULL);
            fin >> a->controller_definition;
            //        cout<<a->controller_definition<<endl;
            s->population.push_back(a);
            count++;
        }
        cout << "Loaded " << count << " individuals from " << filename << "." << endl;
        s->population_size = s->population.size();
        return s;
    }

    bool Species::savePopulation( string filename ) {
        ofstream fout(filename);
        for(unsigned int c=0; c<population_size; c++ )
            fout << population[c]->controller_definition << std::endl;

        return true;
    }

    Agent* Species::getMember(unsigned int i) {
        return population.at(i);

    }

    void Species::newGeneration(EvolutionScheme* e) {

        vector<Agent*> newpop;

        for(unsigned int c=0; c<population_size; c++) {
            newpop.push_back(e->produceAgent(population));
        }

        /* Delete old population */
        for(unsigned int c=0; c<population_size; c++) {
            if(population[c]) delete population[c];
            population[c] = newpop[c];
        }
    }

    void Species::evaluate(vector<Task*> tasks, string filename, int generation) {
        ofstream fout(filename);

        for(auto t: tasks) {
            for(unsigned int e=0; e<t->NUM_ENVIRONMENTS; e++) {
                for(unsigned int i=0; i<population.size(); i++ ) {
                    TaskWorld *tw = t->launchTask(population[i], e);
                    double f = tw->getFitness();
                    fout << "\"" << t->name << "\", " << e << ", " << generation << ", " << i << ", " << f << endl;
                }
            }
        }
    }
} // namespace
