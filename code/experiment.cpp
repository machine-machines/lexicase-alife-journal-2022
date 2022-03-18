#include <vector>
#include <chrono>

#include "xeres.hpp"

using namespace std;
using namespace std::chrono;

int main( int argc, char **argv ) {

    xeres::Species* s = new xeres::Species(50);

    xeres::WallTask* wt = new xeres::WallTask;
    xeres::TurnTask* tt = new xeres::TurnTask;
    xeres::FloorIsLavaTask* filt = new xeres::FloorIsLavaTask;
    xeres::PreciousCargoTask* pct = new xeres::PreciousCargoTask;

    vector<xeres::Task*> tasks;
    tasks.push_back(wt);
    tasks.push_back(tt);
    // tasks.push_back(filt);
    tasks.push_back(pct);
    
    int gen=0;
    auto start = steady_clock::now();

    while(gen<5000) {
        xeres::LexicaseEvolutionScheme e(tasks, gen, "logfile.notimplementedyet.csv");

        auto gen_start = steady_clock::now();
        s->newGeneration(&e);
        auto gen_end = steady_clock::now();
        
        duration<double> elapsed_seconds = gen_end-gen_start;
        cout << "evolution at generation " <<gen << " done (" <<elapsed_seconds.count() << "s)"<< endl;
        gen_start=gen_end;
        gen++;

        if(gen%500==0) { // save population and evaluate
            string pop_fname = "gen"+to_string(gen)+".txt";
            string eva_fname = "gen"+to_string(gen)+".csv";

            s->savePopulation(pop_fname);

            auto ev_start = steady_clock::now();
            s->evaluate(tasks, eva_fname, gen);
            auto ev_end = steady_clock::now();

            duration<double> elapsed_seconds = ev_end-ev_start;
            cout<<"evaluation at generation " << gen << " done (" <<elapsed_seconds.count() << "s)"<< endl;
        }
    }

    auto end = steady_clock::now();
    duration<double> elapsed_seconds = end-start;
    cout << "experiment done (" <<elapsed_seconds.count() << "s)"<< endl;
}

