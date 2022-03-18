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
    
    vector<xeres::Task*> tasks;
    tasks.push_back(wt);
    tasks.push_back(tt);
    tasks.push_back(filt);
    
    int gen=0;
    auto start = steady_clock::now();
    while(gen<500) {
        xeres::LexicaseEvolutionScheme e(tasks,gen,"logfile.notimplementedyet.csv");
        s->newGeneration(&e);
        auto end = steady_clock::now();
        duration<double> elapsed_seconds = end-start;
        cout << "generation " <<gen << " done (" <<elapsed_seconds.count() << "s)"<< endl;
        start=end;
        gen++; 
    }

    s->savePopulation("test-2tasks5envs.txt");
    cout << "test done" << endl;
}

