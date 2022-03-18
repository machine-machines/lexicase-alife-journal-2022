#include <vector>
#include <chrono>

#include "xeres.hpp"

/*
 * Evaluation could look like this
 *
 * "taskname", "environment", "generation", "individual", "fitness"
 *
 */

using namespace std;
using namespace std::chrono;

int main( int argc, char **argv ) {

    xeres::Species* s = xeres::Species::loadPopulation("test.txt");

    xeres::WallTask* wt = new xeres::WallTask;
    xeres::TurnTask* tt = new xeres::TurnTask;
    xeres::FloorIsLavaTask* filt = new xeres::FloorIsLavaTask;


    vector<xeres::Task*> tasks;
    tasks.push_back(wt);
    tasks.push_back(tt);
    tasks.push_back(filt);

    int gen=0;
    auto start = steady_clock::now();
    s->evaluate(tasks, "test-eval.csv", gen);
    auto end = steady_clock::now();
    duration<double> elapsed_seconds = end-start;
    cout << "evaluation done (" <<elapsed_seconds.count() << "s)"<< endl;

    cout << "test done" << endl;
}

