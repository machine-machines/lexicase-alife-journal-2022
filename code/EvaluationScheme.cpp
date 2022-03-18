#include "EvaluationScheme.hpp"

#include <vector>
#include <string>

#include "Task.hpp"

using namespace std;

namespace xeres {

    EvaluationScheme::EvaluationScheme(vector<Task*> t) : tasks(t) {

    }

    EvaluationScheme::~EvaluationScheme() {

    }

    void EvaluationScheme::runEvaluation(string filename) {

        for(auto t: tasks) {
            for(int e=0; e<t->NUM_ENVIRONMENTS; e++) {

    }

} // namespace
