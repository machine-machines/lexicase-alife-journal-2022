#pragma once

#include <vector>
#include <string>

#include "Task.hpp"

using namespace std;

namespace xeres {

    class EvaluationScheme {

        public:

            EvaluationScheme(vector<Task*> t);
            ~EvaluationScheme();
            
            void runEvaluation(string filename);

        private:
            vector<Task*> tasks;

    };

} // namespace
