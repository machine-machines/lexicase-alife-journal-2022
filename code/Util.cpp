#include <iostream>
#include <string>

#include "Util.hpp"

#include "Agent.h"

namespace xeres {

    std::random_device rd;
    unsigned int s = rd();
    std::mt19937 g(s);
    std::uniform_real_distribution<> dis(0, 1);

    std::string vectorToPythonListString(std::vector<int> theVector) {
        std::string ret;

        ret = "[";
        for(const auto &value : theVector) {
            ret += std::to_string(value);
            ret += ",";
        }
        ret.pop_back();
        ret += "]";
        return ret;
    }

    double randomdouble() {
        return dis(g);
    }
    int random(int maximum) {
        return (int) (randomdouble() * maximum);
    }
    double randomGaussianDouble() { 
        /* Return a random double from N(0,1); caller can then multiply by standard deviation and add mean if required */
        /* Marsaglia polar method (see http://en.wikipedia.org/wiki/Normal_distribution#Generating_values_from_normal_distribution) */
        double s,x,y;
        do {
            x=randomdouble()*2.0-1.0;
            y=randomdouble()*2.0-1.0;
            s=x*x+y*y;
        } while (s>=1.0 || s==0.0);
        return x * sqrt(-2.0*std::log(s)/s);
    }

    simbox geomToBox(dGeomID t, const char* id) {

        simbox s; dVector3 ss; dReal rot[4];
        dGeomGetQuaternion(t, rot);
        const dReal *pos = dGeomGetPosition(t);
        dGeomBoxGetLengths(t, ss);
        s.pos[0]=pos[0]; s.pos[1]=pos[1]; s.pos[2] = pos[2];
        s.rot[0]=rot[0]; s.rot[1]=rot[1]; s.rot[2] = rot[2]; s.rot[3] = rot[3];
        s.siz[0]=ss[0]; s.siz[1]=ss[1]; s.siz[2]=ss[2];
        s.id = std::string(id);
        return s;
    }

} // namespace lwp
