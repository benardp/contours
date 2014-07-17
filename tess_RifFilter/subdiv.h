#ifndef __SUBDIV_H__
#define __SUBDIV_H__

#include "VecMat.h"
#include <osdutil/topology.h>
#include <osdutil/adaptiveEvaluator.h>

using namespace OpenSubdiv::OPENSUBDIV_VERSION;

//------------------------------------------------------------------------------

class Subdiv {
public:
    static Subdiv& getInstance()
    {
        static Subdiv instance;
        return instance;
    }

    void initialize(const OsdUtilSubdivTopology &topology, const std::vector<real> &pointPositions);

    void getRefineTopology(OsdUtilSubdivTopology &refinedTopologym, std::vector<real> &positions);

    void Evaluate(OsdEvalCoords coord, vec3 *limitPos, vec3 *tanU=NULL, vec3 *tanV=NULL, mat2* I=NULL, mat2* II=NULL);

    std::map<int,int> faceIndexMap;
private:
    Subdiv() {}

    OsdUtilAdaptiveEvaluator _adaptiveEvaluator;
};

#endif
