#include "subdiv.h"

#include <hbr/mesh.h>

void Subdiv::initialize(const OsdUtilSubdivTopology &topology, const std::vector<real> &pointPositions)
{
    std::string *errorMessage;

    // Create adaptiveEvaluator
    if (!_adaptiveEvaluator.Initialize(topology, errorMessage)) {
        std::cout << "Initialize failed with " << *errorMessage << std::endl;
        return;
    }

    // Push the vertex data
    _adaptiveEvaluator.SetCoarsePositions(&(pointPositions[0]), (int) pointPositions.size(), errorMessage);

    // Refine with one thread
    if (!_adaptiveEvaluator.Refine(1, errorMessage)) {
        std::cout << "Refine failed with " << *errorMessage << std::endl;
        return;
    }

    if(_adaptiveEvaluator.GetHbrMesh()->GetNumDisconnectedVertices()>0)
    {
        printf("The specified subdivmesh contains disconnected surface components.\n");
        // should abort or iterate over the mesh to remove the offending vertices
    }
}

void Subdiv::getRefineTopology(OsdUtilSubdivTopology &refinedTopology, std::vector<real> &positions)
{
    std::string *errorMessage;

    if (!_adaptiveEvaluator.GetRefinedTopology(&refinedTopology, &positions, errorMessage)) {
        std::cout << "GetRefinedTopology failed with " << *errorMessage << std::endl;
        return;
    }

}

void Subdiv::Evaluate(OsdEvalCoords coord, vec3 *limitPos, vec3 *tanU, vec3 *tanV, mat2* I, mat2* II)
{
    real P[3], dPdu[3], dPdv[3], dPdudu[3], dPdvdv[3], dPdudv[3];
    assert(faceIndexMap.find(coord.face) != faceIndexMap.end());
    coord.face = faceIndexMap[coord.face];
    _adaptiveEvaluator.EvaluateLimit(coord, P, dPdu, dPdv, dPdudu, dPdvdv, dPdudv);

    limitPos->setX(P[0]);
    limitPos->setY(P[1]);
    limitPos->setZ(P[2]);

    assert(*limitPos  * * limitPos > 0);

    if(tanU){
        tanU->setX(dPdu[0]);
        tanU->setY(dPdu[1]);
        tanU->setZ(dPdu[2]);
    }
    if(tanV){
        tanV->setX(dPdv[0]);
        tanV->setY(dPdv[1]);
        tanV->setZ(dPdv[2]);
    }
    if(I && II){
        assert(tanU!=0 && tanV!=0);
        vec3 dudu = vec3(dPdudu[0], dPdudu[1], dPdudu[2]);
        vec3 dvdv = vec3(dPdvdv[0], dPdvdv[1], dPdvdv[2]);
        vec3 dudv = vec3(dPdudv[0], dPdudv[1], dPdudv[2]);

        real xform[2][2] = { {(*tanU)*(*tanU), (*tanU)*(*tanV)}, {(*tanU)*(*tanV), (*tanV)*(*tanV)} };
        I->Set(xform);
        vec3 normal = (*tanU) ^ (*tanV);
        normal.normalize();
        real xform2[2][2] = { {dudu*normal, dudv*normal}, {dudv*normal, dvdv*normal} };
        II->Set(xform2);
    }
}
