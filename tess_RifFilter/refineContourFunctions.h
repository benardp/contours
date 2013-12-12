#ifndef __REFINE_CONTOUR_FUNCTIONS_H__
#define __REFINE_CONTOUR_FUNCTIONS_H__

inline
const char* FacingToString(FacingType f)
{
    switch(f)
    {
    case CONTOUR: return "CONTOUR";
    case FRONT: return "FRONT";
    case BACK: return "BACK";
    default: assert(0); return NULL;
    }
}

inline
char FacingToChar(FacingType f)
{
    switch (f)
    {
    case CONTOUR: return 'C';
    case FRONT: return 'F';
    case BACK: return 'B';
    default: assert(0); return '?';
    }
}

inline
FacingType Facing(const vec3 & viewVec, const vec3 & normal, real threshold = CONTOUR_THRESHOLD,
                  real * n_dot_v = NULL)
// view vec is (position - cameraCenter). always normalized.
// only produces a normalized result if the input normal is normalized
{
    real norm = viewVec*viewVec;
    assert(norm > 0);

    vec3 vv = viewVec / sqrt(norm);

    real ndotv = normal*vv;

    if (n_dot_v != NULL)
        *n_dot_v = ndotv;

    if (ndotv < -threshold)
        return BACK;

    if (ndotv > threshold)
        return FRONT;

    return CONTOUR;
}

inline
FacingType Facing(const ParamPointCC & p, vec3 cameraCenter, real threshold = CONTOUR_THRESHOLD, real * ndotv = NULL)
{
    vec3 limitPosition, limitNormal;

    p.Evaluate(limitPosition, limitNormal);

    return Facing(limitPosition - cameraCenter, limitNormal, threshold, ndotv);
}

template<class T>
bool SmallTriangle(const HbrVertex<T> * v0, const HbrVertex<T> * v1, const HbrVertex<T> * v2)
{
    vec3 normal = GetNormal<T>(v0,v1,v2);

    return normal*normal/2 < MIN_INCONSISTENT_TRIANGLE_AREA;
}

template<class T>
bool SmallTriangle(HbrVertex<T> * const v[3])
{
    return SmallTriangle<T>(v[0],v[1],v[2]);
}

template<class T>
bool SmallTriangle(const HbrFace<T> * face)
{
    return SmallTriangle<T>(face->GetVertex(0), face->GetVertex(1), face->GetVertex(2));
}


// The face orientation according to the vertices of the face; CONTOUR if the face is not vertex-consistent
// e.g., for a face that's CCF, CFF, FFF, return F; for CCB, CBB, BBB, return B; otherwise return C.
// (The actual orientation of the face is irrelevant.)
template<class T>
FacingType VertexBasedFacing(HbrVertex<T> * v[3])
{
    FacingType ft[3] = { v[0]->GetData().facing, v[1]->GetData().facing, v[2]->GetData().facing };
    
    FacingType result = CONTOUR;
    
    for(int i=0;i<3;i++)
    {
        if (ft[i] == CONTOUR)
            continue;
        
        // check if we have a zero-crossing (both B and F)
        if (result != CONTOUR && ft[i] != result)
            return CONTOUR;
        
        result = ft[i];
    }
    
    return result;
}

template<class T>
FacingType VertexBasedFacing(const HbrFace<T> * face)
{
    HbrVertex<T> * v[3] = {face->GetVertex(0), face->GetVertex(1), face->GetVertex(2) };
    return VertexBasedFacing(v);
}

bool IsSplittable(HbrVertex<VertexDataCatmark> * v0, HbrVertex<VertexDataCatmark> * v1);

// Is the face consistent?  Treat cases that cannot be refined any further as consistent.
// Could separate those two questions, so that shifts and wiggles are tried on inconsistent triangles...
template<class T>
bool IsConsistent(HbrVertex<T> * v[3], const vec3 & cameraCenter, real * ndotv = NULL)
{
    vec3 normal = GetNormal<T>(v[0],v[1],v[2]);

    real nmag = sqrt(normal * normal);

    normal /= nmag;

    FacingType facing = Facing(v[0]->GetData().pos-cameraCenter,normal,0, ndotv);

    bool valid[3] = { true, true, true };

    // check if there are any zero-crossing edges.

    bool hasSplittable = false;

    // check for a splittable zero crossing
    for(int i=0;i<3;i++)
    {
        T & d0 = v[i]->GetData();
        T & d1 = v[(i+1)%3]->GetData();

        // if this edge isn't splittable, mark the vertices
        if ( !IsSplittable(v[i], v[(i+1)%3]))
        {
            valid[i] = false;
            valid[(i+1)%3] = false;
        }
        else
        {
            hasSplittable = true;

            // check for zero-crossing
            if (d0.facing != CONTOUR && d1.facing != CONTOUR && d0.facing != d1.facing)
                return false;
        }
    }

    // check for mismatch between vertex and face
    if (hasSplittable)
        for(int i=0;i<3;i++)
            if (v[i]->GetData().facing != CONTOUR && v[i]->GetData().facing != facing && valid[i])
                return false;

    // check for CCC
    if (hasSplittable && v[0]->GetData().facing == CONTOUR && v[1]->GetData().facing == CONTOUR &&
            v[2]->GetData().facing == CONTOUR)
        return false;

    return true;
}


template <class T>  // concistency checking for the optimizer; TODO: reconcile with the other IsConsistent...
bool IsConsistentOpt(HbrFace<T> * face, const vec3 & cameraCenter, real * ndotv = NULL) 
{
    FacingType vbf = VertexBasedFacing<T>(face);
    if (vbf == CONTOUR) // this function is only called in places that don't modify the VBF
        return false;
    int s = -(vbf == FRONT ? 1 : -1);   // Negative because source code uses v = p - c, derivation uses v=p-c.

    vec3 p[3] = { face->GetVertex(0)->GetData().pos, face->GetVertex(1)->GetData().pos, face->GetVertex(2)->GetData().pos };
    real R = (cameraCenter - p[0]) * ((p[2] - p[0]) ^ (p[1] - p[0]));

    if (ndotv != NULL)
        *ndotv = R;
    return s * R >= 0;
}

template<class T>
bool IsConsistent(HbrFace<T> * face, const vec3 & cameraCenter, real * ndotv = NULL)
{
    HbrVertex<T> * v[3] = { face->GetVertex(0), face->GetVertex(1), face->GetVertex(2) };
    return IsConsistent<T>(v, cameraCenter, ndotv);
}

template<class T>
void CullBackFaces(HbrMesh<T> * mesh)
{
    std::list<HbrFace<T>*> faces;
    mesh->GetFaces(std::back_inserter(faces));

    for(typename std::list<HbrFace<T>*>::iterator it = faces.begin(); it != faces.end(); it++)
    {
        HbrFace<T>* face = *it;

        if (face == NULL)
            continue;

        if (VertexBasedFacing<T>(face) == BACK)
            mesh->DeleteFace(face);
    }

    // delete any isolated vertices
    std::list<HbrVertex<T>*> verts;
    int n = 0, m = 0;
    mesh->GetVertices(std::back_inserter(verts));
    for(typename std::list<HbrVertex<T>*>::iterator it = verts.begin(); it != verts.end(); ++it)
    {
        HbrVertex<T> * vert = *it;
        if (vert == NULL)
            continue;

        m++;
        if (!vert->IsConnected())
        {
            assert(mesh->GetVertex(vert->GetID()) == vert);

            mesh->DeleteVertex(vert);
            n++;
        }
        else
            if (!vert->IsSingular() && vert->GetValence() < 2)
                printf("low valence: %d\n", vert->GetValence());
    }
    printf("deleted %d / %d isolated vertices\n",n,m);
}

template<class T>
HbrFace<T> * NewFaceDebug(HbrMesh<T> * mesh, int numVertices, int * IDs, bool tryReverse)
{
    bool bad = false;
    for(int i=0;i<numVertices;i++)
    {
        HbrVertex<T> * v0 = mesh->GetVertex(IDs[i]);
        HbrVertex<T> * v1 = mesh->GetVertex(IDs[(i+1)%3]);

        assert(v0 != v1);

        if (v0->GetEdge(v1) != NULL)
            bad = true;
    }

    if (bad)
    {
        printf("ERROR: INVALID FACE WITH DUPLICATED EDGE IN NewFace()\n");
        if (tryReverse)
        {
            printf("TRYING FLIPPED FACE\n");
            int flipIDs[numVertices];
            for(int i=0;i<numVertices;i++)
                flipIDs[i] = IDs[numVertices-i-1];
            return NewFaceDebug<T>(mesh, numVertices, flipIDs, false);
        }
        return NULL;
    }

    HbrFace<T> * face = mesh->NewFace(numVertices, IDs, 0);

    assert(!bad);

    return face;
}

template<class T>
HbrFace<T> * NewFaceDebug(HbrMesh<T> * mesh, HbrVertex<T> * v0, HbrVertex<T> * v1, HbrVertex<T> * v2)
{
    int IDs[3] = { v0->GetID(), v1->GetID(), v2->GetID() };
    return NewFaceDebug<T>(mesh, 3, IDs);
}

// for now, don't do normal projection for Catmarks
inline vec3
GetNormal(HbrVertex<VertexDataCatmark> * vertex)
{
    vec3 point, normal;
    vertex->GetData().sourceLoc.Evaluate(point, normal);
    return normal;
}

template<class T>
real ConsistencyEnergy(std::set<HbrFace<T>*>& meshFaceCluster, std::set<HbrVertex<T>*>& meshVertexCluster, std::map<HbrVertex<T>*,vec3> & origPos,
                       const vec3 & cameraCenter, real lambda, real epsilon, int & numInconsistent,
                       std::map<HbrVertex<T>*,vec3> * gradients)
{
    real energy = 0;
    numInconsistent = 0;

    if (gradients != NULL)
    {
        gradients->clear(); // storage is sparse
        assert(gradients->size() == 0);
    }

    // compute the consistency energy term

    for(typename std::set<HbrFace<T>*>::iterator fit = meshFaceCluster.begin(); fit != meshFaceCluster.end(); ++fit)
    {
        FacingType vbf = VertexBasedFacing<T>(*fit);
        if (vbf == CONTOUR)
            continue;
        int s = -(vbf == FRONT ? 1 : -1);   // Negative because source code uses v = p - c, derivation uses v=p-c.


        HbrVertex<T> * v[3] = { (*fit)->GetVertex(0), (*fit)->GetVertex(1), (*fit)->GetVertex(2) };
        vec3 p[3] = { v[0]->GetData().pos, v[1]->GetData().pos, v[2]->GetData().pos };
        real R = (cameraCenter - p[0]) * ((p[2] - p[0]) ^ (p[1] - p[0])); // /w;

        if ((s > 0) != (R > 0))
            numInconsistent ++;

        if (epsilon < s*R)
        {
            energy += -epsilon;
            continue;
        }

        energy += -s*R;

        if (gradients != NULL)
        {
            for(int i=0;i<3;i++)
            {
                // check if this vertex is a free variable
                if (meshVertexCluster.find( v[i]) == meshVertexCluster.end())
                    continue;

                vec3 dRdp_i = (p[(i+1)%3] - cameraCenter)^(p[(i+2)%3] - cameraCenter);
                typename std::map<HbrVertex<T>*,vec3>::iterator it = gradients->find(v[i]);
                if (it != gradients->end())
                    (*it).second += -s*dRdp_i;
                else
                    (*gradients)[v[i]] = -s*dRdp_i;

            }
        }
    }

    // add the squared-distance to the original positions, and then project the gradient onto normal direction (?)

    for(typename std::set<HbrVertex<T>*>::iterator it = meshVertexCluster.begin(); it != meshVertexCluster.end(); ++it)
    {
        if ((*it)->GetData().pos == origPos[*it])
            continue;

        vec3 d = (*it)->GetData().pos - origPos[*it];
        energy += lambda * (d*d);
        if (gradients != NULL)
        {
            typename std::map<HbrVertex<T>*,vec3>::iterator git = gradients->find(*it);
            if (git != gradients->end())
                (*git).second += lambda * d;
            else
                (*gradients)[*it] = lambda * d;

            // project onto normal direction
            vec3 normal = GetNormal( *it);
            (*gradients)[*it] = ((*gradients)[*it] * normal) * normal;
        }
    }

    return energy;
}


template<class T>
int OptimizeCluster(std::set<HbrFace<T>*> & meshFaceCluster, std::set<HbrVertex<T>*> & meshVertexCluster, std::map<HbrVertex<T>*,vec3> & origPos,
                    const vec3 & cameraCenter, real lambda, real epsilon)
{
    real stepSize = 1;

    std::map<HbrVertex<T>*,vec3> iterOrigVals;
    std::map<HbrVertex<T>*,vec3> gradients;

    //    int initialNumInconsistent;
    int numInconsistent;

    real energy = ConsistencyEnergy<T>(meshFaceCluster, meshVertexCluster, origPos, cameraCenter, lambda, epsilon, numInconsistent, NULL);
    printf("Initial Energy = %f, numInconsistent = %d\n", double(energy), numInconsistent);

    for(int iteration=0;iteration<100;iteration++)
    {
        // only need to compute this for elements with non-zero gradients
        for(typename std::set<HbrVertex<T>*>::iterator vit = meshVertexCluster.begin(); vit != meshVertexCluster.end(); vit++)
            iterOrigVals[*vit] = (*vit)->GetData().pos;

        // compute current energy and gradient
        energy = ConsistencyEnergy<T>(meshFaceCluster, meshVertexCluster, origPos, cameraCenter, lambda, epsilon, numInconsistent, &gradients);

        for(typename std::map<HbrVertex<T>*,vec3>::iterator git = gradients.begin(); git != gradients.end(); ++git) {
            (*git).first->GetData().pos = iterOrigVals[(*git).first] - stepSize * (*git).second;
        }

        real newEnergy = ConsistencyEnergy<T>(meshFaceCluster, meshVertexCluster, origPos, cameraCenter, lambda, epsilon,
                                              numInconsistent, NULL);

        // perform line-search
        while(newEnergy >= energy && stepSize > 1e-16)
        {
            stepSize /= 2;

            for(typename std::map<HbrVertex<T>*,vec3>::iterator git = gradients.begin(); git != gradients.end(); ++git)
                (*git).first->GetData().pos = iterOrigVals[(*git).first] - stepSize * (*git).second;

            newEnergy = ConsistencyEnergy<T>(meshFaceCluster, meshVertexCluster, origPos, cameraCenter, lambda, epsilon, numInconsistent, NULL);
        }

        if ((iteration%10) == 0 || iteration == 99)
            printf("Iteration %d.  Energy = %f, dE = %e, stepSize = %e, numInconsistent = %d\n",
                   iteration, double(newEnergy), double(newEnergy - energy), double(stepSize), numInconsistent);

        if (numInconsistent == 0 || stepSize <= 1e-16)
            break;

        stepSize *= 2;
    }

    return numInconsistent;
}


template<class T>
bool IsContourFace(HbrFace<T> * face)
{
    FacingType vbf = VertexBasedFacing<T>(face);
    if (vbf == CONTOUR)
        return false;
    for(int e=0;e<3;e++)
    {
        HbrFace<T> * oppFace;
        oppFace = GetOppFace(face,e);
        if (oppFace != NULL && VertexBasedFacing<T>(oppFace) != vbf)
            return true;
    }
    return false;
}


// search along the normal direction for the offset amount that minimizes the number of inconsistent adjacent triangles, and minimizes distance to source mesh
template<class T>
int WiggleVertex(HbrVertex<T> * vertex, const vec3 & cameraCenter, const vec3 & origPos)
{
    std::set<HbrFace<T>*> oneRing;
    GetOneRing<T>(vertex, oneRing);

    std::set<HbrVertex<T>*> oneRingVertices;
    GetOneRingVertices<T>(vertex, oneRingVertices);

    vec3 normal = GetNormal(vertex);

    const int contourScore = 1;

    int initialNumInconsistent = 0;
    for(typename std::set<HbrFace<T>*>::iterator it = oneRing.begin(); it != oneRing.end(); ++it)
        if (!IsConsistentOpt<T>(*it, cameraCenter))
        {
            if (IsContourFace<T>(*it))
                initialNumInconsistent += contourScore;
            else
                initialNumInconsistent ++;
        }

    real aveDistToORVs = 0;
    for(typename std::set<HbrVertex<T>*>::iterator it = oneRingVertices.begin(); it != oneRingVertices.end(); ++it)
        aveDistToORVs += length((*it)->GetData().pos - vertex->GetData().pos);
    aveDistToORVs /= oneRingVertices.size();

    int numSamples = 401;
    int h = (numSamples - 1)/2;
    int bestNumInconsistent = initialNumInconsistent;
    real bestDist = length(vertex->GetData().pos - origPos);
    vec3 curPos = vertex->GetData().pos;
    vec3 bestPos = vertex->GetData().pos;

    for (int i=-h;i<=h;i++)
    {
        if (i == 0)
            continue;

        int numInconsistent = 0;
        vertex->GetData().pos = curPos + aveDistToORVs / real(numSamples) * real(i) * normal;

        for(typename std::set<HbrFace<T>*>::iterator it = oneRing.begin(); it != oneRing.end(); ++it)
            if (!IsConsistentOpt<T>(*it, cameraCenter))
            {
                if (IsContourFace<T>(*it))
                    numInconsistent += contourScore;
                else
                    numInconsistent ++;
            }

        real dist = length(vertex->GetData().pos - origPos);

        if (numInconsistent < bestNumInconsistent || (numInconsistent == bestNumInconsistent && dist < bestDist))
        {
            bestPos = vertex->GetData().pos;
            bestNumInconsistent = numInconsistent;
            bestDist = dist;
        }
    }
    vertex->GetData().pos = bestPos;
    return initialNumInconsistent - bestNumInconsistent;
}

template<class T>
int WiggleFace(HbrFace<T> * face, const vec3 & cameraCenter, const vec3 origPos[3])
{
    printf("Wiggle face %p : ",face);

    std::set<HbrFace<T>*> adjacentFaces;

    for(int i=0; i<3; i++){
        std::set<HbrFace<T>*> oneRing;
        GetOneRing<T>(face->GetVertex(i), oneRing);
        for(typename std::set<HbrFace<T>*>::iterator it = oneRing.begin(); it != oneRing.end(); ++it)
            adjacentFaces.insert(*it);
    }
    const int contourScore = 1;

    int initialNumInconsistent = 0;
    for(typename std::set<HbrFace<T>*>::iterator it = adjacentFaces.begin(); it != adjacentFaces.end(); ++it)
        if (!IsConsistentOpt<T>(*it, cameraCenter))
        {
            if (IsContourFace<T>(*it))
                initialNumInconsistent += contourScore;
            else
                initialNumInconsistent ++;
        }

    vec3 normal[3];
    real aveDistToORVs[3] = {0.0,0.0,0.0};
    vec3 curPos[3];
    vec3 bestPos[3];
    real dist[3];
    for(int i=0; i<3; i++){
        std::set<HbrVertex<T>*> oneRingVertices;
        HbrVertex<T>* vertex = face->GetVertex(i);
        GetOneRingVertices<T>(vertex, oneRingVertices);
        for(typename std::set<HbrVertex<T>*>::iterator it = oneRingVertices.begin(); it != oneRingVertices.end(); ++it)
            aveDistToORVs[i] += length((*it)->GetData().pos - vertex->GetData().pos);
        aveDistToORVs[i] /= oneRingVertices.size();
        dist[i] = length(vertex->GetData().pos - origPos[i]);
        curPos[i] = vertex->GetData().pos;
        bestPos[i] = vertex->GetData().pos;
        normal[i] = GetNormal(vertex);
    }
    real bestDist = fmaxl(fmaxl(dist[0],dist[1]),dist[2]);

    int numSamples = 51;
    int h = (numSamples - 1)/2;
    int bestNumInconsistent = initialNumInconsistent;

    for (int i=-h;i<=h;i++)
    {
        face->GetVertex(0)->GetData().pos = curPos[0] + i * normal[0] * aveDistToORVs[0] / real(numSamples);
        dist[0] = length(face->GetVertex(0)->GetData().pos - origPos[0]);

        for (int j=-h;j<=h;j++)
        {
            face->GetVertex(1)->GetData().pos = curPos[1] + j * normal[1] * aveDistToORVs[1] / real(numSamples);
            dist[1] = length(face->GetVertex(1)->GetData().pos - origPos[1]);

            for (int k=-h;k<=h;k++)
            {
                if (i == 0 && j == 0 && k == 0)
                    continue;

                face->GetVertex(2)->GetData().pos = curPos[2] + k * normal[2] * aveDistToORVs[2] / real(numSamples);

                int numInconsistent = 0;

                for(typename std::set<HbrFace<T>*>::iterator it = adjacentFaces.begin(); it != adjacentFaces.end(); ++it)
                    if (!IsConsistentOpt<T>(*it, cameraCenter))
                    {
                        if (IsContourFace<T>(*it))
                            numInconsistent += contourScore;
                        else
                            numInconsistent ++;
                    }

                dist[2] = length(face->GetVertex(2)->GetData().pos - origPos[2]);
                real maxDist = fmaxl(fmaxl(dist[0],dist[1]),dist[2]);
                if (numInconsistent < bestNumInconsistent || (numInconsistent == bestNumInconsistent && maxDist < bestDist))
                {
                    for(int l=0; l<3; l++)
                        bestPos[l] = face->GetVertex(l)->GetData().pos;
                    bestNumInconsistent = numInconsistent;
                    bestDist = maxDist;
                }
            }
        }
    }

    for(int l=0; l<3; l++)
        face->GetVertex(l)->GetData().pos = bestPos[l];

    printf("%d \n", initialNumInconsistent - bestNumInconsistent);
    return initialNumInconsistent - bestNumInconsistent;
}

template<class T>
class WiggleCandidate
{
public:
    std::map<HbrVertex<T>*, int> offset;
    int _h;
    bool done;

    WiggleCandidate() { }
    WiggleCandidate(std::set<HbrVertex<T>*> & meshVertexCluster, int h)
    {
        _h = h;
        for(typename std::set<HbrVertex<T>*>::iterator it = meshVertexCluster.begin(); it != meshVertexCluster.end(); ++it)
            offset[*it] = -_h;
        done = false;
    }

    void increment()
    {
        typename std::map<HbrVertex<T>*, int>::iterator it = offset.begin();
        while (it != offset.end())
        {
            (*it).second ++;
            if ((*it).second < _h)
                return;
            (*it).second = -_h;
            it++;
        }
        done = true;
    }
};

template<class T>
void WiggleCluster(std::set<HbrFace<T>*> & meshFaceCluster, std::set<HbrVertex<T>*> & meshVertexCluster, std::map<HbrVertex<T>*,vec3> & origPos,
                   const vec3 & cameraCenter, real lambda, real epsilon)
{
    printf("%d  ", meshVertexCluster.size()); fflush(stdout);
    int numSamples = 11;
    int h = (numSamples - 1)/2;

    // compute one-ring distances
    std::map<HbrVertex<T>*, real> aveOneRingDist;
    for(typename std::set<HbrVertex<T>*>::iterator it = meshVertexCluster.begin(); it != meshVertexCluster.end(); ++it)
    {
        HbrVertex<T> * v = (*it);
        std::set<HbrVertex<T>*> oneRingVertices;
        GetOneRingVertices<T>(v, oneRingVertices);

        real aveDistToORVs = 0;
        for(typename std::set<HbrVertex<T>*>::iterator orvit = oneRingVertices.begin(); orvit != oneRingVertices.end(); ++orvit)
            aveDistToORVs += length((*orvit)->GetData().pos - v->GetData().pos);
        aveDistToORVs /= oneRingVertices.size();

        aveOneRingDist[v] = aveDistToORVs;
    }

    // loop over all possible wiggles of the vertices in the cluster
    WiggleCandidate<T> bestCandidate;
    int bestInconsistent = meshFaceCluster.size() + 1;

    int numTests = 0;
    for(WiggleCandidate<T> counter(meshVertexCluster, h); !counter.done; counter.increment())
    {
        // offset all vertices
        for(typename std::set<HbrVertex<T>*>::iterator it = meshVertexCluster.begin(); it != meshVertexCluster.end(); ++it)
        {
            HbrVertex<T> * v = (*it);
            v->GetData().pos = origPos[v] + .5*counter.offset[v] * GetNormal(v) * aveOneRingDist[v] / real(numSamples);
        }

        // TODO: prefer the offset with minimum deviation from the original surface

        int inconsistent = 0;
        // count the inconsistency in the neighborhood
        for (typename std::set<HbrFace<T>*>::iterator it = meshFaceCluster.begin(); it != meshFaceCluster.end() && bestInconsistent > 0; ++it)
            if (!IsConsistentOpt(*it, cameraCenter))
                inconsistent ++;

        if (inconsistent < bestInconsistent)
        {
            bestInconsistent = inconsistent;
            bestCandidate = counter;
        }
        numTests ++;
        assert(numTests < pow(numSamples+1, meshVertexCluster.size()));
    }

    // apply the best candidate
    for(typename std::set<HbrVertex<T>*>::iterator it = meshVertexCluster.begin(); it != meshVertexCluster.end(); ++it)
    {
        HbrVertex<T> * v = (*it);
        v->GetData().pos = origPos[v] + .5* bestCandidate.offset[v] * GetNormal(v) * aveOneRingDist[v] / real(numSamples);
    }

    printf("(%d). ", bestInconsistent);
}


template <class T>
void
ProcessClusters(HbrMesh<T> * mesh, const vec3 & cameraCenter, std::map<HbrVertex<T>*,vec3> & origPos, real lambda, real epsilon)
{
    printf("Wiggling clusters\n");
    std::set<HbrVertex<T>*> visitedVertices;
    int totalNumInconsistent = 0;

    std::list<HbrFace<T>*> meshFaces;
    std::list<HbrVertex<T>*> meshVertices;

    mesh->GetFaces(std::back_inserter(meshFaces));
    mesh->GetVertices(std::back_inserter(meshVertices));

    // determine which vertices are variables (active).  Those that are adjacent to an inconsistent face are variable
    int initialNumInconsistent =0;
    std::set<HbrVertex<T>*> activeVertices;
    for(typename std::list<HbrFace<T>*>::iterator fit= meshFaces.begin(); fit != meshFaces.end(); ++fit)
        if (!IsConsistentOpt(*fit, cameraCenter))
        {
            initialNumInconsistent ++;
            for(int i=0;i<3;i++)
                activeVertices.insert((*fit)->GetVertex(i));
        }

    for(typename std::set<HbrVertex<T>*>::iterator vit = activeVertices.begin(); vit != activeVertices.end(); ++vit)
    {
        if (visitedVertices.find(*vit) != visitedVertices.end())
            continue;

        // --------- create a cluster by flood fill ----------

        // a vertex is a active (variable) if it is adjacent to an inconsistent face
        // a face is active if it is adjacent to a active vertex
        // adjacent variable faces are put in the same cluster, as are their variable vertices

        std::set<HbrFace<T>*> meshFaceCluster;
        std::set<HbrVertex<T>*> meshVertexCluster;

        std::list<HbrVertex<T>*> vertexQueue;

        vertexQueue.push_back(*vit);
        meshVertexCluster.insert(*vit);
        visitedVertices.insert(*vit);

        while(vertexQueue.size() > 0)  // flood fill
        {
            HbrVertex<T> * v = vertexQueue.front();
            vertexQueue.pop_front();

            std::set<HbrFace<T>*> oneRing;
            GetOneRing<T>(v, oneRing);
            for(typename std::set<HbrFace<T>*>::iterator fit = oneRing.begin(); fit != oneRing.end(); ++fit)
            {
                if (meshFaceCluster.find(*fit) != meshFaceCluster.end())
                    continue;
                meshFaceCluster.insert(*fit);
                for(int i=0;i<3;i++)
                {
                    HbrVertex<T> * v = (*fit)->GetVertex(i);
                    if (visitedVertices.find(v) == visitedVertices.end() && activeVertices.find(v) != activeVertices.end())
                    {
                        vertexQueue.push_back(v);
                        meshVertexCluster.insert(v);
                        visitedVertices.insert(v);
                    }
                }
            }
        }

        // ---------- optimize the cluster ------------
        //            WiggleCluster<T>(meshFaceCluster, meshVertexCluster, origPos, cameraCenter, lambda, epsilon);
        OptimizeCluster<T>(meshFaceCluster, meshVertexCluster, origPos, cameraCenter, lambda, epsilon);
    }

    totalNumInconsistent = 0;

    for(typename std::list<HbrFace<T>*>::iterator fit= meshFaces.begin(); fit != meshFaces.end(); ++fit)
        if (!IsConsistentOpt(*fit, cameraCenter))
            totalNumInconsistent ++;
    printf("\nTOTAL INCONSISTENT AFTER CLUSTER WIGGLING/OPTIMIZING: %d (was %d)\n", totalNumInconsistent, initialNumInconsistent);
}


template<class T>
void FilterNdotV(HbrMesh<T> * mesh)
{
    std::list<HbrVertex<T>*> meshVertices;
    mesh->GetVertices(std::back_inserter(meshVertices));

    for(typename std::list<HbrVertex<T>*>::iterator it = meshVertices.begin(); it!=meshVertices.end();it++)
    {
        HbrVertex<T> * vertex = *it;
        std::set<HbrVertex<T>*> oneRingVertices;
        GetOneRingVertices<T>(vertex, oneRingVertices);

        FacingType myFacing = vertex->GetData().facing;
        FacingType oppFacing = myFacing == FRONT ? BACK : FRONT;

        int numSame = 0;
        for(typename std::set<HbrVertex<T>*>::iterator orit = oneRingVertices.begin(); orit != oneRingVertices.end(); ++orit)
            if ( (*orit)->GetData().facing == myFacing)
                numSame ++;

        if (numSame <= 1)
        {
            vertex->GetData().facing = oppFacing;
            vertex->GetData().ndotv = oppFacing == FRONT ?   CONTOUR_THRESHOLD + 1e-5 : -CONTOUR_THRESHOLD - 1e-5;
        }
    }
}


inline
vec3 GetOrigPos(const VertexDataCatmark & vd)
{
    // this should probably just be precomputed and stored as "origPos"
    ParamPointCC srcPt = vd.sourceLoc;
    srcPt.SetNormalOffset(0);
    vec3 pos, normal;
    srcPt.Evaluate(pos, normal);
    return pos;
}

template<class T>
void OptimizeConsistency(HbrMesh<T> * outputMesh, const vec3 & cameraCenter, real lambda, real epsilon)
{
    printf("Optimizing consistency.\n");

    std::list<HbrFace<T>*> meshFaces;
    std::list<HbrVertex<T>*> meshVertices;

    outputMesh->GetFaces(std::back_inserter(meshFaces));
    outputMesh->GetVertices(std::back_inserter(meshVertices));

    // save the original positions (would prefer to use limit positions or whatever)
    std::map<HbrVertex<T>*,vec3> origPos;

    for(typename std::list<HbrVertex<T>*>::iterator it = meshVertices.begin(); it != meshVertices.end(); ++it)
        origPos[*it] = (*it)->GetData().pos;

    // optimize everything jointly
    std::set<HbrFace<T>*> meshFaceCluster;
    std::set<HbrVertex<T>*> meshVertexCluster;

    for(typename std::list<HbrFace<T>*>::iterator fit = meshFaces.begin(); fit != meshFaces.end(); ++fit)
        meshFaceCluster.insert(*fit);

    for(typename std::list<HbrVertex<T>*>::iterator vit = meshVertices.begin(); vit != meshVertices.end(); ++vit)
        meshVertexCluster.insert(*vit);

    OptimizeCluster(meshFaceCluster, meshVertexCluster, origPos, cameraCenter, lambda, epsilon);


}

template<class T>
void WiggleAllFaces(HbrMesh<T> * outputMesh, const vec3 & cameraCenter)
{
    // wiggle conjointly all vertices of an inconsistent face.

    printf("Wiggling face:\n");

    std::list<HbrFace<T>*> meshFaces;
    std::list<HbrVertex<T>*> meshVertices;
    outputMesh->GetFaces(std::back_inserter(meshFaces));
    outputMesh->GetVertices(std::back_inserter(meshVertices));

    // save the original positions (would prefer to use limit positions or whatever)
    std::map<HbrVertex<T>*,vec3> origPos;

    for(typename std::list<HbrVertex<T>*>::iterator it = meshVertices.begin(); it != meshVertices.end(); ++it)
        origPos[*it] = (*it)->GetData().pos;

    int reduction;
    int pass = 1;
    do{

        reduction = 0;
        printf("pass %d\n",pass++);
        fflush(stdout);
        for(typename std::list<HbrFace<T>*>::iterator fit= meshFaces.begin(); fit != meshFaces.end(); ++fit)
        {
            if (!IsConsistentOpt<T>(*fit , cameraCenter)){
                vec3 pos[3] = {origPos[(*fit)->GetVertex(0)], origPos[(*fit)->GetVertex(1)], origPos[(*fit)->GetVertex(2)]};
                reduction += WiggleFace<T>((*fit), cameraCenter, pos);
            }
        }
    }while (reduction > 0);

    printf("\n");
}

template<class T>
void WiggleAllVertices(HbrMesh<T> * outputMesh, const vec3 & cameraCenter)
{
    // wiggle each vertex of an inconsistent face.

    
    printf("Wiggling individual vertices: passes");

    std::list<HbrFace<T>*> meshFaces;
    std::list<HbrVertex<T>*> meshVertices;
    outputMesh->GetFaces(std::back_inserter(meshFaces));
    outputMesh->GetVertices(std::back_inserter(meshVertices));

    // save the original positions (would prefer to use limit positions or whatever)
    std::map<HbrVertex<T>*,vec3> origPos;

    for(typename std::list<HbrVertex<T>*>::iterator it = meshVertices.begin(); it != meshVertices.end(); ++it)
        origPos[*it] = (*it)->GetData().pos;

    int reduction;
    int pass = 1;
    // this could be made much more efficient by keeping a vertex or face queue
    do{

        reduction = 0;
        printf(" %d",pass++);
        fflush(stdout);
        for(typename std::list<HbrFace<T>*>::iterator fit= meshFaces.begin(); fit != meshFaces.end(); ++fit)
        {
            if (!IsConsistentOpt<T>(*fit , cameraCenter))
                for(int i=0;i<3;i++)
                    reduction += WiggleVertex<T>( (*fit)->GetVertex(i), cameraCenter, origPos[(*fit)->GetVertex(i)]);
        }
        printf("(%d)",reduction);
    }while (reduction > 0);

    printf("\n");


    // wiggle/optimize clusters
    // danger: code below assumes origPos hasn't changed

    int numInconsistent = 0;
    for(typename std::list<HbrFace<T>*>::iterator it = meshFaces.begin(); it != meshFaces.end(); ++it)
        if (!IsConsistentOpt(*it, cameraCenter))
            numInconsistent ++;

    // update the normal offsets
    for(typename std::list<HbrVertex<T>*>::iterator it = meshVertices.begin(); it != meshVertices.end(); it++)
    {
        vec3 motion = (*it)->GetData().pos - origPos[*it];
        vec3 normal = GetNormal(*it);
        real oldOffset = (*it)->GetData().sourceLoc.NormalOffset();
        (*it)->GetData().sourceLoc.SetNormalOffset( oldOffset + motion * normal);
    }

    printf("Final # inconsistent faces: %d\n", numInconsistent);
}


///////////////////////////////////////////// PRIORITY QUEUE /////////////////////////////////

template<class T>
HbrFace<T> * FacePriorityQueue<T>::PopFront()
{
    assert(Size()!=0);

    HbrFace<T> * face;
    typename std::set<HbrFace<T>*>::iterator rit;

    do
    {
        typename std::multimap<real,HbrFace<T>*>::iterator it=_queue.end();
        --it;
        face = (*it).second;
        _queue.erase(it);

        rit = _readySet.find(face);
    }
    while (rit == _readySet.end());

    _readySet.erase(rit);

    return face;
}

template<class T>
void FacePriorityQueue<T>::Insert(HbrFace<T>* face)
{
    if (_readySet.find(face) != _readySet.end())
        return;

    _readySet.insert(face);

    _queue.insert(std::pair<real,HbrFace<T>*>(_Priority(face),face));
}

template<class T>
void FacePriorityQueue<T>::Remove(HbrFace<T> * face)
{
    if (_readySet.find(face) == _readySet.end())
        return;

    _readySet.erase(face);

    // could also remove from the queue and then simplify "PopFront()"
}  


template<class T>
real FacePriorityQueue<T>::_Priority(const HbrFace<T> * face) const
{
    return GetArea<T>(face);
}



template<class T>
void MakeSplitFaceIndices(HbrHalfedge<T> * edge, HbrVertex<T> * vnew, HbrVertex<T> * newFaces[4][3])
// given an edge and a vertex to insert in that edge, determine the indexes (with correct orientations)
// of the vertices to be inserted in the new triangles
{
    // four new possible faces, each of 3 vertices
    for(int i=0;i<4;i++)
        for(int j=0;j<3;j++)
            newFaces[i][j] = NULL;

    HbrVertex<T> * v1 = edge->GetOrgVertex();
    HbrVertex<T> * v2 = edge->GetDestVertex();

    // gather the adjacent faces/vertices with consistent ordering
    if (edge->GetLeftFace() != NULL)
    {
        HbrFace<T> * leftFace = edge->GetLeftFace();
        HbrVertex<T> * v[3] = { leftFace->GetVertex(0),leftFace->GetVertex(1),leftFace->GetVertex(2)};
        int e;
        for(e=0;e<3;e++)
            if ((v[e] == v1 && v[(e+1)%3] == v2) || (v[e] == v2 && v[(e+1)%3] == v1))
                break;
        assert(e < 3);
        newFaces[0][0] = v[e]; newFaces[0][1] = vnew;       newFaces[0][2] = v[(e+2)%3];
        newFaces[1][0] = vnew; newFaces[1][1] = v[(e+1)%3]; newFaces[1][2] = v[(e+2)%3];
    }

    if (edge->GetRightFace() != NULL)
    {
        HbrFace<T> * rightFace = edge->GetRightFace();
        HbrVertex<T> * v[3] = { rightFace->GetVertex(0),rightFace->GetVertex(1),rightFace->GetVertex(2)};
        int e;
        for(e=0;e<3;e++)
            if ((v[e] == v1 && v[(e+1)%3] == v2) || (v[e] == v2 && v[(e+1)%3] == v1))
                break;
        assert(e < 3);
        newFaces[2][0] = v[e]; newFaces[2][1] = vnew;       newFaces[2][2] = v[(e+2)%3];
        newFaces[3][0] = vnew; newFaces[3][1] = v[(e+1)%3]; newFaces[3][2] = v[(e+2)%3];
    }
}




void SetupVertex(VertexDataCatmark & vd, const ParamPointCC & p, const vec3 & cameraCenter);

inline
bool CheesyInterpolatePoint(VertexDataCatmark & newData, VertexDataCatmark & vd0, VertexDataCatmark & vd1, real t, const vec3 & cameraCenter)
{
    ParamPointCC newPt = ParamPointCC::Interpolate(vd0.sourceLoc, vd1.sourceLoc, t);
    if (newPt.IsNull())
        return false;
    SetupVertex(newData, newPt, cameraCenter);
    return true;
}

#define MYTEST(x)  ;

inline
bool Inverse(real A[3][3])
{
    real B[3][3];//the transpose of a matrix A
    real C[3][3];//the adjunct matrix of transpose of a matrix A not adjunct of A
    int i,j;
    real x,n=0;//n is the determinant of A

    for(i=0,j=0;j<3;j++){
        if(j==2)
            n+=A[i][j]*A[i+1][0]*A[i+2][1];
        else if(j==1)
            n+=A[i][j]*A[i+1][j+1]*A[i+2][0];
        else
            n+=A[i][j]*A[i+1][j+1]*A[i+2][j+2];
    }
    for(i=2,j=0;j<3;j++){
        if(j==2)
            n-=A[i][j]*A[i-1][0]*A[i-2][1];
        else if(j==1)
            n-=A[i][j]*A[i-1][j+1]*A[i-2][0];
        else
            n-=A[i][j]*A[i-1][j+1]*A[i-2][j+2];
    }

    if(fabsl(n)>0.0)
        x=1.0/n;
    else{
        return false;
    }

    for(i=0;i<3;i++){
        for(j=0;j<3;j++){
            B[i][j]=A[j][i];
        }
    }

    C[0][0]=B[1][1]*B[2][2]-(B[2][1]*B[1][2]);
    C[0][1]=-(B[1][0]*B[2][2]-(B[2][0]*B[1][2]));
    C[0][2]=B[1][0]*B[2][1]-(B[2][0]*B[1][1]);

    C[1][0]=-(B[0][1]*B[2][2]-B[2][1]*B[0][2]);
    C[1][1]=B[0][0]*B[2][2]-B[2][0]*B[0][2];
    C[1][2]=-(B[0][0]*B[2][1]-B[2][0]*B[0][1]);

    C[2][0]=B[0][1]*B[1][2]-B[1][1]*B[0][2];
    C[2][1]=-(B[0][0]*B[1][2]-B[1][0]*B[0][2]);
    C[2][2]=B[0][0]*B[1][1]-B[1][0]*B[0][1];

    for(i=0;i<3;i++){
        for(j=0;j<3;j++){
            A[i][j]=C[i][j]*x;
        }
    }
    return true;
}

template<class T>
bool Outside(vec3 & pos0, vec3 & pos1, vec3 & pos2, vec3 & testPos,
             real & lambda, real & mu)
{
    vec3 trisN = GetNormal(pos0,pos1,pos2);

    vec3 p = testPos - pos0;
    vec3 a = pos1 - pos0;
    vec3 b = pos2 - pos0;
    real M[3][3];
    M[0][0]= a[0];
    M[0][1]= a[1];
    M[0][2]= a[2];

    M[1][0]= b[0];
    M[1][1]= b[1];
    M[1][2]= b[2];

    M[2][0]= trisN[0];
    M[2][1]= trisN[1];
    M[2][2]= trisN[2];
    if(!Inverse(M)){
        p = testPos - pos1;
        a = pos0 - pos1;
        b = pos2 - pos1;

        M[0][0]= a[0];
        M[0][1]= a[1];
        M[0][2]= a[2];

        M[1][0]= b[0];
        M[1][1]= b[1];
        M[1][2]= b[2];

        M[2][0]= trisN[0];
        M[2][1]= trisN[1];
        M[2][2]= trisN[2];

        if(!Inverse(M)){
            p = testPos - pos2;
            a = pos0 - pos2;
            b = pos2 - pos2;

            M[0][0]= a[0];
            M[0][1]= a[1];
            M[0][2]= a[2];

            M[1][0]= b[0];
            M[1][1]= b[1];
            M[1][2]= b[2];

            M[2][0]= trisN[0];
            M[2][1]= trisN[1];
            M[2][2]= trisN[2];
            if(!Inverse(M)){
#ifdef LINK_FREESTYLE
                char str[200];
                sprintf(str, "CAN'T INVERSE MATRIX");
                addRIFDebugPoint(-1,double(testPos[0]),double(testPos[1]),double(testPos[2]),str,0);
#endif
                return false;
            }
        }
    }

    lambda = M[0][0] * p[0] + M[1][0] * p[1] + M[2][0] * p[2];
    mu =     M[0][1] * p[0] + M[1][1] * p[1] + M[2][1] * p[2];

    if(lambda<0.0 || mu<0.0 || (lambda+mu) > 1.0)
        return true;
    return false;
}

template<class T>
bool Outside(HbrVertex<T>* v[3], HbrVertex<T>* testV, real & lambda, real & mu)
{
    return Outside<T>(v[0]->GetData().pos,v[1]->GetData().pos,v[2]->GetData().pos,testV->GetData().pos,lambda,mu);
}

template<class T>
bool Outside(HbrFace<T>* f, HbrVertex<T>* testV, real & lambda, real & mu)
{
    HbrVertex<T>* vertices[3] = { f->GetVertex(0), f->GetVertex(1), f->GetVertex(2) };
    return Outside(vertices,testV,lambda,mu);
}

template<class T>
void FlipAndInsert(HbrVertex<T>* v[3], HbrVertex<T>* newV, HbrFace<T>* face, HbrMesh<T>* mesh, std::list< HbrFace<T>* > & f,
FacePriorityQueue<T> & wiggleQueue, FacePriorityQueue<T> & splitQueue, bool skip[3])
{
    real lambda, mu;
    if(Outside(v,newV,lambda,mu)){
        if(lambda>=0 && mu<0){
            if(v[0]->GetData().facing!=CONTOUR || v[1]->GetData().facing!=CONTOUR){
                //                printf("\nOUTSIDE 1\n");
                //Flip v[0]v[1]
                HbrHalfedge<T>* adjEdge = v[0]->GetEdge(v[1]) ? v[0]->GetEdge(v[1]) : v[1]->GetEdge(v[0]);
                HbrFace<T>* adjFace = adjEdge->GetLeftFace() != face ? adjEdge->GetLeftFace() : adjEdge->GetRightFace();
                if (adjFace){
                    HbrVertex<T>* oppositeV;
                    for(int i=0; i<3; i++){
                        assert(adjFace != NULL);
                        oppositeV = adjFace->GetVertex(i);
                        if(oppositeV!=v[0] && oppositeV!=v[1])
                            break;
                    }
                    if(oppositeV->GetData().facing==CONTOUR)
                        return;
                    wiggleQueue.Remove(adjFace);
                    splitQueue.Remove(adjFace);
                    mesh->DeleteFace(adjFace);
                    if(v[0]->GetEdge(v[1])){
                        f.push_back(NewFace<T>(mesh, newV, v[0], oppositeV));
                        f.push_back(NewFace<T>(mesh, newV, oppositeV, v[1]));
                    }else{
                        f.push_back(NewFace<T>(mesh, newV, oppositeV, v[0]));
                        f.push_back(NewFace<T>(mesh, newV, v[1], oppositeV));
                    }
                    skip[0] = true;
                }
            }
        }else if(lambda<0 && mu>=0){
            if(v[0]->GetData().facing!=CONTOUR || v[2]->GetData().facing!=CONTOUR){
                //Flip v[0]v[2]
                HbrHalfedge<T>* adjEdge = v[0]->GetEdge(v[2]) ? v[0]->GetEdge(v[2]) : v[2]->GetEdge(v[0]);
                HbrFace<T>* adjFace = adjEdge->GetLeftFace() != face ? adjEdge->GetLeftFace() : adjEdge->GetRightFace();
                if(adjFace){
                    HbrVertex<T>* oppositeV;
                    for(int i=0; i<3; i++){
                        assert(adjFace != NULL);
                        oppositeV = adjFace->GetVertex(i);
                        if(oppositeV!=v[0] && oppositeV!=v[2])
                            break;
                    }
                    if(oppositeV->GetData().facing==CONTOUR)
                        return;
                    wiggleQueue.Remove(adjFace);
                    splitQueue.Remove(adjFace);
                    mesh->DeleteFace(adjFace);
                    if(v[2]->GetEdge(v[0])){
                        f.push_back(NewFace<T>(mesh, newV, v[2], oppositeV));
                        f.push_back(NewFace<T>(mesh, newV, oppositeV, v[0]));
                    }else{
                        f.push_back(NewFace<T>(mesh, newV, oppositeV, v[2]));
                        f.push_back(NewFace<T>(mesh, newV, v[0], oppositeV));
                    }
                    skip[2] = true;
                }
            }
        }else if((lambda+mu) > 1.0){
            if(v[1]->GetData().facing!=CONTOUR || v[2]->GetData().facing!=CONTOUR){
                //Flip v[1]v[2]
                HbrHalfedge<T>* adjEdge = v[1]->GetEdge(v[2]) ? v[1]->GetEdge(v[2]) : v[2]->GetEdge(v[1]);
                HbrFace<T>* adjFace = adjEdge->GetLeftFace() != face ? adjEdge->GetLeftFace() : adjEdge->GetRightFace();
                if(adjFace){
                    HbrVertex<T>* oppositeV;
                    for(int i=0; i<3; i++){
                        assert(adjFace != NULL);
                        oppositeV = adjFace->GetVertex(i);
                        if(oppositeV!=v[1] && oppositeV!=v[2])
                            break;
                    }
                    if(oppositeV->GetData().facing==CONTOUR)
                        return;
                    wiggleQueue.Remove(adjFace);
                    splitQueue.Remove(adjFace);
                    mesh->DeleteFace(adjFace);
                    if(v[1]->GetEdge(v[2])){
                        f.push_back(NewFace<T>(mesh, newV, v[1], oppositeV));
                        f.push_back(NewFace<T>(mesh, newV, oppositeV, v[2]));
                    }else{
                        f.push_back(NewFace<T>(mesh, newV, oppositeV, v[1]));
                        f.push_back(NewFace<T>(mesh, newV, v[2], oppositeV));
                    }
                    skip[1] = true;
                }
            }
        }else{
            printf("\nCAN'T FLIP\n");
        }
    }
}

template<class T>
void InsertVertex(HbrFace<T> * face, int oppVertex, HbrVertex<T> * newVertex, HbrMesh<T> * mesh, 
                  FacePriorityQueue<T> & wiggleQueue,FacePriorityQueue<T> & splitQueue,
                  bool enqueueNewFaces = true, bool enqueueNewOppFaces = true)
{
    HbrVertex<T> * v0 = face->GetVertex(oppVertex);
    HbrVertex<T> * v1 = face->GetVertex( (oppVertex+1)%3 );
    HbrVertex<T> * v2 = face->GetVertex( (oppVertex+2)%3 );

    MYTEST("a\n");

    int eOpp;
    HbrFace<T> * oppFace = GetOppFace<T>(face, oppVertex, eOpp);

    std::list<HbrFace<T>*> newFaces;
    bool skip0[3] = {false,false,false};
    bool skip1[3] = {false,false,false};

    real lambda0, mu0;
    HbrVertex<T>* vertices0[3] = {v0,v1,v2};
    if(Outside(vertices0,newVertex,lambda0,mu0)){
        if(oppFace){
            real lambda1, mu1;
            HbrVertex<T>* vertices1[3] = {oppFace->GetVertex(eOpp),v1,v2};
            if(Outside(vertices1,newVertex,lambda1,mu1)){
                if(lambda0<0 || mu0<0){
                    FlipAndInsert(vertices0,newVertex,face,mesh,newFaces,wiggleQueue,splitQueue,skip0);
                }else if(lambda1<0 || mu1<0){
                    FlipAndInsert(vertices1,newVertex,oppFace,mesh,newFaces,wiggleQueue,splitQueue,skip1);
                }
            }
        }else if(lambda0<0 || mu0<0)
            FlipAndInsert(vertices0,newVertex,face,mesh,newFaces,wiggleQueue,splitQueue,skip0);
    }

    wiggleQueue.Remove(face);
    splitQueue.Remove(face);

    MYTEST("b\n");

    mesh->DeleteFace(face);

    MYTEST("c\n");

    HbrFace<T> * f1 = !skip0[0] ? NewFace<T>(mesh, v0, v1, newVertex) : NULL;
    MYTEST("c1\n");

    HbrFace<T> * f2 = !skip0[2] ? NewFace<T>(mesh, v0, newVertex, v2) : NULL;

    MYTEST("d\n");

    if (enqueueNewFaces)
    {
        if(f1){
            wiggleQueue.Insert(f1);
            splitQueue.Insert(f1);
        }
        if(f2){
            wiggleQueue.Insert(f2);
            splitQueue.Insert(f2);
        }
    }

    assert((skip0[0] || GetArea<T>(f1) > 0) && (skip0[2] ||GetArea<T>(f2) > 0));

    MYTEST("e\n");


    if (oppFace != NULL) // split the opposite face as well
    {
        HbrVertex<T> * v3 = oppFace->GetVertex(eOpp);

        wiggleQueue.Remove(oppFace);
        splitQueue.Remove(oppFace);

        MYTEST("F\n");

        mesh->DeleteFace(oppFace);

        HbrFace<T> * f3 = !skip1[0] ? NewFace<T>(mesh, newVertex, v1, v3) : NULL;
        HbrFace<T> * f4 = !skip1[2] ? NewFace<T>(mesh, v2, newVertex, v3) : NULL;

        MYTEST("G\n");

        if((!skip1[0] && !(GetArea<T>(f3) > 0)) || (!(skip1[2]) && !(GetArea<T>(f4) > 0))){
            printf("Initial area: %.16f\n",double(GetArea(v1,v2,v3)));
            if(!skip1[0])
                printf("f3: %.16f / %.16f %.16f %.16f\n",double(GetArea<T>(f3)),
                       double(length(f3->GetVertex(0)->GetData().pos-f3->GetVertex(1)->GetData().pos)),
                       double(length(f3->GetVertex(1)->GetData().pos-f3->GetVertex(2)->GetData().pos)),
                       double(length(f3->GetVertex(2)->GetData().pos-f3->GetVertex(0)->GetData().pos)));
            if(!skip1[2])
                printf("f4: %.16f / %.16f %.16f %.16f\n",double(GetArea<T>(f4)),
                       double(length(f4->GetVertex(0)->GetData().pos-f4->GetVertex(1)->GetData().pos)),
                       double(length(f4->GetVertex(1)->GetData().pos-f4->GetVertex(2)->GetData().pos)),
                       double(length(f4->GetVertex(2)->GetData().pos-f4->GetVertex(0)->GetData().pos)));
            v3->GetData().sourceLoc.Print();
            newVertex->GetData().sourceLoc.Print();

#if LINK_FREESTYLE
            char str[200];
            sprintf(str, "SPLIT");
            addRIFDebugPoint(-1, double(newVertex->GetData().pos[0]), double(newVertex->GetData().pos[1]), double(newVertex->GetData().pos[2]), str, 0);
#endif
        }
        assert((skip1[0] || GetArea<T>(f3) > 0) && (skip1[2] ||GetArea<T>(f4) > 0));

        if (enqueueNewOppFaces)
        {
            if(!skip1[0]){
                wiggleQueue.Insert(f3);
                splitQueue.Insert(f3);
            }
            if(!skip1[2]){
                wiggleQueue.Insert(f4);
                splitQueue.Insert(f4);
            }
        }

    }

    MYTEST("J\n");
}

template<class T>
void InsertVertex(HbrFace<T> * face, int oppVertex, HbrVertex<T> * newVertex, HbrMesh<T> * mesh, std::vector<HbrFace<T>*> & deletedFaces)
{
    HbrVertex<T> * v0 = face->GetVertex(oppVertex);
    HbrVertex<T> * v1 = face->GetVertex( (oppVertex+1)%3 );
    HbrVertex<T> * v2 = face->GetVertex( (oppVertex+2)%3 );

    int eOpp;
    HbrFace<T> * oppFace = GetOppFace<T>(face, oppVertex, eOpp);

    mesh->DeleteFace(face);
    deletedFaces.push_back(face);

    HbrFace<T> * f1 = NewFace<T>(mesh, v0, v1, newVertex);
    HbrFace<T> * f2 = NewFace<T>(mesh, v0, newVertex, v2);

    assert(GetArea<T>(f1) > 0 && GetArea<T>(f2) > 0);

    if (oppFace != NULL) // split the opposite face as well
    {
        HbrVertex<T> * v3 = oppFace->GetVertex(eOpp);

        mesh->DeleteFace(oppFace);
        deletedFaces.push_back(oppFace);

        HbrFace<T> * f3 = NewFace<T>(mesh, newVertex, v1, v3);
        HbrFace<T> * f4 = NewFace<T>(mesh, v2, newVertex, v3);

        assert(GetArea<T>(f3) > 0 && GetArea<T>(f4) > 0);
    }
}


template<class T>
bool HasTwoContourPoints(HbrFace<T> * face)
{
    int n = 0;
    for(int i=0;i<3;i++)
        if (face->GetVertex(i)->GetData().facing == CONTOUR)
            n++;
    return n>=2;
}


template<class T>
bool TooSmallToSplit(HbrFace<T> * face, int oppVertex, HbrVertex<T> * newVertex, real areaThreshold)
{
    HbrVertex<T> * newFaces[4][3];

    MakeSplitFaceIndices(face->GetEdge((oppVertex+1)%3),newVertex,newFaces);

    for(int f = 0; f<4;f++)
        if (newFaces[f][0] != NULL && GetArea(newFaces[f][0], newFaces[f][1], newFaces[f][2]) < areaThreshold)
            return true;

    return false;
}


bool FindContour(const ParamPointCC & p0, const ParamPointCC & p1, vec3 cameraCenter, ParamPointCC & resultPoint);


#endif

