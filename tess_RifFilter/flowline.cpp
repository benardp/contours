#include <map>
#include <deque>
#include <set>
#include <iostream>

#include <math.h>  // for FLT_EPS?

#include "refineContour.h"

typedef HbrMesh<VertexDataCatmark> Mesh;
typedef HbrVertex<VertexDataCatmark> MeshVertex;
typedef HbrFace<VertexDataCatmark> MeshFace;
typedef HbrHalfedge<VertexDataCatmark> MeshEdge;


struct FlowLinePoint;

struct ContourPoint
{
    MeshVertex * vertex; // new vertex in the output mesh
    MeshEdge * sourceEdge; // edge of the original quad mesh that this point came from

    ContourPoint * siblings[2];
    FlowLinePoint * nearChild;
    FlowLinePoint * farChild;

    ContourPoint() { siblings[0] = siblings[1] = NULL; nearChild = farChild = NULL; sourceEdge = NULL; }
    int NumSiblings() { if (siblings[0] == NULL) return 0; else if (siblings[1] == NULL) return 1; else return 2; }
    void AddSibling(ContourPoint * p) { int N = NumSiblings(); assert(N<2); siblings[N] = p; }
    bool HasSibling(ContourPoint *p) { return siblings[0] == p || siblings[1] == p; }
    ContourPoint * OtherSibling(ContourPoint * p) { assert(HasSibling(p)); if (NumSiblings() != 2) return NULL;
        return siblings[0] != p ? siblings[0] : siblings[1]; }
};

struct FlowLinePoint
{
    MeshVertex * vertex; // new vertex in the output mesh
    NsdVVHalfedge * sourceEdgeVV; // edge of the original subdiv base mesh

    MeshVertex * stitchTarget;

    FlowLinePoint * siblings[2];

    ContourPoint * parent;

    FlowLinePoint() { siblings[0] = siblings[1] = NULL; parent = NULL; stitchTarget = NULL; sourceEdgeVV = NULL; }
    int NumSiblings() { if (siblings[0] == NULL) return 0; else if (siblings[1] == NULL) return 1; else return 2; }
    void AddSibling(FlowLinePoint* p) { int N = NumSiblings(); assert(N<2); siblings[N] = p; }
    bool HasSibling(FlowLinePoint *p) { return siblings[0] == p || siblings[1] == p; }
    FlowLinePoint * OtherSibling(FlowLinePoint * p) { assert(HasSibling(p)); if (NumSiblings() != 2) return NULL;
        return siblings[0] != p ? siblings[0] : siblings[1]; }
};


struct FlowLineCluster
{
    MeshVertex * stitchVertex;

    std::set<FlowLinePoint*> parents;
    std::list<FlowLinePoint*> parentsOrdered;

    std::set<MeshVertex*> existingPoints; // original mesh vertices to be stitched to this cluster

    FlowLineCluster * siblings[2];
    FlowLinePoint * endpoints[2]; // the parent that connects to each sibling's parent

    FlowLineCluster() { siblings[0] = siblings[1] = NULL; }
    int NumSiblings() { if (siblings[0] == NULL) return 0; else if (siblings[1] == NULL) return 1; else return 2; }
    void AddSibling(FlowLineCluster* sib, FlowLinePoint * endpoint) { int N = NumSiblings(); assert(N<2); siblings[N] = sib; endpoints[N] = endpoint;}
    bool HasSibling(FlowLineCluster *sib) { return siblings[0] == sib || siblings[1] == sib; }
    FlowLineCluster * OtherSibling(FlowLineCluster * p) { assert(HasSibling(p)); if (NumSiblings() != 2) return NULL;
        return siblings[0] != p ? siblings[0] : siblings[1]; }
};


struct FaceRec
{
    MeshVertex * v[3];

    FaceRec(MeshVertex *v0, MeshVertex * v1, MeshVertex * v2) { v[0] = v0; v[1] = v1; v[2] = v2; }
};


MeshFace * NewFaceDebug(Mesh * mesh, MeshVertex * v0, MeshVertex * v1, MeshVertex * v2)
{
    int IDs[3] = { v0->GetID(), v1->GetID(), v2->GetID() };
    return NewFaceDebug<VertexDataCatmark>(mesh, 3, IDs);
}

void MakeFaceUnoriented(Mesh * mesh, MeshVertex * v0, MeshVertex * v1, MeshVertex * v2)
{
    if (v0->GetEdge(v1) != NULL)
        NewFaceDebug(mesh, v1, v0, v2);
    else
        NewFaceDebug(mesh, v0, v1, v2);
}

void MakeFaceUnoriented(Mesh * mesh, MeshVertex * v0a, MeshVertex * v1a, MeshVertex * v0b, MeshVertex * v1b)
{
    MeshVertex * v[4] = { NULL, NULL, NULL, NULL };

    if (v0a->GetEdge(v1a) != NULL)
    {
        v[0] = v1a;
        v[1] = v0a;
    }
    else
    {
        v[0] = v0a;
        v[1] = v1a;
    }

    if (v1b->GetEdge(v0b) != NULL)
    {
        v[2] = v0b;
        v[3] = v1b;
    }
    else
    {
        v[2] = v1b;
        v[3] = v0b;
    }

    NewFaceDebug(mesh, v[0], v[1], v[2]);
    NewFaceDebug(mesh, v[0], v[2], v[3]);
}

ContourPoint * MakeFLPoint(MeshVertex * v0, MeshVertex *v1, Mesh * mesh,
                           const vec3 & cameraCenter,
                           std::map<std::pair<MeshVertex*,MeshVertex*>, ContourPoint *> & contourPoints)
{
    std::map<std::pair<MeshVertex*,MeshVertex*>, ContourPoint*>::iterator it =
            contourPoints.find(std::pair<MeshVertex*,MeshVertex*>(v0,v1));

    if (it == contourPoints.end())
        it = contourPoints.find(std::pair<MeshVertex*,MeshVertex*>(v1,v0));

    if (it != contourPoints.end())
        return (*it).second;


    // make the new vertex data.

    ContourPoint * flpt = new ContourPoint();

    flpt->vertex = mesh->NewVertex();

    ParamPointCC newPt;
    FindContour(v0->GetData().sourceLoc, v1->GetData().sourceLoc, cameraCenter, newPt);

    SetupVertex(flpt->vertex->GetData(), newPt, cameraCenter);

    flpt->vertex->GetData().facing = CONTOUR;
    //  flpt->loc = newPt;
    flpt->sourceEdge = v0->GetEdge(v1);

    contourPoints.insert(std::pair<std::pair<MeshVertex*,MeshVertex*>,ContourPoint*>
                         (std::pair<MeshVertex*,MeshVertex*>(v0,v1), flpt));

    return flpt;
}

double GetEdgeIndex(const MeshFace * face, const ContourPoint * v0)
{
    int ind0 = GetVertexIndex(face, v0->sourceEdge->GetOrgVertex());
    int ind1 = GetVertexIndex(face, v0->sourceEdge->GetDestVertex());

    if (ind0 == ind1 + 1 || ind1 == ind0 + 1)
        return std::min(ind0, ind1) + 0.5;
    else
        return std::max(ind0, ind1) + 0.5;
}

MeshEdge * MapEdge(NsdVVHalfedge* sourceEdge, MeshEdge * searchStart)
// find the edge of the mesh that corresponds to "sourceEdge", searching in the neighborhood of searchStart
{
    NsdVVVertex * v0 = sourceEdge->GetOrgVertex();
    NsdVVVertex * v1 = sourceEdge->GetDestVertex();

    for(int i=0;i<2;i++)
    {
        MeshFace * face = (i == 0 ? searchStart->GetLeftFace() : searchStart->GetRightFace() );

        if (face == NULL)
            continue;

        for(int e=0;e<4;e++)
        {
            MeshVertex * mv0 = face->GetVertex(e);
            MeshVertex * mv1 = face->GetVertex((e+1)%3);
            if ( (mv0->GetData().sourceLoc.SourceVertex() == v0 && mv1->GetData().sourceLoc.SourceVertex() == v1) ||
                 (mv1->GetData().sourceLoc.SourceVertex() == v0 && mv0->GetData().sourceLoc.SourceVertex() == v1))
            {
                MeshEdge * edge = mv0->GetEdge(mv1);
                if (edge == NULL)
                    edge = mv1->GetEdge(mv0);
                assert(edge != NULL);
                return edge;
            }
        }
    }

    assert(0);
    return NULL;
}

/*
bool ContourPointOrientation(const FlowLinePoint * v0, const FlowLinePoint * v1)
{
  assert(v0->sourceEdge != NULL && v1->sourceEdge != NULL);

  // find the common face they came from

  MeshFace * commonFace = NULL;
  if (v0->sourceEdge->GetLeftFace() != NULL && (v0->sourceEdge->GetLeftFace() == v1->sourceEdge->GetLeftFace() ||
                        v0->sourceEdge->GetLeftFace() == v1->sourceEdge->GetRightFace()))
    commonFace = v0->sourceEdge->GetLeftFace();
  else
    if (v0->sourceEdge->GetRightFace() != NULL && (v0->sourceEdge->GetRightFace() == v1->sourceEdge->GetLeftFace() ||
                           v0->sourceEdge->GetRightFace() == v1->sourceEdge->GetRightFace()))
      commonFace = v0->sourceEdge->GetRightFace();

  assert(commonFace != NULL);

  // use vertex indices to figure out the orientation of this edge w.r.t. the face
  int v0ind0 = GetVertexIndex(commonFace, v0->sourceEdge->GetOrgVertex());
  int v0ind1 = GetVertexIndex(commonFace, v0->sourceEdge->GetDestVertex());
  int v1ind0 = GetVertexIndex(commonFace, v1->sourceEdge->GetOrgVertex());
  int v1ind1 = GetVertexIndex(commonFace, v1->sourceEdge->GetDestVertex());
  
  int v0ind, v1ind;  // originating vertex index (i.e., (0-1) -> 0, (1-2) -> 1, (2-3) -> 2, (3-0) -> 3)
  // might be an better way to get this, but I don't know the details of the Hbr representation

  if (v0ind0 == v0ind1 + 1 || v0ind1 == v0ind0 + 1)
    v0ind = std::min(v0ind0, v0ind1);
  else
    v0ind = std::max(v0ind0, v0ind1);

  if (v1ind0 == v1ind1 + 1 || v1ind1 == v1ind0 + 1)
    v1ind = std::min(v1ind0, v1ind1);
  else
    v1ind = std::max(v1ind0, v1ind1);

  assert(v0ind != v1ind);

  return v0ind < v1ind;
}
*/

void MakeContourFaces(Mesh * mesh, ContourPoint * v0, ContourPoint * v1, FlowLinePoint *v0child, FlowLinePoint *v1child)
{
    if (v0 == NULL || v1 == NULL || v0child == NULL || v1child == NULL ||
            v0child->stitchTarget == NULL || v1child->stitchTarget == NULL)
        return;

    // check if the flowlines cross
    if (!ParamPointCC::ConvexInChart(v0->vertex->GetData().sourceLoc, v0child->vertex->GetData().sourceLoc, v1child->vertex->GetData().sourceLoc, v1->vertex->GetData().sourceLoc))
        //  if (!ParamPointCC::SameSide(v0->vertex->GetData().sourceLoc, v0child->vertex->GetData().sourceLoc,
        //			    v1->vertex->GetData().sourceLoc, v1child->vertex->GetData().sourceLoc) &&
        //      !ParamPointCC::SameSide(v1->vertex->GetData().sourceLoc, v1child->vertex->GetData().sourceLoc,
        //			    v0->vertex->GetData().sourceLoc, v0child->vertex->GetData().sourceLoc))
    {
        printf("WARNING: CROSSING FLOWLINES/NON-CONVEX FLOWLINE QUAD\n");
        return;
    }

    assert(v0->vertex->GetData().facing == CONTOUR && v1->vertex->GetData().facing == CONTOUR &&
           v0child->vertex->GetData().facing != CONTOUR && v1child->vertex->GetData().facing != CONTOUR);

    // find the common face that they came from
    MeshFace * commonFace = NULL;
    if (v0->sourceEdge->GetLeftFace() != NULL && (v0->sourceEdge->GetLeftFace() == v1->sourceEdge->GetLeftFace() ||
                                                  v0->sourceEdge->GetLeftFace() == v1->sourceEdge->GetRightFace()))
        commonFace = v0->sourceEdge->GetLeftFace();
    else
        if (v0->sourceEdge->GetRightFace() != NULL && (v0->sourceEdge->GetRightFace() == v1->sourceEdge->GetLeftFace() ||
                                                       v0->sourceEdge->GetRightFace() == v1->sourceEdge->GetRightFace()))
            commonFace = v0->sourceEdge->GetRightFace();

    assert(commonFace != NULL);

    double v0ind = GetEdgeIndex(commonFace, v0);
    double v1ind = GetEdgeIndex(commonFace, v1);
    int v0Cind = GetVertexIndex(commonFace, v0child->stitchTarget);
    int v1Cind = GetVertexIndex(commonFace, v1child->stitchTarget);

    for(int i=0;i<4;i++)
        printf("v[%d] = %s\n", i, FacingToString(commonFace->GetVertex(i)->GetData().facing));

    assert(v0ind != v1ind);
    assert(v0Cind >= 0 && v0Cind <= 3 && v1Cind >= 0 && v1Cind <= 3);

    if (v0ind == 3.5 && v0Cind == 0)
        v0Cind = 4;
    if (v1ind == 3.5 && v1Cind == 0)
        v1Cind = 4;


    printf("v0: %08X, v1: %08X, v0ind: %f, v1ind: %f, v0Cind: %d, v1Cind: %d\n",
           v0, v1, v0ind, v1ind, v0Cind, v1Cind);

    assert(v0child->vertex->GetEdge(v1child->vertex) == NULL &&
           v1child->vertex->GetEdge(v0child->vertex) == NULL);


    if (v0ind > v0Cind)
    {
        if ( v1ind < v1Cind)
        {
            printf("A: %08X, B: %08X\n", v0->vertex->GetEdge(v1->vertex), v1->vertex->GetEdge(v0->vertex));
            NewFaceDebug(mesh, v0->vertex, v1->vertex, v1child->vertex);
            NewFaceDebug(mesh, v0->vertex, v1child->vertex, v0child->vertex);

            //	  int vIDa[3] = { v0->vertex->GetID(), v1->vertex->GetID(), v1child->vertex->GetID() };
            //	  int vIDb[3] = { v0->vertex->GetID(), v1child->vertex->GetID(), v0child->vertex->GetID() };
            //	  NewFaceDebug(mesh, 3, vIDa);
            //	  NewFaceDebug(mesh, 3, vIDb);
            //	  newFaces.push_back(FaceRec(v0->vertex, v1->vertex, v1child->vertex));
            //	  newFaces.push_back(FaceRec(v0->vertex, v1child->vertex, v0child->vertex));
        }
        else
            printf("WARNING: SKIPPING MESSED-UP TRIANGLE\n");
    }
    else
    {
        if (v1ind > v1Cind)
        {
            printf("C: %08X, D: %08X\n", v0->vertex->GetEdge(v1->vertex), v1->vertex->GetEdge(v0->vertex));
            NewFaceDebug(mesh, v1->vertex, v0->vertex, v0child->vertex);
            NewFaceDebug(mesh, v1->vertex, v0child->vertex, v1child->vertex);

            //	  int vIDa[3] = { v1->vertex->GetID(), v0->vertex->GetID(), v0child->vertex->GetID() };
            //	  int vIDb[3] = { v1->vertex->GetID(), v0child->vertex->GetID(), v1child->vertex->GetID() };
            //	  NewFaceDebug(mesh, 3, vIDa);
            //	  NewFaceDebug(mesh, 3, vIDb);
            //	  newFaces.push_back(FaceRec(v1->vertex, v0->vertex, v0child->vertex));
            //	  newFaces.push_back(FaceRec(v1->vertex, v0child->vertex, v1child->vertex));
        }
        else
            printf("WARNING: SKIPPING MESSED-UP TRIANGLE\n");
    }
}


// input can be a quad mesh (triangles ok?), output is a triangle mesh
void FlowlineTesselate(Mesh * mesh, const vec3 & cameraCenter)
{
    std::map<std::pair<MeshVertex*,MeshVertex*>, ContourPoint* > contourPoints;
    std::vector<FlowLinePoint*> childPoints;
    std::multimap<MeshFace*,ContourPoint*> faceMap;

    std::set<MeshFace*> oldContourFaces;

    // each child point has a "stitchVertex" in the original mesh.  some children might share them
    std::map<MeshVertex*,FlowLineCluster*> stitchVertices;
    //  std::multimap<MeshVertex*, FlowLinePoint*> stitchToChildMap;

    // create contour vertices

    std::list<MeshFace*> inputFaces;
    mesh->GetFaces(inputFaces);

    std::vector<FaceRec> newFaces;

    // -------------------  create all the initial contour points --------------------

    for(std::list<MeshFace*>::iterator it = inputFaces.begin(); it != inputFaces.end(); ++it)
    {
        ContourPoint * p[4];
        MeshFace * face = *it;
        int numContourPoints = 0;
        int N = face->GetNumVertices();

        assert(N ==3 || N == 4);

        for(int e=0;e<N;e++)
        {
            MeshVertex * v0 = face->GetVertex(e);
            MeshVertex * v1 = face->GetVertex((e+1)%N);

            assert(v0->GetData().facing != CONTOUR && v1->GetData().facing != CONTOUR);

            if (v0->GetData().facing == v1->GetData().facing)
                continue;

            ContourPoint * flpt = MakeFLPoint(v0,v1,mesh,cameraCenter,contourPoints);
            faceMap.insert(std::pair<MeshFace*,ContourPoint*>(face, flpt));

            p[numContourPoints] = flpt;
            numContourPoints ++;
        }

        assert(numContourPoints == 0 || numContourPoints == 2 || numContourPoints == 4);

        // mark this as a contour face
        if (numContourPoints != 0)
            oldContourFaces.insert(face);

        if (numContourPoints == 4)
            printf("WARNING: QUAD FACE WITH TWO CONTOURS\n");
    }

    // ---------------------- connect contour siblings  -------------------

    for(std::map<std::pair<MeshVertex*,MeshVertex*>, ContourPoint* >::iterator it = contourPoints.begin();
        it != contourPoints.end(); ++it)
    {
        ContourPoint * flpt = (*it).second;
        MeshEdge * edge = flpt->sourceEdge;
        assert(edge != NULL);  // all contour points lie on edges

        MeshFace * adjFace[2] = { edge->GetLeftFace(), edge->GetRightFace() };

        // connect to siblings

        for(int i=0;i<2;i++)
            if (adjFace[i] != NULL)
            {
                // assuming only 2 points per face

                for(std::multimap<MeshFace*,ContourPoint*>::iterator fit = faceMap.lower_bound(adjFace[i]);
                    fit != faceMap.upper_bound(adjFace[i]); ++fit)
                    if ( (*fit).second != flpt)
                        flpt->AddSibling( (*fit).second);
            }
    }

    // ----------------------- create child points ----------------------------

    for(std::map<std::pair<MeshVertex*,MeshVertex*>, ContourPoint* >::iterator it = contourPoints.begin();
        it != contourPoints.end(); ++it)
    {
        ContourPoint * flpt = (*it).second;
        //      MeshEdge * edge = flpt->sourceEdge;
        //      assert(edge != NULL);  // all contour points lie on edges

        assert(flpt->nearChild == NULL && flpt->farChild == NULL);

        vec3 viewVec = cameraCenter - flpt->vertex->GetData().pos;

        // create two children
        // trace forward
        ParamRayCC nearRay = flpt->vertex->GetData().sourceLoc.VectorToParamRay(viewVec);
        ParamPointCC nearChildLoc = nearRay.Advance();

        if (!nearChildLoc.IsNull() && nearChildLoc.IsEvaluable())
        {
            //	  assert(nearChildLoc.IsValid());
            flpt->nearChild = new FlowLinePoint();
            flpt->nearChild->vertex = mesh->NewVertex();
            //	  flpt->nearChild->loc = nearChildLoc;
            SetupVertex(flpt->nearChild->vertex->GetData(), nearChildLoc, cameraCenter);
            flpt->nearChild->parent = flpt;
            flpt->nearChild->sourceEdgeVV = nearChildLoc.SourceEdge();

            childPoints.push_back(flpt->nearChild);

            assert(flpt->nearChild->vertex->GetData().facing != CONTOUR);
        }

        // do the same thing, tracing backward
        // trace backward
        ParamRayCC farRay = flpt->vertex->GetData().sourceLoc.VectorToParamRay(-viewVec);
        ParamPointCC farChildLoc = farRay.Advance();
        if (!farChildLoc.IsNull() && farChildLoc.IsEvaluable())
        {
            //	  assert(farChildLoc.IsValid());
            flpt->farChild = new FlowLinePoint();
            flpt->farChild->vertex = mesh->NewVertex();
            //	  flpt->farChild->loc = farChildLoc;
            SetupVertex(flpt->farChild->vertex->GetData(), farChildLoc, cameraCenter);
            flpt->farChild->parent = flpt;
            flpt->nearChild->sourceEdgeVV = nearChildLoc.SourceEdge();
            childPoints.push_back(flpt->farChild);

            assert(flpt->farChild->vertex->GetData().facing != CONTOUR);

            //	  flpt->farChild->sourceEdge = farChildLoc.SourceEdge();
        }
    }

    // ----------------------- connect children to their siblings --------------------

    for(std::vector<FlowLinePoint*>::iterator flit = childPoints.begin(); flit != childPoints.end(); ++flit)
    {
        FlowLinePoint * child = *flit;
        ContourPoint * parent = child->parent;
        for(int i=0;i<child->parent->NumSiblings();i++)
        {
            ContourPoint * parentSib = parent->siblings[i];
            FlowLinePoint * sib = (child == parent->nearChild ? parentSib->nearChild : parentSib->farChild);
            if (sib != NULL)
                child->AddSibling(sib);
        }
    }


    // ----------------- associate children with stitch vertices ------------------

    for(std::map<std::pair<MeshVertex*,MeshVertex*>, ContourPoint* >::iterator it = contourPoints.begin();
        it != contourPoints.end(); ++it)
    {
        ContourPoint * flpt = (*it).second;
        MeshEdge * edge = flpt->sourceEdge;
        assert(edge != NULL);  // all contour points lie on edges

        // check if we have a cusp

        // skip cusps and other weird cases for now
        if (flpt->nearChild == NULL || flpt->farChild == NULL ||
                flpt->nearChild->vertex->GetData().facing == flpt->farChild->vertex->GetData().facing)
            continue;

        // find the front-facing/back-facing adjecent vertices of the source mesh to use as stitch vertices
        assert(edge->GetOrgVertex()->GetData().facing != edge->GetDestVertex()->GetData().facing);

        MeshVertex * frontVertex = NULL;
        MeshVertex * backVertex = NULL;

        if (edge->GetOrgVertex()->GetData().facing == FRONT)
        {
            frontVertex = edge->GetOrgVertex();
            backVertex = edge->GetDestVertex();
        }
        else
        {
            frontVertex = edge->GetDestVertex();
            backVertex = edge->GetOrgVertex();
        }

        flpt->nearChild->stitchTarget = (flpt->nearChild->vertex->GetData().facing == FRONT ? frontVertex : backVertex);
        flpt->farChild->stitchTarget =  (flpt->farChild ->vertex->GetData().facing == FRONT ? frontVertex : backVertex);


        if (stitchVertices.find(flpt->nearChild->stitchTarget) == stitchVertices.end())
            stitchVertices[flpt->nearChild->stitchTarget] = new FlowLineCluster();

        FlowLineCluster * nearCluster = stitchVertices[flpt->nearChild->stitchTarget];
        nearCluster->stitchVertex = flpt->nearChild->stitchTarget;
        nearCluster->parents.insert(flpt->nearChild);

        if (stitchVertices.find(flpt->farChild->stitchTarget) == stitchVertices.end())
            stitchVertices[flpt->farChild->stitchTarget] = new FlowLineCluster();

        FlowLineCluster * farCluster = stitchVertices[flpt->farChild->stitchTarget];
        farCluster->stitchVertex = flpt->farChild->stitchTarget;
        farCluster->parents.insert(flpt->farChild);

        //      stitchToChildMap.insert(std::pair<MeshVertex*, FlowLinePoint*>( flpt->nearChild->stitchTarget, flpt->nearChild));
        //      stitchToChildMap.insert(std::pair<MeshVertex*, FlowLinePoint*>( flpt->farChild->stitchTarget, flpt->farChild));
    }


    // ----------------------- generate contour faces -----------------------------

    printf("Num contour points = %d\n", (int)contourPoints.size());

    for(std::map<std::pair<MeshVertex*,MeshVertex*>, ContourPoint*>::iterator it = contourPoints.begin();
        it != contourPoints.end(); ++it)
    {
        ContourPoint * flpt = (*it).second;
        for(int i =0; i<flpt->NumSiblings(); i++)
        {
            if (flpt->nearChild != NULL && flpt->siblings[i]->nearChild != NULL &&
                    flpt->nearChild->vertex->GetEdge(flpt->siblings[i]->nearChild->vertex) == NULL &&
                    flpt->siblings[i]->nearChild->vertex->GetEdge(flpt->nearChild->vertex) == NULL) // not already created
                MakeContourFaces(mesh, flpt, flpt->siblings[i], flpt->nearChild, flpt->siblings[i]->nearChild);

            if (flpt->farChild != NULL && flpt->siblings[i]->farChild != NULL &&
                    flpt->farChild->vertex->GetEdge(flpt->siblings[i]->farChild->vertex) == NULL &&
                    flpt->siblings[i]->farChild->vertex->GetEdge(flpt->farChild->vertex) == NULL)
                MakeContourFaces(mesh, flpt, flpt->siblings[i],flpt->farChild, flpt->siblings[i]->farChild);
        }
    }


    //  -------------------- find the existing mesh vertices to be stitched to this cluster -------

    for(std::map<MeshVertex*,FlowLineCluster*>::iterator it = stitchVertices.begin(); it != stitchVertices.end(); ++it)
    {
        MeshVertex * stitchVertex = (*it).first;
        FlowLineCluster * cluster = (*it).second;

        std::set<MeshFace*> oneRingFaces;
        GetOneRing(stitchVertex, oneRingFaces);

        for(std::set<MeshFace*>::iterator fit = oneRingFaces.begin(); fit != oneRingFaces.end(); ++fit)
            for(int i=0;i< (*fit)->GetNumVertices();i++)
            {
                MeshVertex * v = (*fit)->GetVertex(i);

                if (v->GetData().facing == stitchVertex->GetData().facing && stitchVertices.find(v) == stitchVertices.end())
                    cluster->existingPoints.insert(v);
            }
    }

    // ---------------------- linearize parents of flowline clusters ------------

    for(std::map<MeshVertex*,FlowLineCluster*>::iterator it = stitchVertices.begin(); it != stitchVertices.end(); ++it)
    {
        FlowLineCluster * cluster = (*it).second;
        std::set<FlowLinePoint*> remainingParents = cluster->parents; // copy the set of parents

        assert (remainingParents.size() > 0);
        FlowLinePoint * start = *(remainingParents.begin());
        remainingParents.erase(remainingParents.begin());
        cluster->parentsOrdered.push_back(start);

        for(int i=0;i<start->NumSiblings();i++)
        {
            FlowLinePoint * next = start->siblings[i];
            FlowLinePoint * last = start;
            while (remainingParents.find(next) != remainingParents.end())
            {
                remainingParents.erase(remainingParents.find(next));
                if (i == 0)
                    cluster->parentsOrdered.push_back(next);
                else
                    cluster->parentsOrdered.push_front(next);
                if (next->NumSiblings() > 1 && next->siblings[0] != last)
                    next = next->siblings[0];
                else
                    if (next->NumSiblings() > 2 && next->siblings[1] != last)
                        next = next->siblings[1];
                    else
                        break;
            }
        }

        if (remainingParents.size() != 0)
            printf("CLUSTER WITH DISCONNECTED PARENTS\n");
        else
            assert(cluster->parents.size() == cluster->parentsOrdered.size());
    }


    // ---------------------- connect flowline clusters to siblings -------------

    for(std::map<MeshVertex*,FlowLineCluster*>::iterator it = stitchVertices.begin(); it != stitchVertices.end(); ++it)
    {
        FlowLineCluster * cluster = (*it).second;

        // skip clusters with disconnected parents
        if (cluster->parents.size() != cluster->parentsOrdered.size())
            continue;

        for(std::set<FlowLinePoint*>::iterator pit = cluster->parents.begin(); pit != cluster->parents.end(); ++pit)
            for(int i=0;i<(*pit)->NumSiblings();++i)
            {
                FlowLinePoint * sib = (*pit)->siblings[i];
                if ( sib->stitchTarget != NULL && cluster->parents.find(sib) == cluster->parents.end() &&
                     !cluster->HasSibling(stitchVertices[sib->stitchTarget]))
                    cluster->AddSibling(stitchVertices[sib->stitchTarget], *pit);
            }
    }


    // ----------------------- delete old contour/stitch faces ------------------------
    // note: needed to keep the old faces around long enough for MakeContourFaces to use orientation and to mark the stitch faces

    std::set<MeshFace*> oldStitchFaces;

    for(std::set<MeshFace*>::iterator it = oldContourFaces.begin(); it != oldContourFaces.end(); ++it)
        for(int e=0;e<(*it)->GetNumVertices();e++)
        {
            MeshFace * left = (*it)->GetEdge(e)->GetLeftFace();
            MeshFace * right = (*it)->GetEdge(e)->GetRightFace();

            if (left != (*it) && oldContourFaces.find(left) == oldContourFaces.end())
                oldStitchFaces.insert(left);

            if (right != (*it) && oldContourFaces.find(right) == oldContourFaces.end())
                oldStitchFaces.insert(right);
        }

    for(std::set<MeshFace*>::iterator it = oldContourFaces.begin(); it != oldContourFaces.end(); ++it)
        mesh->DeleteFace(*it);

    for(std::set<MeshFace*>::iterator fit = oldStitchFaces.begin(); fit != oldStitchFaces.end(); ++fit)
        if ( (*fit) != NULL)
            mesh->DeleteFace(*fit);


    // ---------------------- create connecting faces between flowline clusters ------

    /*  This part crashes
  printf("Creating cluster connecting faces\n");

  std::set<std::pair<FlowLineCluster*,FlowLineCluster*> > visitedPairs;


  for(std::map<MeshVertex*,FlowLineCluster*>::iterator it = stitchVertices.begin(); it != stitchVertices.end(); ++it)
    {
      MeshVertex * stitchVertex = (*it).first;
      FlowLineCluster * cluster = (*it).second;
      for(int i=0;i<cluster->NumSiblings(); i++)
    {
      FlowLineCluster * sibling = cluster->siblings[i];
      if (!sibling->HasSibling(cluster) ||
          visitedPairs.find(std::pair<FlowLineCluster*,FlowLineCluster*>(sibling,cluster)) != visitedPairs.end())
        continue;

      visitedPairs.insert(std::pair<FlowLineCluster*,FlowLineCluster*>(cluster,sibling));

      int oppIndex = (sibling->siblings[0] == cluster ? 0 : 1);

      vector<MeshVertex*> intersection(cluster->existingPoints.size());
      vector<MeshVertex*>::iterator vend;

      // find one or two existing points that these clusters share using a set intersection
      vend = std::set_intersection(cluster->existingPoints.begin(), cluster->existingPoints.end(),
                       sibling->existingPoints.begin(), sibling->existingPoints.end(), intersection.begin());

      // there should be 1 or 2 intersection points
      int intSize = int(vend - intersection.begin());

      if (intSize == 0)  // not sure this can happen; maybe at mesh boundaries or places where tracing went bad
        continue;

      //	  printf("intSize = %d\n", intSize);

      assert(sibling->endpoints[oppIndex]->vertex != cluster->endpoints[i]->vertex);

      if (intSize == 1)
        MakeFaceUnoriented(mesh, cluster->endpoints[i]->vertex, sibling->endpoints[oppIndex]->vertex, intersection[0]);
      else
        {
          assert(intSize == 2);

          MakeFaceUnoriented(mesh, cluster->endpoints[i]->vertex, sibling->endpoints[oppIndex]->vertex, intersection[0], intersection[1]);
        }
    }
    }





  // ----------------------- connect clusters to their own children ("existingPoints") ------------------------------

  for(std::map<MeshVertex*,FlowLineCluster*>::iterator it = stitchVertices.begin(); it != stitchVertices.end(); ++it)
    {
      FlowLineCluster * cluster = (*it).second;
      
      if (cluster->existingPoints.size() == 0) // huh?
    continue;

      FlowLinePoint* parent = *(cluster->parentsOrdered.begin());
      
      // make a list of children

      std::list<MeshVertex*> existingOrdered;

      // find a vertex adjacent to the first parent, but along a hole in the mesh. there should be only one...
      MeshVertex * child = NULL;
      std::list<MeshVertex*> oneRing;
      parent->vertex->GetSurroundingVertices(oneRing);

      for(std::list<MeshVertex*>::iterator it = oneRing.begin(); it != oneRing.end(); ++it)
    if ( ((*it)->GetEdge(parent->vertex)==NULL) != (parent->vertex->GetEdge(*it)==NULL) &&
         cluster->existingPoints.find(*it) != cluster->existingPoints.end())
      {
        child = *it;
        break;
      }
      
      while (child != NULL)
    {
      existingOrdered.push_back(child);
      cluster->existingPoints.erase(cluster->existingPoints.find(child));

      MeshVertex * nextChild = NULL;
      oneRing.clear();
      child->GetSurroundingVertices(oneRing);
      for(std::list<MeshVertex*>::iterator it = oneRing.begin(); it != oneRing.end();++it)
        if ( ((*it)->GetEdge(child)==NULL) != (child->GetEdge(*it)==NULL) &&
         cluster->existingPoints.find(*it) != cluster->existingPoints.end())
          {
        nextChild = *it;
        break;
          }
      child = nextChild;
    }

      // connect up pairs of child and parent into quads

      std::list<FlowLinePoint*>::iterator parentIt = cluster->parentsOrdered.begin();
      std::list<FlowLinePoint*>::iterator nextParentIt;
      std::list<MeshVertex*>::iterator childIt = existingOrdered.begin();
      std::list<MeshVertex*>::iterator nextChildIt;

      nextParentIt = parentIt;
      nextParentIt++;
      nextChildIt = childIt;
      nextChildIt++;

      while (nextParentIt != cluster->parentsOrdered.end() && nextChildIt != existingOrdered.end())
    {
      MakeFaceUnoriented(mesh, (*parentIt)->vertex, (*nextParentIt)->vertex, *childIt, *nextChildIt);
      parentIt ++;
      nextParentIt++;
      childIt ++;
      nextChildIt++;
    }

      // make triangles from any leftovers

      while(nextParentIt != cluster->parentsOrdered.end())
    {
      MakeFaceUnoriented(mesh, (*parentIt)->vertex, (*nextParentIt)->vertex, *childIt);
      parentIt ++;
      nextParentIt++;
    }

      while(nextChildIt != existingOrdered.end())
    {
      MakeFaceUnoriented(mesh, *childIt, *nextChildIt, (*parentIt)->vertex);
      childIt ++;
      nextChildIt++;
    }
    }

  // ----------------------- connect gaps between stitch geometry and original ---------------------

  // TBD
  */



    // ----------------------- convert all remaining quads to triangles ----------------------

    printf("Converting quads to triangles\n");

    std::list<MeshFace*> faces;
    mesh->GetFaces(faces);

    for(std::list<MeshFace*>::iterator it = faces.begin(); it != faces.end(); ++it)
        if ( (*it)->GetNumVertices() == 4)
        {
            MeshFace * face = *it;
            int vID1[3] = { face->GetVertex(0)->GetID(), face->GetVertex(1)->GetID(),  face->GetVertex(2)->GetID() };
            int vID2[3] = { face->GetVertex(0)->GetID(), face->GetVertex(2)->GetID(),  face->GetVertex(3)->GetID() };

            mesh->DeleteFace(face);
            NewFaceDebug(mesh, 3, vID1);
            NewFaceDebug(mesh, 3, vID2);
        }

    // ----------------------- create all new faces ----------------------

    printf("Adding new faces\n");

    for(std::vector<FaceRec>::iterator it = newFaces.begin(); it != newFaces.end(); ++it)
    {
        int IDs[3] = { (*it).v[0]->GetID(), (*it).v[1]->GetID(), (*it).v[2]->GetID() };
        NewFaceDebug(mesh, 3, IDs);
    }

}
