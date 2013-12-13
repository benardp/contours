// Copyright (C) : Please refer to the COPYRIGHT file distributed with
//this source distribution.
// This program is free software; you can redistribute it and/or
//modify it under the terms of the GNU General Public License as
//published by the Free Software Foundation; either version 2 of the
//License, or (at your option) any later version.
// This program is distributed in the hope that it will be useful, but
//WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//General Public License for more details.
// You should have received a copy of the GNU General Public License
//along with this program; if not, write to the Free Software
//Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
//02111-1307, USA.
//
///////////////////////////////////////////////////////////////////////////////

//#define DEBUG_INTERSECTION



#include <algorithm>
#include "ViewMapBuilder.h"
#include "../geometry/FastGrid.h"  // included as a workaround
#include "../scene_graph/NodeGroup.h"
#include "../scene_graph/NodeShape.h"
#include "../scene_graph/VertexRep.h"
#include "../scene_graph/LineRep.h"

using namespace std;

static const unsigned gProgressBarMaxSteps = 10;
static const unsigned gProgressBarMinSize = 2000;

bool orientableSurfaces; // assume that n.v < 0 means backfacing? 



bool badIntersection; // was any bad intersection detected?

#ifdef DEBUG_INTERSECTION
vector<TriangleRep*> debugTriangles;
//void debugFES(FEdgeIntersection * fes);
#endif



bool USE_SHEWCHUCK_TESTS = false;
bool NEW_SILHOUETTE_HEURISTIC = false;

extern "C"
{
int tri_tri_intersection_test_3d(real p1[3], real q1[3], real r1[3],
real p2[3], real q2[3], real r2[3],
int * coplanar,
real source[3],real target[3]);
}




// visualizations for debugging

NodeGroup * visDebugNode = NULL;

bool hasEdge(ViewVertex *vv, ViewEdge *ve);

/*
void addDebugPoint(Vec3r point, NodeShape * node)
{
  const int debugPointSize = 10;
  
  VertexRep * vert1 = new VertexRep(point.x(), point.y(), point.z());
  vert1->SetPointSize(debugPointSize);
  vert1->ComputeBBox();
  node->AddRep(vert1);
  }*/

#ifdef DEBUG_INTERSECTION
void makeDebugTriangles(WFace * face1, WFace * face2)
{
    static real rand = 0;
    rand = 1-rand;
    printf("rand = %f\n", rand);
    Vec3r color1(1,0,rand);
    Vec3r color2(0,1,rand);

    TriangleRep * tri1 = new TriangleRep(face1->GetVertex(0)->GetVertex(),color1,
                                         face1->GetVertex(1)->GetVertex(),color1,
                                         face1->GetVertex(2)->GetVertex(),color1);
    debugTriangles.push_back(tri1);
    TriangleRep * tri2 = new TriangleRep(face2->GetVertex(0)->GetVertex(),color2,
                                         face2->GetVertex(1)->GetVertex(),color2,
                                         face2->GetVertex(2)->GetVertex(),color2);
    debugTriangles.push_back(tri2);
}
#endif


real ArcLength2D(ViewEdge * ve, real maxLength)
{
    real arcLength = 0;

    FEdge * fe = ve->fedgeA();

    do
    {
        arcLength += fe->getLength2D();
        fe = fe->nextEdge();
    } while (fe != NULL && fe != ve->fedgeA() && arcLength < maxLength);

    return arcLength;
}

ViewMap* ViewMapBuilder::BuildViewMap(WingedEdge& we, visibility_algo iAlgo, real epsilon) {
    _ViewMap = new ViewMap;
    _currentId = 1;
    _currentFId = 0;
    _currentSVertexId = 0;

    exactinit();  // initialize Shewchuk's orientation code
    visDebugNode = new NodeGroup;

    _ViewMap->checkPointers("init");

    // Compute Punch-Out Regions
    if (iAlgo == punch_out)
        ComputePunchOutRegions(we, _Grid);

    _ViewMap->checkPointers("after ComputePunchOutRegions");

    // -------- computing intersections -------

    // compute surface intersection edges
    computeSelfIntersections(we);

    _ViewMap->checkPointers("after computeSelfIntersections");

    // ---------- building view edges ----------

    // Builds initial view edges (except for self-intersections)
    computeInitialViewEdges(we);

    _ViewMap->checkPointers("after computeInitialViewEdges");

    // ---------- additional intersections of SIs and SILs -----

    if (iAlgo == punch_out)
    {
        // handle SI viewedges and the intersections they have entering and inside punchout/cusp regions
        ComputePunchOutIntersections(_ViewMap);

        _ViewMap->checkPointers("after ComputePunchOutIntersections");
    }

    // ---------- edge splitting --------------

    // Detects cusps
    computeCusps(_ViewMap);

    _ViewMap->checkPointers("after computeCusps");

    // Compute remaining image-space curve intersections
    ComputeCurveIntersections(_ViewMap, iAlgo, epsilon);

    _ViewMap->checkPointers("after ComputeIntersections");

    // ---------- visibility -------------------

    // Compute visibility
    ComputeEdgesVisibility(_ViewMap, we, iAlgo, _Grid, epsilon);

    PropagateVisibilty(_ViewMap);

    printf("Check coherence of visibility\n");
    for(vector<ViewVertex*>::iterator vit = _ViewMap->ViewVertices().begin(); vit != _ViewMap->ViewVertices().end(); ++vit)
    {
        ViewVertex * vVertex = *vit;
        TVertex* tVertex = dynamic_cast<TVertex*>(vVertex);
        if(tVertex!=NULL && tVertex->backEdgeA().first && tVertex->backEdgeB().first &&
                tVertex->frontEdgeA().first && tVertex->frontEdgeB().first)
        {
            // Check if visibility at T-junctions involving CUSPs is correct (>=2)
            // If not, make the ViewEdge connected to the front CUSP visible

            int visibleFont = 0, visibleBack = 0;;
            if(tVertex->backEdgeA().first->qi() == 0)
                visibleBack++;
            if(tVertex->frontEdgeA().first->qi() == 0)
                visibleFont++;
            if(tVertex->backEdgeB().first->qi() == 0)
                visibleBack++;
            if(tVertex->frontEdgeB().first->qi() == 0)
                visibleFont++;

            if((visibleFont + visibleBack)==1){
                printf("INCOHERENT VISIBILITY\n");
                char str[200];

                ViewEdge* mate, *edge;
                NonTVertex* ntVertex = NULL;
                ViewEdge* nextE = NULL;
                ViewVertex* nextV = NULL;
                if(visibleFont==1){
                    edge = tVertex->backEdgeB().first;
                    if(edge->B()->getNature() & Nature::CUSP){
                        ntVertex = dynamic_cast<NonTVertex*>(edge->B());
                        assert(ntVertex->viewedges().size() == 2);
                        mate = (ntVertex->viewedges()[0].first != edge ? ntVertex->viewedges()[0].first : ntVertex->viewedges()[1].first);
                        if(mate->qi() == 0){
                            sprintf(str, "FIX 1\n");
                        }else{
                            ntVertex = NULL;
                        }
                    }else if(edge->B()->getNature() & Nature::T_VERTEX){
                        TVertex* tv = dynamic_cast<TVertex*>(edge->B());
                        if(tv->numEdges()<4)
                            continue;
                        if(tv->backEdgeA().first == edge){
                            nextE = tv->backEdgeB().first;
                            nextV = nextE->B();
                        }else if(tv->backEdgeB().first == edge){
                            nextE = tv->backEdgeA().first;
                            nextV = nextE->A();
                        }else if(tv->frontEdgeB().first == edge){
                            nextE = tv->frontEdgeA().first;
                            nextV = nextE->A();
                        }else if(tv->frontEdgeA().first == edge){
                            nextE = tv->frontEdgeB().first;
                            nextV = nextE->B();
                        }
                        if(nextV->getNature() & Nature::CUSP){
                            ntVertex = dynamic_cast<NonTVertex*>(nextV);
                            assert(ntVertex->viewedges().size() == 2);
                            mate = (ntVertex->viewedges()[0].first != edge ? ntVertex->viewedges()[0].first : ntVertex->viewedges()[1].first);
                            if(mate->qi() == 0){
                                sprintf(str, "FIX 1.1\n");
                            }else{
                                ntVertex = NULL;
                                nextE = NULL;
                            }
                        }
                    }
                    if(!ntVertex){
                        edge = tVertex->frontEdgeA().first;
                        if(edge->A()->getNature() & Nature::CUSP){
                            ntVertex = dynamic_cast<NonTVertex*>(edge->A());
                            assert(ntVertex->viewedges().size() == 2);
                            mate = (ntVertex->viewedges()[0].first != edge ? ntVertex->viewedges()[0].first : ntVertex->viewedges()[1].first);
                            if(mate->qi() == 0){
                                sprintf(str, "FIX 2\n");
                            }else{
                                ntVertex = NULL;
                            }
                        }else if(edge->A()->getNature() & Nature::T_VERTEX){
                            TVertex* tv = dynamic_cast<TVertex*>(edge->A());
                            if(tv->numEdges()<4)
                                continue;
                            if(tv->backEdgeA().first == edge){
                                nextE = tv->backEdgeB().first;
                                nextV = nextE->B();
                            }else if(tv->backEdgeB().first == edge){
                                nextE = tv->backEdgeA().first;
                                nextV = nextE->A();
                            }else if(tv->frontEdgeB().first == edge){
                                nextE = tv->frontEdgeA().first;
                                nextV = nextE->A();
                            }else if(tv->frontEdgeA().first == edge){
                                nextE = tv->frontEdgeB().first;
                                nextV = nextE->B();
                            }
                            if(nextV->getNature() & Nature::CUSP){
                                ntVertex = dynamic_cast<NonTVertex*>(nextV);
                                assert(ntVertex->viewedges().size() == 2);
                                mate = (ntVertex->viewedges()[0].first != edge ? ntVertex->viewedges()[0].first : ntVertex->viewedges()[1].first);
                                if(mate->qi() == 0){
                                    sprintf(str, "FIX 2.1\n");
                                }else{
                                    ntVertex = NULL;
                                    nextE = NULL;
                                }
                            }
                        }
                    }
                }else if(visibleBack==1){
                    edge = tVertex->frontEdgeB().first;
                    if(edge->B()->getNature() & Nature::CUSP){
                        ntVertex = dynamic_cast<NonTVertex*>(edge->B());
                        assert(ntVertex->viewedges().size() == 2);
                        mate = (ntVertex->viewedges()[0].first != edge ? ntVertex->viewedges()[0].first : ntVertex->viewedges()[1].first);
                        if(mate->qi() == 0){
                            sprintf(str, "FIX 3\n");
                        }else{
                            ntVertex = NULL;
                        }
                    }else if(edge->B()->getNature() & Nature::T_VERTEX){
                        TVertex* tv = dynamic_cast<TVertex*>(edge->B());
                        if(tv->numEdges()<4)
                            continue;
                        if(tv->backEdgeA().first == edge){
                            nextE = tv->backEdgeB().first;
                            nextV = nextE->B();
                        }else if(tv->backEdgeB().first == edge){
                            nextE = tv->backEdgeA().first;
                            nextV = nextE->A();
                        }else if(tv->frontEdgeB().first == edge){
                            nextE = tv->frontEdgeA().first;
                            nextV = nextE->A();
                        }else if(tv->frontEdgeA().first == edge){
                            nextE = tv->frontEdgeB().first;
                            nextV = nextE->B();
                        }
                        if(nextV->getNature() & Nature::CUSP){
                            ntVertex = dynamic_cast<NonTVertex*>(nextV);
                            assert(ntVertex->viewedges().size() == 2);
                            mate = (ntVertex->viewedges()[0].first != edge ? ntVertex->viewedges()[0].first : ntVertex->viewedges()[1].first);
                            if(mate->qi() == 0){
                                sprintf(str, "FIX 3.1\n");
                            }else{
                                ntVertex = NULL;
                                nextE = NULL;
                            }
                        }
                    }
                    if(!ntVertex){
                        edge = tVertex->frontEdgeA().first;
                        if(edge->A()->getNature() & Nature::CUSP){
                            ntVertex = dynamic_cast<NonTVertex*>(edge->A());
                            assert(ntVertex->viewedges().size() == 2);
                            mate = (ntVertex->viewedges()[0].first != edge ? ntVertex->viewedges()[0].first : ntVertex->viewedges()[1].first);
                            if(mate->qi() == 0){
                                sprintf(str, "FIX 4\n");
                            }else{
                                ntVertex = NULL;
                            }
                        }else if(edge->A()->getNature() & Nature::T_VERTEX){
                            TVertex* tv = dynamic_cast<TVertex*>(edge->A());
                            if(tv->numEdges()<4)
                                continue;
                            if(tv->backEdgeA().first == edge){
                                nextE = tv->backEdgeB().first;
                                nextV = nextE->B();
                            }else if(tv->backEdgeB().first == edge){
                                nextE = tv->backEdgeA().first;
                                nextV = nextE->A();
                            }else if(tv->frontEdgeB().first == edge){
                                nextE = tv->frontEdgeA().first;
                                nextV = nextE->A();
                            }else if(tv->frontEdgeA().first == edge){
                                nextE = tv->frontEdgeB().first;
                                nextV = nextE->B();
                            }
                            if(nextV->getNature() & Nature::CUSP){
                                ntVertex = dynamic_cast<NonTVertex*>(nextV);
                                assert(ntVertex->viewedges().size() == 2);
                                mate = (ntVertex->viewedges()[0].first != edge ? ntVertex->viewedges()[0].first : ntVertex->viewedges()[1].first);
                                if(mate->qi() == 0){
                                    sprintf(str, "FIX 1.4\n");
                                }else{
                                    ntVertex = NULL;
                                    nextE = NULL;
                                }
                            }
                        }
                    }
                }
                if(ntVertex){
                    assert(ntVertex->viewedges().size() == 2);
                    assert(mate->qi() == 0);
                    real arcLength = ArcLength2D(edge,FLT_MAX);
                    if(arcLength<=2.0){
                        printf("%s",str);
                        edge->SetQI(0);
                        edge->MarkInconsistent(true);
                    }
                    if(nextE != NULL){
                        nextE->SetQI(0);
                        nextE->MarkInconsistent(true);
                    }
                }
            }
        }else{
            // If 2 CUSPs are directly (no image space intersection) connected by an invisible edge with arclength<=1px, change it to visible
            NonTVertex* ntVertex = dynamic_cast<NonTVertex*>(vVertex);
            if(ntVertex && ntVertex->getNature() & Nature::CUSP){
                assert(ntVertex->viewedges().size()==2);
                for(int i=0; i<2; i++){
                    ViewEdge* edge = ntVertex->viewedges()[i].first;
                    ViewVertex* otherVVertex = (edge->A() != vVertex) ? edge->A() : edge->B();
                    NonTVertex* otherNTVertex = dynamic_cast<NonTVertex*>(otherVVertex);
                    if(otherNTVertex != NULL && otherNTVertex->getNature() & Nature::CUSP){
                        assert(otherNTVertex->viewedges().size()==2);

                        real arcLength = ArcLength2D(edge,FLT_MAX);
                        if(arcLength<=1.0){
                            ViewEdge* otherEdge = (otherNTVertex->viewedges()[0].first != edge ? otherNTVertex->viewedges()[0].first : otherNTVertex->viewedges()[1].first);
                            if(edge->qi()!=0 && ntVertex->viewedges()[(i+1)%2].first->qi()==0 && otherEdge->qi()==0){
                                edge->SetQI(0);
                                edge->MarkInconsistent();
                            }
                        }
                    }
                }
            }
        }
    }

    /*
 if (_graftThreshold > 0)
  {
      printf("Grafting dead-ends\n");
      GraftDeadEnds(_ViewMap);
  }
*/

    if (_cuspTrimThreshold > 0)
    {
        printf("Hiding small loops and dead ends\n");
        bool changed1 = false;
        bool changed2 = false;
        do
        {
            //changed1 = HideDeadEnds(_ViewMap);
            changed1 = HideSmallBits(_ViewMap);
            changed2 = HideSmallLoopsAndDeadEnds(_ViewMap);
        } while (changed1 || changed2);
    }
    /*
  if (_graftThreshold > 0)
  {
      printf("Grafting dead-ends\n");
      GraftDeadEnds(_ViewMap);
  }
*/
    // wiggle all svertices for debugging

    /*
  for(vector<SVertex*>::iterator it = _ViewMap->SVertices().begin(); it != _ViewMap->SVertices().end(); ++it)
    {
      Vec3r p = (*it)->getPoint3D();

      for(int j=0;j<3;j++)
    p[j]+= drand48()/10;
      (*it)->SetPoint3D(p);
    }
  */

    _ViewMap->checkPointers("after ComputeEdgesVisibility",true);

    return _ViewMap;
}

struct TriPair
        // data structure for storing the intersection between two triangles:
        //    - face1 and face2 are the two triangles
        //    Let A and B be the endpoints of the intersection line segment.
        //      - ptA and ptB are the 3D coordinates
        //      Each of these endpoints must lie on an edge of one triangle
        //      - trinumA \in {0,1} and edgeNumA \in {0,1,2} indicates which edge on which triangle contains point A.
        //        same for trinumB and edgeNumB.   trinum = 0 means face1, 1->face2.  (not great notation there).
{
    WFace * face1, * face2;
    Vec3r ptA, ptB;
    int triNumA, triNumB;
    int edgeNumA, edgeNumB;
    
    TriPair();
    TriPair(const WFace * f1, const WFace * f2);
    bool operator==(const TriPair & tp) const;
    void forward();
    void backward();
    WEdge * getEdge(bool A);
    //  SVertex * getPOendpoint(ViewMap*vm, bool fromA);
};


/*
// still using this?
SVertex * TriPair::getPOendpoint(ViewMap*vm,bool fromA)
{
  int triNum = fromA ? triNumA : triNumB;
  int edgeNum = fromA ? edgeNumA : edgeNumB;
  WXFace * face = (WXFace*)(triNum == 0 ? face1 : face2);
  POBoundaryEdge * po = face->sourcePOB();
  Vec3r pt = fromA? ptA : ptB;
  //  WFace * oppFace = face->GetBordingFace(edgeNum);  // this ought to be NULL

  printf("GET_PO_ENDPOINT, fromA: %s\n",fromA?"true":"false");
  printf("edgenum: %d, trinum: %d, sourcePOB: %08X\n", edgeNum, triNum,face->sourcePOB());

  if (edgeNum == -1)
    return NULL;

  if (po == NULL)
    return NULL;

  // find the mesh edge that the source silhouette point came from
  assert(po->sourceType == POBoundaryEdge::MESH_SIL);
  assert(po->srcFaceEdge != -1);

  // figure out whether the po's A or B corresponds to the endpoint
  bool fromPOA = GeomUtils::distPointSegment<Vec3r>(pt, po->A, po->Asrc) <
    GeomUtils::distPointSegment<Vec3r>(pt, po->B, po->Bsrc);

  // check if the source is a silhouette point or not
  if (( fromPOA && (po->AsrcVert != NULL || po->isAdegen || po->isAclipPoint || po->Aedge != -1)) ||
      (!fromPOA && (po->BsrcVert != NULL || po->isBdegen || po->isBclipPoint || po->Bedge != -1)))
    return NULL;

  printf("dA: %f, dB: %f\n", GeomUtils::distPointSegment<Vec3r>(pt, po->A, po->Asrc),
     GeomUtils::distPointSegment<Vec3r>(pt, po->B, po->Bsrc));
  printf("fromPOA: %s\n", fromPOA?"true":"false");
  po->print();

  // now, we need to find the silhouette svertex that lies on edge "edgeNum" of face "face"
  // this is a very inefficient search, might need to replace it with a map or pointers if it's too slow

  WFace * srcFace = po->sourceFace;
  WFace * oppFace = srcFace->GetBordingFace(po->srcFaceEdge);
  
  assert(oppFace != NULL);

  Vec3r srcPt = fromPOA ? po->Asrc : po->Bsrc;

  for(vector<SVertex*>::iterator sit = vm->SVertices().begin(); sit != vm->SVertices().end(); ++sit)
    {
      SVertex * sv = *sit;

      if ((sv->getPoint3D() - srcPt).norm() < 10)
    printf("svDist = %f\n",(sv->getPoint3D() - srcPt).norm());

      if (sv->fedges().size() != 2)
    continue;
      FEdge *fe1 = sv->fedges()[0];
      FEdge *fe2 = sv->fedges()[1];

      if (!(fe1->getNature() & Nature::SILHOUETTE) ||
      !(fe2->getNature() & Nature::SILHOUETTE))
    continue;

      FEdgeSmooth * fes1=dynamic_cast<FEdgeSmooth*>(fe1);
      FEdgeSmooth * fes2=dynamic_cast<FEdgeSmooth*>(fe2);

      assert(fes1 != NULL && fes2 != NULL);

      if ( (fes1->face() == srcFace && fes2->face() == oppFace) ||
       (fes1->face() == oppFace && fes2->face() == srcFace))
    return sv;
    }
  assert(0);
}
*/

TriPair::TriPair()
{
    face1 = face2 = NULL;
    ptA = ptB = Vec3r(-1,-1,-1);
    triNumA = triNumB = edgeNumA = edgeNumB = -1;
}

/*
TriPair::TriPair(const WFace *f1, const WFace *f2)
{
  face1 = f1;
  face2 = f2;
  ptA = ptB = Vec3r(-1,-1,-1);
  triNumA = triNumB = edgeNumA = edgeNumB = -1;
}

bool TriPair::operator==(const TriPair & tp) const
{
  return (tp.face1 == face1 && tp.face2 == face2) ||
    (tp.face2 == face1 && tp.face1 == face2);
}
*/

// advance to the next triangle-triangle pair that shares intersection point B
// all other elements of the data structure become uninitialized
void TriPair::forward()
{
    if (triNumB == 0)
        face1 = face1->GetBordingFace(edgeNumB);
    else
    {
        assert(triNumB == 1);

        face2 = face2->GetBordingFace(edgeNumB);
    }

    // for debugging
    triNumA = -1;
    triNumB = -1;
    edgeNumA = -1;
    edgeNumB = -1;
};

// advance to the next triangle-triangle pair that shares intersection point A
void TriPair::backward()
{
    if (triNumA == 0)
        face1 = face1->GetBordingFace(edgeNumA);
    else
    {
        assert(triNumA == 1);

        face2 = face2->GetBordingFace(edgeNumA);
    }

    // for debugging
    triNumA = -1;
    triNumB = -1;
    edgeNumA = -1;
    edgeNumB = -1;
};

bool shareVertex(const WFace * face1, const WFace * face2)
{
    for(int m=0;m<3;m++)
        for(int n=0;n<3;n++)
        {
            const WVertex * v1 = face1->GetVertex(m);
            const WVertex * v2 = face2->GetVertex(n);

            assert(v1 != NULL && v2 != NULL); // not sure if this can legitimately happen

            if (v1 == v2)
                return true;
        }
    return false;
}

WEdge * TriPair::getEdge(bool A)
{
    int triNum = A ? triNumA : triNumB;
    int edgeNum = A ? edgeNumA : edgeNumB;

    WFace * face = triNum == 0 ? face1 : face2;

    // there ought to be a better way to do this

    WOEdge * oedge=NULL;

    bool result = face->getOppositeEdge( face->GetVertex((edgeNum+2)%3), oedge);

    assert(result && oedge != NULL);

    return oedge->GetOwner();
}

bool shareEdge(WFace *face1, WFace* face2)
{
    for(int e=0;e<3;e++)
        if (face1->GetBordingFace(e)==face2)
            return true;
    return false;
}

bool adjacent(const WFace * face1, const WFace * face2)
{
    if (shareVertex(face1,face2))
        return true;


    // --- check if these are adjacent triangles where one was generated as PO cusp region geom, and the other was a source for it ---

    const WXFace * wxf1 = (WXFace*)face1;
    const WXFace * wxf2 = (WXFace*)face2;

    // note: this test may reject too many candidates, and could be done more carefully by considering precisely the topology of the PO regions
    if (wxf1->sourcePOB() != NULL)// && wxf2->sourcePOB() == NULL)
    {
        POBoundaryEdge * po = wxf1->sourcePOB();

        if (adjacent(po->sourceFace, wxf2) || adjacent(po->myFace, wxf2))
            return true;
    }

    if (wxf2->sourcePOB() != NULL)// && wxf1->sourcePOB() == NULL)
    {
        POBoundaryEdge * po = wxf2->sourcePOB();

        if (adjacent(po->sourceFace, wxf1) || adjacent(po->myFace, wxf1))
            return true;
    }
    /*
  // ---- check if both are PO cusp region geom that are adjacent but don't share vertices.
  // this can happen if they should be topologically connected, but the PO boundary got split into two regions
  // I haven't investigated how plausible this split is, but it appears to be occurring in practice.

  if (wxf1->sourcePOB() != NULL && wxf2->sourcePOB() != NULL)
    {
      POBoundaryEdge * po1 = wxf1->sourcePOB();
      POBoundaryEdge * po2 = wxf2->sourcePOB();

      if (adjacent(po1->sourceFace, po2->sourceFace) &&
      adjacent(po1->myFace, po2->myFace))
    return true;
    }
  */

    return false;
}


inline
bool adjacent(const TriPair & tp)
{
    return adjacent(tp.face1, tp.face2);
}


bool intersectFaces(TriPair & tp, Vec3r * lastpt = NULL, bool A = true)
// given the faces in "tp", compute the rest of the data structure, as described above
// If "lastpt" isn't NULL, then sort it so that the point A is the same as "lastpt"
//                         unless "A" is false, in which case point B becomes same as "lastpt"
{
    // ------------------- check for intersection -------------------------

    assert(tp.face1->numberOfVertices() == 3 && tp.face1->numberOfVertices() == 3);
    assert(tp.face1 != tp.face2);

    if (adjacent(tp))
    {
#ifdef DEBUG_INTERSECTION
        printf("WARNING: adjacent triangles in intersectFaces()\n");
#endif
        return false;
    }

    int coplanar = 0;
    real source[3], target[3];
    real t1[3][3], t2[3][3];
    for(int m=0;m<3;m++)
        for(int n=0;n<3;n++)
        {
            t1[m][n] = tp.face1->GetVertex(m)->GetVertex()[n];
            t2[m][n] = tp.face2->GetVertex(m)->GetVertex()[n];
        }


    int result = tri_tri_intersection_test_3d(t1[0], t1[1], t1[2],
            t2[0], t2[1], t2[2],
            &coplanar, source, target);

    if (result == 0)
    {
        //      printf("No intersection: [%08X, %08X]\n", tp.face1, tp.face2);
        return false;
    }

    // it appears that the intersection_test code never returns 1 (with probability 1), so I hacked it
    // the other code returned NaNs when I tried it, but that might have been an earlier bug
    if (coplanar == 1)
    {
        printf("WARNING: Ignoring coplanar triangles\n");
        return false;
    }

    if (isinf(source[0]) || isinf(source[1]) || isinf(source[2]) ||
            isinf(target[0]) || isinf(target[1]) || isinf(target[2]))
    {
        printf("WARNING: Ignoring INF in triangle-triangle intersection\n");
        return false;
    }
    if (isnan(source[0]) || isnan(source[1]) || isnan(source[2]) ||
            isnan(target[0]) || isnan(target[1]) || isnan(target[2]))
    {
        printf("WARNING: Ignoring NaN in triangle-triangle intersection\n");
        return false;
    }

#ifdef DEBUG_INTERSECTION
    printf("Intersection detected. Coplanar = %d\n", coplanar);
    printf("Face1: %08X, [%08X,%08X,%08X]\n", (unsigned long int)tp.face1,
           (unsigned long int) tp.face1->GetVertex(0),(unsigned long int) tp.face1->GetVertex(1),(unsigned long int) tp.face1->GetVertex(2));
    printf("Face2: %08X, [%08X,%08X,%08X]\n", (unsigned long int)tp.face2,
           (unsigned long int) tp.face2->GetVertex(0),(unsigned long int) tp.face2->GetVertex(1),(unsigned long int) tp.face2->GetVertex(2));
    printf("\tt1: [%f %f %f; %f %f %f; %f %f %f]\n",
           t1[0][0], t1[0][1], t1[0][2], t1[1][0], t1[1][1], t1[1][2], t1[2][0], t1[2][1], t1[2][2]);
    printf("\tt2: [%f %f %f; %f %f %f; %f %f %f]\n",
           t2[0][0], t2[0][1], t2[0][2], t2[1][0], t2[1][1], t2[1][2], t2[2][0], t2[2][1], t2[2][2]);
    printf("\tclf; \n");
    printf("\tplot3([%f %f %f %f],[%f %f %f %f],[%f %f %f %f]); hold on;\n",
           t1[0][0],t1[1][0],t1[2][0],t1[0][0],
            t1[0][1],t1[1][1],t1[2][1],t1[0][1],
            t1[0][2],t1[1][2],t1[2][2],t1[0][2]);
    printf("\tplot3([%f %f %f %f],[%f %f %f %f],[%f %f %f %f])\n",
           t2[0][0],t2[1][0],t2[2][0],t2[0][0],
            t2[0][1],t2[1][1],t2[2][1],t2[0][1],
            t2[0][2],t2[1][2],t2[2][2],t2[0][2]);
    printf("\tplot3([%f %f],[%f %f],[%f %f])\n", source[0], target[0], source[1], target[1], source[2], target[2]);
#endif

    /*
  if (coplanar == 1)
    {
      printf("WARNING: COPLANAR TRIANGLES DETECTED\n");
      printf("\tt1: [%f %f %f; %f %f %f; %f %f %f]\n",
         t1[0][0], t1[0][1], t1[0][2], t1[1][0], t1[1][1], t1[1][2], t1[2][0], t1[2][1], t1[2][2]);
      printf("\tt2: [%f %f %f; %f %f %f; %f %f %f]\n",
         t2[0][0], t2[0][1], t2[0][2], t2[1][0], t2[1][1], t2[1][2], t2[2][0], t2[2][1], t2[2][2]);
      return false;
    }
  */


    // ---------- assign intersection pts to A and B according to the arguments ----

    Vec3r src(source[0], source[1], source[2]);
    Vec3r tgt(target[0], target[1], target[2]);

    if (lastpt == NULL)
    {
        tp.ptA = src;
        tp.ptB = tgt;
    }
    else
    {
        real dsrc = (src-*lastpt).squareNorm();
        real dtgt = (tgt-*lastpt).squareNorm();
        if ( (dsrc < dtgt && A) ||
             (dtgt < dsrc && !A))
        {
            tp.ptA = src;
            tp.ptB = tgt;
        }
        else
        {
            tp.ptA = tgt;
            tp.ptB = src;
        }

    }

    // ---------- figure out which intersection goes with which face edge -----
    // this information is probably already contained in the intersection code, but it's not easy to extract it

    WFace * faces[2] = { tp.face1, tp.face2 };

    real min_distA = DBL_MAX, min_distB = DBL_MAX;

#ifdef DEBUG_INTERSECTION
    printf("A: [%f %f %f], B: [%f %f %f]\n", tp.ptA[0], tp.ptA[1], tp.ptA[2], tp.ptB[0], tp.ptB[1], tp.ptB[2]);
#endif

    for(int f=0;f<2;f++)
        for(int e=0;e<3;e++)
        {
#ifdef DEBUG_INTERSECTION
            printf("\tf: %d, e: %d, ",f,e);
#endif

            WOEdge * oedge = faces[f]->GetOEdge(e);

            Vec3f v1 = oedge->GetaVertex()->GetVertex();
            Vec3f v2 = oedge->GetbVertex()->GetVertex();
            //	Vec3f v1 = faces[f]->GetVertex((e+1)%3)->GetVertex();
            //	Vec3f v2 = faces[f]->GetVertex((e+2)%3)->GetVertex();

#ifdef DEBUG_INTERSECTION
            printf("v1: [%f %f %f], v2: [%f %f %f], ", v1[0], v1[1], v1[2], v2[0], v2[1], v2[2]);
#endif

            real distA = GeomUtils::distPointSegment<Vec3r>(tp.ptA, v1, v2);

            //	assert(distA < DBL_MAX);

            if (distA < min_distA)
            {
                min_distA = distA;
                tp.triNumA = f;
                tp.edgeNumA = e;
            }

            real distB = GeomUtils::distPointSegment<Vec3r>(tp.ptB, v1, v2);

            //	assert(distB < DBL_MAX);

            if (distB < min_distB)
            {
                min_distB = distB;
                tp.triNumB = f;
                tp.edgeNumB = e;
            }

#ifdef DEBUG_INTERSECTION
            printf("distA: %f, distB: %f\n", distA, distB);
#endif
        }

#ifdef DEBUG_INTERSECTION
    printf("DI: tp.trinumA: %d, tp.trinumB: %d, tp.edgeNumA: %d, tp.edgeNumB: %d\n",
           tp.triNumA, tp.triNumB, tp. edgeNumA, tp.edgeNumB);

    //  ComputeBarycentricCoords(tp.face1, tp.ptA);
    //  ComputeBarycentricCoords(tp.face2, tp.ptA);
    //  ComputeBarycentricCoords(tp.face1, tp.ptB);
    //  ComputeBarycentricCoords(tp.face2, tp.ptB);
#endif

    //  assert(! (tp.triNumA == tp.triNumB && tp.edgeNumA == tp.edgeNumB) );

    if ((tp.triNumA == tp.triNumB && tp.edgeNumA == tp.edgeNumB) ||
            (tp.triNumA == -1) || (tp.triNumB == -1))
    {
        badIntersection = true;

#ifdef DEBUG_INTERSECTION
        printf("=======================\n");
        printf("WARNING: BAD/DEGENERATE TRIANGLE INTERSECTION\n");

        printf("tp.trinumA: %d, tp.trinumB: %d, tp.edgeNumA: %d, tp.edgeNumB: %d\n",
               tp.triNumA, tp.triNumB, tp. edgeNumA, tp.edgeNumB);
        printf("Face1: %08X, [%08X,%08X,%08X]\n", (unsigned long int)tp.face1,
               (unsigned long int) tp.face1->GetVertex(0),(unsigned long int) tp.face1->GetVertex(1),(unsigned long int) tp.face1->GetVertex(2));
        printf("Face2: %08X, [%08X,%08X,%08X]\n", (unsigned long int)tp.face2,
               (unsigned long int) tp.face2->GetVertex(0),(unsigned long int) tp.face2->GetVertex(1),(unsigned long int) tp.face2->GetVertex(2));
        printf("\tt1: [%f %f %f; \n\t    %f %f %f; \n\t    %f %f %f]\n",
               t1[0][0], t1[0][1], t1[0][2], t1[1][0], t1[1][1], t1[1][2], t1[2][0], t1[2][1], t1[2][2]);
        printf("\tt2: [%f %f %f; \n\t    %f %f %f; \n\t    %f %f %f]\n",
               t2[0][0], t2[0][1], t2[0][2], t2[1][0], t2[1][1], t2[1][2], t2[2][0], t2[2][1], t2[2][2]);
        printf("\tclf; \n");
        printf("\tplot3([%f %f %f %f],[%f %f %f %f],[%f %f %f %f]); hold on;\n",
               t1[0][0],t1[1][0],t1[2][0],t1[0][0],
                t1[0][1],t1[1][1],t1[2][1],t1[0][1],
                t1[0][2],t1[1][2],t1[2][2],t1[0][2]);
        printf("\tplot3([%f %f %f %f],[%f %f %f %f],[%f %f %f %f])\n",
               t2[0][0],t2[1][0],t2[2][0],t2[0][0],
                t2[0][1],t2[1][1],t2[2][1],t2[0][1],
                t2[0][2],t2[1][2],t2[2][2],t2[0][2]);
        printf("\tplot3([%f %f],[%f %f],[%f %f])\n", source[0], target[0], source[1], target[1], source[2], target[2]);
        printf("A: [%f %f %f], B: [%f %f %f]\n", tp.ptA[0], tp.ptA[1], tp.ptA[2], tp.ptB[0], tp.ptB[1], tp.ptB[2]);

        printf("face1->sourcePOB: %08X, face2->sourcePOB: %08X\n",
               ((WXFace*)tp.face1)->sourcePOB(), ((WXFace*)tp.face2)->sourcePOB());
        printf("=======================\n");


        WXFace * wxf1 = (WXFace*)tp.face1;
        WXFace * wxf2 = (WXFace*)tp.face2;

        if (tp.triNumA == -1 || tp.triNumB == -1)
            for(int i=0;i<3;i++)
                printf("face1 v[%d] = %08X,  face2 v[%d] = %08X\n", i, wxf1->GetVertex(i), i, wxf2->GetVertex(i));

        makeDebugTriangles(wxf1, wxf2);
#endif

        assert(tp.triNumA != -1 && tp.triNumB != -1);

        return false;
    }





    // -------------------- break region edges (used later when building region graph) ------
    //  printf("tp.trinumA: %d, tp.trinumB: %d, tp.edgeNumA: %d, tp.edgeNumB: %d\n",
    //	 tp.triNumA, tp.triNumB, tp. edgeNumA, tp.edgeNumB);


    WEdge *eA=  faces[tp.triNumA]->GetOEdge(tp.edgeNumA)->GetOwner();
    WEdge *eB=  faces[tp.triNumB]->GetOEdge(tp.edgeNumB)->GetOwner();

    assert(dynamic_cast<WXEdge*>(eA) != NULL);
    assert(dynamic_cast<WXEdge*>(eB) != NULL);

    ((WXEdge*)eA)->SetRegionEdge(false);
    ((WXEdge*)eB)->SetRegionEdge(false);



    return true;
}

bool alreadyVisited(const TriPair & tp, const set<pair<const WFace*,const WFace*> > & processed)
{
    return (processed.find(pair<const WFace*,const WFace*>(tp.face1,tp.face2)) != processed.end()) ||
            (processed.find(pair<const WFace*,const WFace*>(tp.face2, tp.face1)) != processed.end());
}

/*
bool alreadyVisited(const WFace *f1, const WFace*f2, const set<pair<const WFace*,const WFace*> > & processed)
{
  return (processed.find(pair<const WFace*,const WFace*>(f1,f2)) != processed.end()) ||
    (processed.find(pair<const WFace*,const WFace*>(f2,f1)) != processed.end());
}
*/

//#define MEMTEST {   _ViewMap->AddViewEdge(new ViewEdge); }



void ViewMapBuilder::computeSelfIntersections(WingedEdge & we)
{
    printf("Computing self intersections:\n");
    fflush(stdout);

    badIntersection = false;

    FastGrid * fg = dynamic_cast<FastGrid*>(_Grid);


    bool progressBarDisplay = false;
    unsigned progressBarStep = 0;

    if(_pProgressBar != NULL && fg->numNonempty() > gProgressBarMinSize) {
        unsigned progressBarSteps = fg->numNonempty();
        if (progressBarSteps > 100)
            progressBarSteps = 100;
        progressBarStep = fg->numNonempty() / progressBarSteps;
        _pProgressBar->reset();
        _pProgressBar->setLabelText("Computing Self-Intersections");
        _pProgressBar->setTotalSteps(progressBarSteps);
        _pProgressBar->setProgress(0);
        progressBarDisplay = true;
    }

    unsigned counter = progressBarStep;


    /*
  NodeShape *seNode = new NodeShape();
  visDebugNode->AddChild(seNode);
  Material semat;
  semat.SetDiffuse(1,0,1,1);
  semat.SetEmission(0,0,0,0);
  semat.SetAmbient(0,0,0,0);

  seNode->SetMaterial(semat);
  */


    //  PRINTMEM

    assert(we.getWShapes().size() > 0);

    int id = 0;
    for(int i=0;i<we.getWShapes().size();i++)
    {
        int sid = we.getWShapes()[i]->GetId();
        if (sid > id)
            id = sid + 1;
    }

    SShape * psShape = new SShape;
    assert(psShape != NULL);
    psShape->SetId( id );
    psShape->SetMaterials( we.getWShapes()[0]->materials());

    ViewShape * vshape = new ViewShape(psShape);
    assert(vshape != NULL);
    _ViewMap->AddViewShape(vshape);
    psShape->SetViewShape(vshape);

    set<pair<const WFace*,const WFace*> > processed;

    list<TriPair> chain;

    //  FILE * histFile = fopen("hist.txt","wt");

    int n = 0, firstId = _currentId, firstFId = _currentFId;
    // iterate over all non-empty grid cells
    for(FastGrid::FGiterator it = fg->beginFG() ; it != fg->endFG(); ++it)
        //  for(Grid::iterator it = _Grid->begin(); it!= _Grid->end(); ++it)
    {
        printf("Visiting cell # %d / %d. Created so far: %d ViewEdges, %d FEdges   \r",
               n++, fg->numNonempty()-1, _currentId-firstId, _currentFId - firstFId);
        //      printf("sizeof: ViewEdge: %d, ViewVertex: %d, FEdgeSmooth: %d, SVertex: %d\n",
        //	     (int)sizeof(ViewEdge), (int)sizeof(ViewVertex), (int)sizeof(FEdgeSmooth),
        //	     (int)sizeof(SVertex));

        vector<Polygon3r*> & tris = _Grid->getCell(*it)->getOccluders();

        //      fprintf(histFile, "%d ", tris.size());

        for(int i=0;i<tris.size();i++)
            for(int j=i+1;j<tris.size();j++)
            {
                TriPair start_tp;

                start_tp.face1 = (WFace*)tris[i]->userdata;
                start_tp.face2 = (WFace*)tris[j]->userdata;

                // ------- check if we've already done this pair of triangles ---------

                if (alreadyVisited(start_tp,processed))
                    continue;

                // -------------- check if the polys share any vertices ---------------

                if (adjacent(start_tp))
                {
                    //		printf("Skipping adjacent: [%08X, %08X]\n", start_tp.face1, start_tp.face2);
                    continue;
                }

                // ------------------------ check for intersection ------------------------

                bool result = intersectFaces(start_tp);
                if (result == false)
                    continue;

                processed.insert(pair<const WFace*,const WFace*>(start_tp.face1,start_tp.face2));

                // ------------------------ create the chain ------------------------------

                bool closed = false;

                chain.clear();
                chain.push_back(start_tp);

                // ------------------------ forward chaining pass -------------------------

                TriPair lasttp = start_tp;

                while(true)
                {
                    TriPair tp = lasttp;

                    // advance to the next pair of faces
#ifdef DEBUG_INTERSECTION
                    printf("forward\n");
#endif
                    tp.forward();

                    // check if we've reached an object boundary
                    if (tp.face1 == NULL || tp.face2 == NULL)
                    {
#ifdef DEBUG_INTERSECTION
                        printf("reached object boundary\n");
#endif
                        break;
                    }

                    // check if we're back where we started
                    if ( (tp.face1 == start_tp.face1 && tp.face2 == start_tp.face2) ||
                         (tp.face1 == start_tp.face2 && tp.face2 == start_tp.face1))
                    {
#ifdef DEBUG_INTERSECTION
                        printf("Closed the loop\n");
#endif
                        closed = true;
                        break;
                    }

                    // check if this pair has already been processed
                    if (alreadyVisited(tp, processed))
                    {
#ifdef DEBUG_INTERSECTION
                        printf("WARNING: found already-processed pair without loop closure\n");
                        printf("Face1: %08X [%08X, %08X, %08X]\n", tp.face1,
                               tp.face1->GetVertex(0),tp.face1->GetVertex(1),tp.face1->GetVertex(2));
                        printf("Face2: %08X [%08X, %08X, %08X]\n", tp.face2,
                               tp.face2->GetVertex(0),tp.face2->GetVertex(1),tp.face2->GetVertex(2));

                        real t1[3][3], t2[3][3];
                        for(int m=0;m<3;m++)
                            for(int n=0;n<3;n++)
                            {
                                t1[m][n] = tp.face1->GetVertex(m)->GetVertex()[n];
                                t2[m][n] = tp.face2->GetVertex(m)->GetVertex()[n];
                            }

                        printf("\tplot3([%f %f %f %f],[%f %f %f %f],[%f %f %f %f]); hold on;\n",
                               t1[0][0],t1[1][0],t1[2][0],t1[0][0],
                                t1[0][1],t1[1][1],t1[2][1],t1[0][1],
                                t1[0][2],t1[1][2],t1[2][2],t1[0][2]);
                        printf("\tplot3([%f %f %f %f],[%f %f %f %f],[%f %f %f %f])\n",
                               t2[0][0],t2[1][0],t2[2][0],t2[0][0],
                                t2[0][1],t2[1][1],t2[2][1],t2[0][1],
                                t2[0][2],t2[1][2],t2[2][2],t2[0][2]);


                        bool result = intersectFaces(tp);
                        printf("  test: %s\n", result ? "INTERSECTION" : "NO INTERSECTION");
#endif
                        break;
                    }

                    processed.insert(pair<const WFace*,const WFace*>(tp.face1,tp.face2));

                    // compute the intersection between these two faces
                    // if things are working properly, these faces must intersect
                    // (unless they are adjacent on the surface, not sure if this case is important)
                    result = intersectFaces(tp, &lasttp.ptB,true);
                    if (!result)
                    {
#ifdef DEBUG_INTERSECTION
                        printf("WARNING: didn't find expected intersection\n");


                        real t1[3][3], t2[3][3];
                        for(int m=0;m<3;m++)
                            for(int n=0;n<3;n++)
                            {
                                t1[m][n] = tp.face1->GetVertex(m)->GetVertex()[n];
                                t2[m][n] = tp.face2->GetVertex(m)->GetVertex()[n];
                            }

                        printf("\tplot3([%f %f %f %f],[%f %f %f %f],[%f %f %f %f]); hold on;\n",
                               t1[0][0],t1[1][0],t1[2][0],t1[0][0],
                                t1[0][1],t1[1][1],t1[2][1],t1[0][1],
                                t1[0][2],t1[1][2],t1[2][2],t1[0][2]);
                        printf("\tplot3([%f %f %f %f],[%f %f %f %f],[%f %f %f %f])\n",
                               t2[0][0],t2[1][0],t2[2][0],t2[0][0],
                                t2[0][1],t2[1][1],t2[2][1],t2[0][1],
                                t2[0][2],t2[1][2],t2[2][2],t2[0][2]);

                        //		    makeDebugTriangles(tp.face1, tp.face2);
#endif
                        break;
                    }
                    //		  FATAL_ERROR("didn't find expected intersection");

                    chain.push_back(tp);
                    lasttp = tp;
                }

                // ------------------------ backward chaining -----------------------------

                if (!closed)
                {
#ifdef DEBUG_INTERSECTION
                    printf("doing bidirectional chaining\n");
#endif

                    lasttp = start_tp;

                    while(true)
                    {
                        TriPair tp = lasttp;

                        Vec3r last_ptA = lasttp.ptA;

                        // advance to the next pair of faces
                        tp.backward();
                        if (tp.face1 == NULL || tp.face2 == NULL)   // should only happens at object boundaries
                            break;

                        if (alreadyVisited(tp, processed))
                        {
#ifdef DEBUG_INTERSECTION
                            printf("warning: found already-processed pair without loop closure\n");
#endif
                            break;
                        }

                        processed.insert(pair<const WFace*,const WFace*>(tp.face1,tp.face2));

                        // compute the intersection between these two faces
                        result = intersectFaces(tp,&lasttp.ptA,false);
                        if (result == false)
                            break;
                        //		      FATAL_ERROR("didn't find expected intersection");

                        chain.push_front(tp);
                        lasttp = tp;
                    }

                }

                // ------- we now have a chain of face pairs, with ordered pts (A,B) -------
                // ------- now we generate the corresponding chain of FEdges, and the ViewEdge and ViewVertices

                //	    printf("i = %d, j= %d\n", i,j);

                //	    PRINTMEM

                //	    _ViewMap->AddViewEdge(newVEdge);
                ///	    vector<ViewEdge*> & ves = _ViewMap->ViewEdges();
                //	    printf("ves.size() = %d\n", ves.size());
                //	    ves.push_back(newVEdge);


                bool isPORegion = ((WXFace*)start_tp.face1)->sourcePOB() != NULL ||
                        ((WXFace*)start_tp.face2)->sourcePOB() != NULL;


                ViewEdge * newVEdge = new ViewEdge;
                assert(newVEdge != NULL);

                newVEdge->SetNature(isPORegion ? Nature::PO_SURFACE_INTERSECTION : Nature::SURFACE_INTERSECTION);
                newVEdge->SetId(_currentId);
                _currentId++;

                _ViewMap->AddViewEdge(newVEdge);
                vshape->AddEdge(newVEdge);


#ifdef DEBUG_INTERSECTION
                Id id = newVEdge->getId();
                printf("NEW VIEWEDGE, ID: %d %d\n", id.getFirst(), id.getSecond());
#endif
                fflush(stdout);

                FEdgeIntersection * fe;
                FEdgeIntersection * fefirst = NULL;
                FEdgeIntersection * feprevious = NULL;
                SVertex * vA = NULL;
                SVertex * vB = NULL;
                SVertex * vFirst = NULL;

                vA = new SVertex( chain.front().ptA, _currentSVertexId);
                vA->SetSourceEdge(chain.front().getEdge(true));
                vFirst = vA;

                _currentSVertexId++;
                SilhouetteGeomEngine::ProjectSilhouette(vA);

                _ViewMap->AddSVertex(vA);
                psShape->AddNewVertex(vA);

                int k;
                list<TriPair>::iterator it;
                for(it = chain.begin(), k=0;it!= chain.end();++it, ++k)
                {
                    // check if we're at the end of a loop
                    if (closed && (k+1 == chain.size()) && (chain.size() > 1))
                    {
                        vB = vFirst;

#ifdef DEBUG_INTERSECTION
                        // make sure vFirst is on these faces
                        //		    printf("DEBUGGING LOOP CLOSURE\n");
                        //		    ComputeBarycentricCoords((*it).face1, vB->getPoint3D());
                        //		    ComputeBarycentricCoords((*it).face2, vB->getPoint3D());
#endif
                    }
                    else
                    {
                        vB = new SVertex( (*it).ptB, _currentSVertexId);
                        vB->SetSourceEdge((*it).getEdge(false));
                        _currentSVertexId++;

                        SilhouetteGeomEngine::ProjectSilhouette(vB);
                        _ViewMap->AddSVertex(vB);
                        psShape->AddNewVertex(vB);
                    }

                    fe = new FEdgeIntersection(vA, vB);
                    fe->SetViewEdge(newVEdge);
                    fe->SetNature(isPORegion ? Nature::PO_SURFACE_INTERSECTION : Nature::SURFACE_INTERSECTION);//Nature::SURFACE_INTERSECTION);
                    fe->SetId(_currentFId);
                    _currentFId++;
                    fe->SetPreviousEdge(feprevious);
                    fe->SetFaces( (*it).face1 , (*it).face2 );
                    assert((*it).face1 != NULL && (*it).face2 != NULL);

                    if (feprevious != NULL)
                        feprevious->SetNextEdge(fe);
                    //		(*it).face1->getIntersections().push_back(fe);
                    //		(*it).face2->getIntersections().push_back(fe);

                    vA->AddFEdge(fe);
                    vB->AddFEdge(fe);
                    _ViewMap->AddFEdge(fe);

#ifdef DEBUG_INTERSECTION
                    printf("\t NEW FE: [%f %f %f], [%f %f %f], ID: %d %d\n", vA->getX(),vA->getY(),vA->getZ(),
                           vB->getX(),vB->getY(),vB->getZ(), fe->getId().getFirst(), fe->getId().getSecond());

                    //		debugFES(fe);
#endif

                    if (fefirst == NULL)
                        fefirst = fe;

                    // increment pointers along the chain
                    feprevious = fe;
                    vA = vB;
                }

                psShape->AddChain(fefirst);

                newVEdge->SetFEdgeA(fefirst);
                newVEdge->SetFEdgeB(fe);

                if (closed)
                {
                    newVEdge->SetA(NULL);
                    newVEdge->SetB(NULL);
                    fe->SetNextEdge(fefirst);
                    fefirst->SetPreviousEdge(fe);
                }
                else
                {
                    // create view vertecies for the endpoints.
                    // Usually a NonTVertex, but a TVertex for one special case (end of PO cusp region connecting to silhouette)

                    //		if (!isPORegion || chain.front().getPOendpoint(_ViewMap,true) == NULL)
                    //		  {
                    // just create a plain new vertex here
                    NonTVertex * vva = new NonTVertex(fefirst->vertexA());
                    newVEdge->SetA(vva);
                    vva->AddOutgoingViewEdge(newVEdge);
                    _ViewMap->AddViewVertex(vva);

                    //		if (!isPORegion || chain.back().getPOendpoint(_ViewMap,false) == NULL)
                    //		  {
                    NonTVertex * vvb = new NonTVertex(fe->vertexB());
                    newVEdge->SetB(vvb);
                    vvb->AddIncomingViewEdge(newVEdge);

                    _ViewMap->AddViewVertex(vvb);
                }
            }

        if(progressBarDisplay) {
            counter--;
            if (counter <= 0) {
                counter = progressBarStep;
                _pProgressBar->setProgress(_pProgressBar->getProgress() + 1);
            }
        }
    }

    // ----- make visualization ----

#ifdef DEBUG_INTERSECTION
    NodeShape * triangles = new NodeShape;
    visDebugNode->AddChild(triangles);

    for(vector<TriangleRep*>::iterator it = debugTriangles.begin(); it != debugTriangles.end(); ++it)
    {
        TriangleRep * t = *it;

        t->ComputeBBox();
        t->SetStyle(TriangleRep::FILL);
        triangles->AddRep(t);
    }
    debugTriangles.clear();
#endif

    if (badIntersection)
        printf("WARNING: DEGENERATE INTERSECTIONS DETECTED\n");

    printf("\nDone computing self-intersections\n");
    //  fclose(histFile);
}


void ViewMapBuilder::computeInitialViewEdges(WingedEdge& we)
{
    vector<WShape*> wshapes = we.getWShapes();
    SShape* psShape;

    for (vector<WShape*>::const_iterator it = wshapes.begin();
         it != wshapes.end();
         it++) {
        // create the embedding
        psShape = new SShape;
        psShape->SetId((*it)->GetId());
        psShape->SetMaterials((*it)->materials()); // FIXME

        // create the view shape
        ViewShape * vshape = new ViewShape(psShape);
        // add this view shape to the view map:
        _ViewMap->AddViewShape(vshape);
        psShape->SetViewShape(vshape);



        _pViewEdgeBuilder->SetCurrentViewId(_currentId); // we want to number the view edges in a unique way for the while scene.
        _pViewEdgeBuilder->SetCurrentFId(_currentFId); // we want to number the feature edges in a unique way for the while scene.
        _pViewEdgeBuilder->SetCurrentSVertexId(_currentFId); // we want to number the SVertex in a unique way for the while scene.
        _pViewEdgeBuilder->BuildViewEdges(dynamic_cast<WXShape*>(*it), vshape,
                                          _ViewMap->ViewEdges(),
                                          _ViewMap->ViewVertices(),
                                          _ViewMap->FEdges(),
                                          _ViewMap->SVertices());

        _currentId = _pViewEdgeBuilder->currentViewId()+1;
        _currentFId = _pViewEdgeBuilder->currentFId()+1;
        _currentSVertexId = _pViewEdgeBuilder->currentSVertexId()+1;

        psShape->ComputeBBox();
    }
}

WFace * GetNearFace(WEdge * edge, Vec3r viewpoint, real * discriminant = NULL)
{
    assert(edge->GetaFace() != NULL && edge->GetbFace() != NULL);

    // find the opp vertices
    WVertex * oppA = NULL;
    for(int j=0;j<3;j++)
    {
        WVertex * vert = edge->GetaFace()->GetVertex(j);
        if (vert != edge->GetaVertex() && vert != edge->GetbVertex())
        {
            oppA = vert;
            break;
        }
    }
    WVertex * oppB = NULL;
    for(int j=0;j<3;j++)
    {
        WVertex * vert = edge->GetbFace()->GetVertex(j);
        if (vert != edge->GetaVertex() && vert != edge->GetbVertex())
        {
            oppB = vert;
            break;
        }
    }

    // Note: this formula only works at contours in theory.  in practice, doesn't seem to fix problems.
    //  bool Anear = sameSide(edge->GetbFace(), oppA->GetVertex(), viewpoint);
    //  bool Bnear = sameSide(edge->GetaFace(), oppB->GetVertex(), viewpoint);

    //  if (Anear != Bnear)
    //    return Anear ? edge->GetaFace() : edge->GetbFace();
    //  else
    //  {
    Vec3r vA = edge->GetaVertex()->GetVertex();
    Vec3r vB = edge->GetbVertex()->GetVertex();
    Vec3r midpoint = (vA+vB)/2;
    Vec3r edgeDir = vA-vB;
    
    edgeDir = edgeDir/edgeDir.norm();
    
    Vec3r vv = viewpoint - midpoint;
    
    // unit vectors in the plane of each face (?), perpendicular to the edge
    Vec3r edgePerpA =   oppA->GetVertex() - midpoint;
    edgePerpA = edgePerpA -  (edgePerpA*edgeDir) * edgeDir;
    edgePerpA = edgePerpA/edgePerpA.norm();
    Vec3r edgePerpB =   oppB->GetVertex() - midpoint;
    edgePerpB = edgePerpB - (edgePerpB * edgeDir) * edgeDir;
    edgePerpB = edgePerpB/edgePerpB.norm();
    
    //  return (edgePerpA * vv > edgePerpB * vv)  ? edge->GetaFace() : edge->GetbFace();
    
    // unit vectors in the plane of each face, parallel to the view vector to the midpoint and with the same orientation
    Vec3r adir = vv - (vv * edge->GetaFace()->GetNormal()) * edge->GetaFace()->GetNormal();
    Vec3r bdir = vv - (vv * edge->GetbFace()->GetNormal()) * edge->GetbFace()->GetNormal();
    
    if (discriminant != NULL)
        *discriminant = (adir -bdir)* vv / vv.norm();

    if (adir * edgePerpA < 0)
        adir = adir/(-adir.norm());
    else
        adir = adir/adir.norm();
    if (bdir * edgePerpB < 0)
        bdir = bdir/(-bdir.norm());
    else
        bdir = bdir/bdir.norm();
    
    return ((adir - bdir) * vv > 0)  ? edge->GetaFace() : edge->GetbFace();
    //    return (adir * vv > bdir * vv)  ? edge->GetaFace() : edge->GetbFace();
    //  }

    /*
  // define a plane containing [midpoint, viewpoint, planePoint]
  Vec3r planePoint = oppA->GetVertex();
  //  Vec3r planePoint = (oppA->GetVertex() + oppB->GetVertex())/2;
  Vec3r planeNormal = (planePoint - midpoint) ^ (midpoint - viewpoint);
  
  // create unit vectors that are the intersection of the new plane with the two faces

  Vec3r adir = edge->GetaFace()->GetNormal() ^ planeNormal;
  if (adir * (oppA->GetVertex() - midpoint) < 0)
    adir = adir/(-adir.norm());
  else
    adir = adir/adir.norm();

  Vec3r bdir = edge->GetbFace()->GetNormal() ^ planeNormal;
  if (bdir * (oppB->GetVertex() - midpoint) < 0)
    bdir = bdir/(-bdir.norm());
  else
    bdir = bdir/bdir.norm();
  return (adir * vv > bdir * vv)  ? edge->GetaFace() : edge->GetbFace();

  */

}


// is the sharp edge "fesh" that ends at the vertex "wv" occluded by another face in the one ring of 
// the vertex?
// result: 1: overlap exists, 0: no overlap, -1: can't tell due to inconsistency

int OneRingOcclusion(SVertex * sv, WVertex * wv, FEdgeSharp * fesh, set<WXFace*> & oneRing, 
                     Vec3r & viewpoint, bool useConsistency)
{
    int result = 0;

    // iterate over the one-ring of wv and check if any face occludes the edge fesh
    for(set<WXFace*>::iterator fit = oneRing.begin(); fit != oneRing.end(); ++fit)
    {
        WXFace * face = *fit;

        // find the vertex's index in this triangle
        int vind = face->GetIndex(wv);
        assert(vind != -1);   // make sure the face is part of the one-ring
        
        // check if the face is back-facing
        if (!face->front(useConsistency))
            continue;

        // check if the face is inconsistent
        if (useConsistency && !face->consistent())
        {
            result = -1;
            continue;
        }

        // check if the face is adjacent to the edge
        if (fesh->edge()->GetaFace() == face || fesh->edge()->GetbFace() == face)
            continue;
        
        // check for image-space overlap.  this is reduces to three clipping tests

        assert(fesh->vertexA() == sv || fesh->vertexB() == sv);

        SVertex * endpoint = (fesh->vertexA() == sv ? fesh->vertexB() : fesh->vertexA());
        Vec3r endPos = endpoint->getPoint3D();
        Vec3r vPos = wv->GetVertex();
        assert(wv == face->GetVertex(vind));
        Vec3r v1Pos = face->GetVertex( (vind+1)%3)->GetVertex();
        Vec3r v2Pos = face->GetVertex( (vind+2)%3)->GetVertex();
        
        if (!sameSide(face, endPos, viewpoint) &&
                sameSide(viewpoint, vPos, v1Pos, endPos, v2Pos) &&
                sameSide(viewpoint, vPos, v2Pos, endPos, v1Pos))
            return 1;
    }
    return result;
}

// check if the sharp edge is adjacent to a vertex, and occluded by the one-ring of that vertex
// result: 1: overlap exists, 0: no overlap, -1: can't tell due to inconsistency
int OneRingOcclusion(FEdgeSharp * fesh, Vec3r & viewpoint, bool useConsistency)
{
    int result = 0;

    for(int i=0;i<2;i++)
    {
        SVertex * sv = (i == 0 ? fesh->vertexA() : fesh->vertexB());
        WVertex * wv = sv->GetSourceVertex();
        if (wv == NULL)
            continue;
        
        set<WXFace*> oneRing;
        
        // collect the one-ring
        for(vector<WEdge*>::iterator wit = wv->GetEdges().begin(); wit != wv->GetEdges().end(); ++wit)
        {
            if ( (*wit)->GetaFace() != NULL)
                oneRing.insert( (WXFace*)(*wit)->GetaFace());
            if ( (*wit)->GetbFace() != NULL)
                oneRing.insert( (WXFace*)(*wit)->GetbFace());
        }
        
        int r = OneRingOcclusion(sv, wv, fesh, oneRing, viewpoint, useConsistency);

        if (r == 1)
            return 1;

        if (r == -1)
            result = -1;
    }

    return result;
}

// check if the vertex sv is a cusp  based on overlaps
// result: 1: cusp, 0: not a cusp, -1: can't tell due to inconsistency
int IsCusp(SVertex * sv, WVertex * wv, FEdgeSharp * fesh0, FEdgeSharp * fesh1, set<WXFace*> & oneRing, 
           Vec3r & viewpoint, bool useConsistency)
{
    int r1 = OneRingOcclusion(sv, wv, fesh0, oneRing, viewpoint, useConsistency);

    if (r1 == -1)
        return -1;

    if (r1 == 1)
        return 1;

    int r2 = OneRingOcclusion(sv, wv, fesh1, oneRing, viewpoint, useConsistency);

    if (r2 == -1)
        return -1;

    if (r2 == 1)
        return 1;

    return 0;

    //    return (r1 == r2 ? 0 : 1);
}

bool DiscreteRadialSign(FEdgeSmooth * fesh, Vec3r & viewpoint)
{
    if (NEW_SILHOUETTE_HEURISTIC)
    {
        // sign (viewvec * (AB ^ normal))   (flipped depending on which side

        Vec3r A = fesh->vertexA()->point3d();
        Vec3r B = fesh->vertexB()->point3d();
        Vec3r AB = B-A;
        AB.normalize();
        Vec3r m((A+B)/2.0);
        Vec3r viewvector(m-viewpoint);
        viewvector.normalize();

        int vind = -1;
        real bestVdot = 0;
        WFace * face = (WFace*)fesh->face();
        for(int i=0;i<3;i++)
        {
            Vec3r v = face->GetVertex(i)->GetVertex() - m;
            v.normalize();
            real vdot = v * viewvector;
            if (fabs(vdot) > fabs(bestVdot))
            {
                vind = i;
                bestVdot = vdot;
            }
        }

        if (vind == -1)
            return false;

        real ndotv = ((WXVertex*)face->GetVertex(vind))->ndotv();
        return  (bestVdot > 0) == (ndotv > 0);
    }
    else
    {
        // TODO: try instead using just (midpoint-vertex) vectors


        Vec3r A = fesh->vertexA()->point3d();
        Vec3r B = fesh->vertexB()->point3d();
        Vec3r AB = B-A;
        AB.normalize();
        Vec3r m((A+B)/2.0);
        Vec3r crossP(AB^fesh->normal());
        crossP.normalize();
        Vec3r viewvector(m-viewpoint);
        viewvector.normalize();

        //---- point crossP in the positive direction ----
        WFace * face = (WFace*)fesh->face();
        real maxNdotVMag = 0;
        int vind = -1;
        for(int i=0;i<3;i++)
        {
            real ndotv = ((WXVertex*)face->GetVertex(i))->ndotv();
            if (fabs(ndotv) > maxNdotVMag)
            {
                vind = i;
                maxNdotVMag = fabs(ndotv);
            }
        }
        if (vind != -1)
        {
            real ndotv = ((WXVertex*)face->GetVertex(vind))->ndotv();
            Vec3r v = face->GetVertex(vind)->GetVertex() - m;
            v.normalize();
            if ( (v * crossP > 0) != (ndotv > 0))
                crossP = crossP * -1;
        }



        return crossP * viewvector > 0;

    }
}


void ViewMapBuilder::computeCusps(ViewMap *ioViewMap){
    /*
  NodeShape *cuspNode = new NodeShape();
  visDebugNode->AddChild(cuspNode);
  Material cusmat;
  cusmat.SetDiffuse(1,0,1,1);
  cusmat.SetEmission(0,0,0,0);
  cusmat.SetAmbient(0,0,0,0);

  cuspNode->SetMaterial(cusmat);
  */

    // ------------------ cusps for smooth curves --------------------------

    /* freestyle's old, questionable version that iterates over a list that it's modifying
  vector<ViewVertex*> newVVertices;
  vector<ViewEdge*> newVEdges;
  ViewMap::viewedges_container& vedges = ioViewMap->ViewEdges();
  ViewMap::viewedges_container::iterator ve=vedges.begin(), veend=vedges.end();
  for(;
  ve!=veend;
  ++ve){
    if((!((*ve)->getNature() & Nature::SILHOUETTE)) || (!((*ve)->fedgeA()->isSmooth())))
      continue;
    FEdge *fe = (*ve)->fedgeA();
    FEdge * fefirst = fe;
    bool first = true;
    bool positive = true;
    do{
      FEdgeSmooth * fes = dynamic_cast<FEdgeSmooth*>(fe);
      Vec3r A((fes)->vertexA()->point3d());
      Vec3r B((fes)->vertexB()->point3d());
      Vec3r AB(B-A);
      AB.normalize();
      Vec3r m((A+B)/2.0);
      Vec3r crossP(AB^(fes)->normal());
      crossP.normalize();
      Vec3r viewvector(m-_viewpoint);
      viewvector.normalize();
      if(first){
        if(((crossP)*(viewvector)) > 0)
          positive = true;
        else
          positive = false;
        first = false;
      }
      // If we're in a positive part, we need
      // a stronger negative value to change
      ViewVertex *cusp = 0;
      if(positive){
        if(((crossP)*(viewvector)) < -0.1){
          // state changes
          positive = false;
          // creates and insert cusp
      cusp = ioViewMap->InsertViewVertex(fes->vertexA(), newVEdges);
          if(cusp!=0)
        {
          cusp->setNature(cusp->getNature()|Nature::CUSP);
          ioViewMap->addDebugPoint(DebugPoint::CUSP, cusp->getPoint3D(), false, fes->vertexA());
        }
      else
        ioViewMap->addDebugPoint(DebugPoint::ERROR, fes->vertexA()->getPoint3D(), false, fes->vertexA());

      //	  addDebugPoint(cusp->getPoint3D(), cuspNode);
        }

      }else{
        // If we're in a negative part, we need
        // a stronger negative value to change
        if(((crossP)*(viewvector)) > 0.1){
          positive = true;
      cusp = ioViewMap->InsertViewVertex(fes->vertexA(), newVEdges);
      assert(cusp != 0);
          if(cusp!=0)
        {
          cusp->setNature(cusp->getNature()|Nature::CUSP);
          ioViewMap->addDebugPoint(DebugPoint::CUSP, cusp->getPoint3D(), false, fes->vertexA());
        }
      else
        ioViewMap->addDebugPoint(DebugPoint::ERROR, fes->vertexA()->getPoint3D(), false, fes->vertexA());
      //	  addDebugPoint(cusp->getPoint3D(), cuspNode);
        }
      }

      ViewEdge * nve = newVEdges.size() > 0 ? newVEdges.back() : NULL;
      assert(nve == NULL || nve->A() == NULL ||hasEdge(nve->A(),nve));
      assert(nve == NULL || nve->B() == NULL ||hasEdge(nve->B(),nve));

      fe = fe->nextEdge();
    }while((fe!=0) && (fe!=fefirst));
  }

  */
    vector<ViewEdge*> newVEdges;

    // cusps for smooth silhouettes


    for(vector<SVertex*>::iterator sit = ioViewMap->SVertices().begin(); sit != ioViewMap->SVertices().end();
        ++sit)
    {
        SVertex * sv = *sit;

        // only need to insert cusps in the interiors of ViewEdges
        if (sv->viewvertex() != NULL)
            continue;

        // retreive the two adjacent FEdges

        FEdge * fes[2];

        fes[0] = sv->fedge();
        fes[1] = fes[0]->vertexB() == sv ? fes[0]->nextEdge() : fes[0]->previousEdge();

        if (fes[0] == NULL || fes[1] == NULL)
            continue;

        // only consider smooth silhouettes and boundaries
        if (!(fes[0]->getNature() & Nature::SILHOUETTE) || !fes[0]->isSmooth())
            continue;

        assert(fes[0]->getNature() & fes[1]->getNature() & (Nature::SILHOUETTE | Nature::BORDER));

        FEdgeSmooth * fesh[2] = { dynamic_cast<FEdgeSmooth*>(fes[0]), dynamic_cast<FEdgeSmooth*>(fes[1]) };
        assert(fesh[0] != NULL && fesh[1] != NULL);

        // sign (viewvec * (AB ^ normal))

        bool frontFacing[2];
        for(int i=0;i<2;i++)
            frontFacing[i] = DiscreteRadialSign(fesh[i], _viewpoint);

        /*
      for(int i=0;i<2;i++)
    {
      Vec3r A = fesh[i]->vertexA()->point3d();
      Vec3r B = fesh[i]->vertexB()->point3d();
      Vec3r AB = B-A;
      AB.normalize();
      Vec3r m((A+B)/2.0);
      Vec3r crossP(AB^(fesh[i])->normal());
      crossP.normalize();
      Vec3r viewvector(m-_viewpoint);
      viewvector.normalize();
      frontFacing[i] = crossP * viewvector > 0;
      }*/

        if (frontFacing[0] != frontFacing[1])
        {
            // this point is a cusp

            ViewVertex* cusp = ioViewMap->InsertViewVertex(sv, newVEdges);
            if (cusp !=NULL)
            {
                cusp->setNature(cusp->getNature()|Nature::CUSP);
                ioViewMap->addDebugPoint(DebugPoint::CUSP, cusp->getPoint3D(), false, sv);
            }

            ViewEdge * nve = newVEdges.size() > 0 ? newVEdges.back() : NULL;
            assert(nve == NULL || nve->A() == NULL ||hasEdge(nve->A(),nve));
            assert(nve == NULL || nve->B() == NULL ||hasEdge(nve->B(),nve));


        }
    }
    ViewMap::viewedges_container& vedges = ioViewMap->ViewEdges();

    for(ViewMap::viewedges_container::iterator ve=newVEdges.begin(), veend=newVEdges.end();
        ve!=veend;
        ++ve){
        (*ve)->viewShape()->AddEdge(*ve);
        vedges.push_back(*ve);
        assert((*ve)->A() == NULL || hasEdge((*ve)->A(), *ve));
        assert((*ve)->B() == NULL || hasEdge((*ve)->B(), *ve));
    }


    // --------------------- cusps for mesh boundaries and silhouettes ---------------------



    for(vector<SVertex*>::iterator sit = ioViewMap->SVertices().begin(); sit != ioViewMap->SVertices().end();
        ++sit)
    {
        SVertex * sv = *sit;


        // only need to insert cusps in the interiors of ViewEdges
        if (sv->viewvertex() != NULL)
            continue;

        // retreive the two adjacent FEdges

        FEdge * fes[2];

        fes[0] = sv->fedge();
        fes[1] = fes[0]->vertexB() == sv ? fes[0]->nextEdge() : fes[0]->previousEdge();

        if (fes[0] == NULL || fes[1] == NULL)
            continue;

        // only consider mesh silhouettes and boundaries.
        if (!(fes[0]->getNature() & (Nature::SILHOUETTE | Nature::BORDER)) || fes[0]->isSmooth())
            continue;

        assert(fes[0]->getNature() & fes[1]->getNature() & (Nature::SILHOUETTE | Nature::BORDER));

        FEdgeSharp * fesh[2] = { dynamic_cast<FEdgeSharp*>(fes[0]), dynamic_cast<FEdgeSharp*>(fes[1]) };
        assert(fesh[0] != NULL && fesh[1] != NULL);

        // find the WVertex shared by both edges
        WVertex * wv = WEdge::CommonVertex(fesh[0]->edge(), fesh[1]->edge());
        //assert(wv == sv->GetSourceVertex()); // another way to get the same vertex

        set<WXFace*> oneRing;

        // collect the one-ring
        for(vector<WEdge*>::iterator wit = wv->GetEdges().begin(); wit != wv->GetEdges().end(); ++wit)
        {
            if ( (*wit)->GetaFace() != NULL)
                oneRing.insert( (WXFace*)(*wit)->GetaFace());
            if ( (*wit)->GetbFace() != NULL)
                oneRing.insert( (WXFace*)(*wit)->GetbFace());
        }
        /*
      bool oneRingIsConsistent = true;
      // check if everything in the one-ring is consistent
      if (_useConsistency && (fesh[0]->getNature() & Nature::SILHOUETTE))
    for(set<WXFace*>::iterator wfit = oneRing.begin(); wfit != oneRing.end(); ++wfit)
      if ( ! (*wfit)->consistent())
        {
          oneRingIsConsistent = false;
          break;
        }
*/


        // check if there are inconsistencies or overlaps in the neighborhood
        int cuspTest = IsCusp(sv, wv, fesh[0], fesh[1], oneRing, _viewpoint, _useConsistency);
        bool isCusp;

        //      assert( cuspTest != 1 || OneRingOcclusion(fesh[0], _viewpoint, _useConsistency)
        //              || OneRingOcclusion(fesh[1], _viewpoint, _useConsistency));


        if (!USE_SHEWCHUCK_TESTS &&  (fesh[0]->getNature() & Nature::SILHOUETTE))  // cusptest == -1
        {
            // use Markosian's algorithm for silhouettes with inconsistency, because it can kinda handle inconsistency
            //
            // Markosian et al. define a silhouette edge as front-facing if the adjacent face nearer to the camera is front-facing.
            // A vertex is a cusp if it connects a front-facing to a back-facing edge.

            bool facing[2];

            for(int i=0;i<2;i++)
            {
                //	      WEdge * edge = fesh[i]->edge();
                WFace * nearerFace = GetNearFace(fesh[i]->edge(),_viewpoint);
                fesh[i]->edge()->nearerFace = nearerFace;

                facing[i] = ((WXFace*)nearerFace)->front(_useConsistency);
            }
            //	  printf("facing: %s, %s\n", facing[0] ? "T" : "F", facing[1] ? "T":"F");
            isCusp = (facing[0] != facing[1]);
        }
        else
            if (cuspTest == 1)
                isCusp = true;
            else
                if (cuspTest == 0)
                    isCusp = false;
                else
                    isCusp = true; // being conservative


        if (isCusp)
        {
            // insert the cusp and split the ViewEdge
            ViewShape * shape = fes[0]->viewedge()->viewShape();

            NonTVertex * ntv = new NonTVertex(sv);

            ioViewMap->AddViewVertex(ntv);
            shape->AddVertex(ntv);
            if (cuspTest != -1)
                ntv->setNature( ntv->getNature() | Nature::CUSP);
            else
                ntv->setNature( ntv->getNature() | Nature::CUSP | Nature::AMBIG_CUSP);

            ViewEdge * vEdge = fes[0]->viewedge();
            ViewEdge * newViewEdge = NULL;

            shape->SplitEdge(ntv, vEdge, newViewEdge, false);

            if (newViewEdge != vEdge)
                ioViewMap->AddViewEdge(newViewEdge);

            ioViewMap->addDebugPoint(DebugPoint::CUSP,sv->getPoint3D(),false, sv);
        }
    }
}


void ViewMapBuilder::ComputeEdgesVisibility(ViewMap *ioViewMap, WingedEdge & we, 
                                            visibility_algo iAlgo,  Grid *iGrid, real epsilon)
{
    printf("Computing visibility with ray-casting\n");

    if((iAlgo == ray_casting ||
        iAlgo == ray_casting_fast ||
        iAlgo == ray_casting_very_fast ||
        iAlgo == region_based ||
        iAlgo == punch_out) && (NULL == iGrid))
    {
        cerr << "Error: can't cast ray, no grid defined" << endl;
        return;
    }

    switch(iAlgo)
    {
    case punch_out:
    case ray_casting:
        ComputeRayCastingVisibility(ioViewMap, iGrid, epsilon, iAlgo);
        break;
        //    case ray_casting:
        //      ComputeRayCastingVisibility(ioViewMap, iGrid, epsilon);
        //      break;
    case ray_casting_fast:
        ComputeFastRayCastingVisibility(ioViewMap, iGrid, epsilon);
        break;
    case ray_casting_very_fast:
        ComputeVeryFastRayCastingVisibility(ioViewMap, iGrid, epsilon);
        break;
    case region_based:
        //      ComputeRegionBasedVisibility(ioViewMap, we, iGrid, epsilon);
        break;
    default:
        break;
    }

    printf("Done\n");
}

void ViewMapBuilder::ComputeRayCastingVisibility(ViewMap *ioViewMap, Grid* iGrid, real epsilon,
                                                 visibility_algo iAlgo)
{
    vector<ViewEdge*>& vedges = ioViewMap->ViewEdges();
    bool progressBarDisplay = false;
    unsigned progressBarStep = 0;
    unsigned vEdgesSize = vedges.size();
    unsigned fEdgesSize = ioViewMap->FEdges().size();

    /*
  NodeShape * rtshape1 = new NodeShape();
  visDebugNode->AddChild(rtshape1);
  Material rtmat1;
  rtmat1.SetDiffuse(0,0,1,1);
  rtmat1.SetEmission(0,0,0,0);
  rtmat1.SetAmbient(0,0,0,0);

  rtshape1->SetMaterial(rtmat1);

  NodeShape * rtshape2 = new NodeShape();
  visDebugNode->AddChild(rtshape2);
  Material rtmat2;
  rtmat2.SetDiffuse(1,1,0,1);
  rtmat2.SetEmission(0,0,0,0);
  rtmat2.SetAmbient(0,0,0,0);

  rtshape2->SetMaterial(rtmat2);
  */

    if(_pProgressBar != NULL && fEdgesSize > gProgressBarMinSize) {
        unsigned progressBarSteps = min(gProgressBarMaxSteps, vEdgesSize);
        progressBarStep = vEdgesSize / progressBarSteps;
        _pProgressBar->reset();
        _pProgressBar->setLabelText("Computing Ray casting Visibility");
        _pProgressBar->setTotalSteps(progressBarSteps);
        _pProgressBar->setProgress(0);
        progressBarDisplay = true;
    }

    unsigned counter = progressBarStep;
    FEdge * fe, *festart;
    int nSamples = 0;
    vector<Polygon3r*> aFaces;
    Polygon3r *aFace = 0;
    unsigned tmpQI = 0;
    unsigned qiClasses[256];
    unsigned maxIndex, maxCard;
    unsigned qiMajority;
    static unsigned timestamp = 1;
    for(vector<ViewEdge*>::iterator ve=vedges.begin(), veend=vedges.end();
        ve!=veend;
        ve++)
    {
        festart = (*ve)->fedgeA();
        fe = (*ve)->fedgeA();
        qiMajority = 1;
        do {
            qiMajority++;
            fe = fe->nextEdge();
        } while (fe && fe != festart);
        //    qiMajority >>= 1;   // halve the number of possible votes. If N/2 votes agree, no point in getting more votes.

        // freestyle used to keep track of QI, but I'm disabling/not supporting
        // those features -Aaron

        tmpQI = 0;
        maxIndex = 0;
        maxCard = 0;
        nSamples = 0;
        fe = (*ve)->fedgeA();
        memset(qiClasses, 0, 256 * sizeof(*qiClasses));

        int visVotes = 0;
        int invisVotes = 0;

        set<ViewShape*> occluders;
        do
        {
            if((maxCard < qiMajority)) {

                if (iAlgo == punch_out)
                    tmpQI = ComputeRayCastingVisibilityPunchOut(fe, iGrid,
                                                                epsilon, occluders, &aFace, timestamp++);
                else
                    tmpQI = ComputeRayCastingVisibility(ioViewMap, fe, iGrid, epsilon,
                                                        occluders, &aFace, timestamp++);

                if (tmpQI != -1)
                {
                    if(tmpQI >= 256)
                        cerr << "Warning: too many occluding levels" << endl;

                    if (++qiClasses[tmpQI] > maxCard)
                    {
                        maxCard = qiClasses[tmpQI];
                        maxIndex = tmpQI;
                    }

                    if (tmpQI == 0)
                        visVotes ++;
                    else
                        invisVotes ++;

                    switch (tmpQI)
                    {
                    case 0:
                        ioViewMap->addDebugPoint(DebugPoint::RAY_TRACE_VISIBLE,fe->center3d());
                        break;

                    case 101:
                        ioViewMap->addDebugPoint(DebugPoint::INVISIBLE_BACK_FACE, fe->center3d(), true);
                        break;

                    case 123:
                        ioViewMap->addDebugPoint(DebugPoint::INVISIBLE_ONE_RING_OVERLAP, fe->center3d(), true);
                        break;

                    default:
                    case 100:
                        ioViewMap->addDebugPoint(DebugPoint::RAY_TRACE_INVISIBLE,fe->center3d(),true);
                        break;
                    }
                }
                else
                    FindOccludee(fe, iGrid, epsilon, &aFace, timestamp++);
            }

            if(aFace) {
                fe->SetaFace(*aFace); aFaces.push_back(aFace);
                fe->SetOccludeeEmpty(false);
            } else
                fe->SetOccludeeEmpty(true);

            ++nSamples; fe = fe->nextEdge();
        } while((maxCard < qiMajority) && (0!=fe) && (fe!=festart));

        // ViewEdge qi -- (*ve)->SetQI(maxIndex);

        // I don't care about estimating QI.
        if (visVotes > invisVotes)
            (*ve)->SetQI(0);
        else
            (*ve)->SetQI(100);

        if (invisVotes > 0 && visVotes > 0)
            (*ve)->MarkInconsistent();

        if (invisVotes == 0 && visVotes == 0){
            (*ve)->MarkAmbiguous();
        }

        (*ve)->visVotes = visVotes;
        (*ve)->invisVotes = invisVotes;

        //    assert(invisVotes + visVotes > 0);

        // occluders --
        for(set<ViewShape*>::iterator o=occluders.begin(),    oend=occluders.end(); o!=oend; ++o)
            (*ve)->AddOccluder((*o));

        // occludee --
        if(!aFaces.empty())
        {
            if(aFaces.size() <= (float)nSamples/2.f)
            {
                (*ve)->SetaShape(0);
            }
            else
            {
                vector<Polygon3r*>::iterator p = aFaces.begin();
                WFace * wface = (WFace*)((*p)->userdata);
                ViewShape *vshape = ioViewMap->viewShape(wface->GetVertex(0)->shape()->GetId());
                ++p;
                (*ve)->SetaShape(vshape);
            }
        }

        if(progressBarDisplay)
        {
            counter--;
            if (counter <= 0)
            {
                counter = progressBarStep;
                _pProgressBar->setProgress(_pProgressBar->getProgress() + 1);
            }
        }
        aFaces.clear();
    }
}

void ViewMapBuilder::ComputeFastRayCastingVisibility(ViewMap *ioViewMap, Grid* iGrid, real epsilon)
{
    vector<ViewEdge*>& vedges = ioViewMap->ViewEdges();
    bool progressBarDisplay = false;
    unsigned progressBarStep = 0;
    unsigned vEdgesSize = vedges.size();
    unsigned fEdgesSize = ioViewMap->FEdges().size();

    if(_pProgressBar != NULL && fEdgesSize > gProgressBarMinSize) {
        unsigned progressBarSteps = min(gProgressBarMaxSteps, vEdgesSize);
        progressBarStep = vEdgesSize / progressBarSteps;
        _pProgressBar->reset();
        _pProgressBar->setLabelText("Computing Ray casting Visibility");
        _pProgressBar->setTotalSteps(progressBarSteps);
        _pProgressBar->setProgress(0);
        progressBarDisplay = true;
    }

    unsigned counter = progressBarStep;
    FEdge * fe, *festart;
    unsigned nSamples = 0;
    vector<Polygon3r*> aFaces;
    Polygon3r *aFace = 0;
    unsigned tmpQI = 0;
    unsigned qiClasses[256];
    unsigned maxIndex, maxCard;
    unsigned qiMajority;
    static unsigned timestamp = 1;
    bool even_test;
    for(vector<ViewEdge*>::iterator ve=vedges.begin(), veend=vedges.end();
        ve!=veend;
        ve++)
    {
        festart = (*ve)->fedgeA();
        fe = (*ve)->fedgeA();
        qiMajority = 1;
        do {
            qiMajority++;
            fe = fe->nextEdge();
        } while (fe && fe != festart);
        if (qiMajority >= 4)
            qiMajority >>= 2;
        else
            qiMajority = 1;

        set<ViewShape*> occluders;

        even_test = true;
        maxIndex = 0;
        maxCard = 0;
        nSamples = 0;
        memset(qiClasses, 0, 256 * sizeof(*qiClasses));
        fe = (*ve)->fedgeA();
        do
        {
            if (even_test)
            {
                if((maxCard < qiMajority)) {
                    tmpQI = ComputeRayCastingVisibility(ioViewMap, fe, iGrid, epsilon, occluders, &aFace, timestamp++);

                    if(tmpQI >= 256)
                        cerr << "Warning: too many occluding levels" << endl;

                    if (++qiClasses[tmpQI] > maxCard) {
                        maxCard = qiClasses[tmpQI];
                        maxIndex = tmpQI;
                    }
                }
                else
                    FindOccludee(fe, iGrid, epsilon, &aFace, timestamp++);

                if(aFace)
                {
                    fe->SetaFace(*aFace);
                    aFaces.push_back(aFace);
                }
                ++nSamples;
                even_test = false;
            }
            else
                even_test = true;
            fe = fe->nextEdge();
        } while ((maxCard < qiMajority) && (0!=fe) && (fe!=festart));

        (*ve)->SetQI(maxIndex);

        if(!aFaces.empty())
        {
            if(aFaces.size() < nSamples / 2)
            {
                (*ve)->SetaShape(0);
            }
            else
            {
                vector<Polygon3r*>::iterator p = aFaces.begin();
                WFace * wface = (WFace*)((*p)->userdata);
                ViewShape *vshape = ioViewMap->viewShape(wface->GetVertex(0)->shape()->GetId());
                ++p;
                //        for(;
                //        p!=pend;
                //        ++p)
                //        {
                //          WFace *f = (WFace*)((*p)->userdata);
                //          ViewShape *vs = ioViewMap->viewShape(f->GetVertex(0)->shape()->GetId());
                //          if(vs != vshape)
                //          {
                //            sameShape = false;
                //            break;
                //          }
                //        }
                //        if(sameShape)
                (*ve)->SetaShape(vshape);
            }
        }

        //(*ve)->SetaFace(aFace);

        if(progressBarDisplay) {
            counter--;
            if (counter <= 0) {
                counter = progressBarStep;
                _pProgressBar->setProgress(_pProgressBar->getProgress() + 1);
            }
        }
        aFaces.clear();
    }
}

void ViewMapBuilder::ComputeVeryFastRayCastingVisibility(ViewMap *ioViewMap, Grid* iGrid, real epsilon)
{
    vector<ViewEdge*>& vedges = ioViewMap->ViewEdges();
    bool progressBarDisplay = false;
    unsigned progressBarStep = 0;
    unsigned vEdgesSize = vedges.size();
    unsigned fEdgesSize = ioViewMap->FEdges().size();

    if(_pProgressBar != NULL && fEdgesSize > gProgressBarMinSize) {
        unsigned progressBarSteps = min(gProgressBarMaxSteps, vEdgesSize);
        progressBarStep = vEdgesSize / progressBarSteps;
        _pProgressBar->reset();
        _pProgressBar->setLabelText("Computing Ray casting Visibility");
        _pProgressBar->setTotalSteps(progressBarSteps);
        _pProgressBar->setProgress(0);
        progressBarDisplay = true;
    }

    unsigned counter = progressBarStep;
    FEdge* fe;
    unsigned qi = 0;
    Polygon3r *aFace = 0;
    static unsigned timestamp = 1;
    for(vector<ViewEdge*>::iterator ve=vedges.begin(), veend=vedges.end();
        ve!=veend;
        ve++)
    {
        set<ViewShape*> occluders;

        fe = (*ve)->fedgeA();
        qi = ComputeRayCastingVisibility(ioViewMap, fe, iGrid, epsilon, occluders, &aFace, timestamp++);
        if(aFace)
        {
            fe->SetaFace(*aFace);
            WFace * wface = (WFace*)(aFace->userdata);
            ViewShape *vshape = ioViewMap->viewShape(wface->GetVertex(0)->shape()->GetId());
            (*ve)->SetaShape(vshape);
        }
        else
        {
            (*ve)->SetaShape(0);
        }

        (*ve)->SetQI(qi);

        if(progressBarDisplay) {
            counter--;
            if (counter <= 0) {
                counter = progressBarStep;
                _pProgressBar->setProgress(_pProgressBar->getProgress() + 1);
            }
        }
    }
}


void ViewMapBuilder::FindOccludee(FEdge *fe, Grid* iGrid, real epsilon, Polygon3r** oaPolygon, unsigned timestamp, 
                                  Vec3r& u, Vec3r& A, Vec3r& origin, Vec3r& edge, vector<WVertex*>& faceVertices)
{
    WFace *face = 0;

    if(fe->isSmooth())
    {
        FEdgeSmooth * fes = dynamic_cast<FEdgeSmooth*>(fe);
        face = (WFace*)fes->face();
    }

    // Aaron: this might need to be modified to also take into account SURFACE_INTERSECTIONS;
    // I think the current think will underestimate QI for invisible regions, not sure.  Invisible QI
    // isn't very important.

    OccludersSet occluders;
    WFace * oface;
    bool skipFace;

    WVertex::incoming_edge_iterator ie;
    OccludersSet::iterator p, pend;

    *oaPolygon = 0;
    if(((fe)->getNature() & Nature::SILHOUETTE) || ((fe)->getNature() & Nature::BORDER))
    {
        occluders.clear();
        // we cast a ray from A in the same direction but looking behind
        Vec3r v(-u[0],-u[1],-u[2]);
        iGrid->castInfiniteRay(A, v, occluders, timestamp);

        bool noIntersection = true;
        real mint=FLT_MAX;
        // we met some occluders, let us fill the aShape field
        // with the first intersected occluder
        for(p=occluders.begin(),pend=occluders.end();
            p!=pend;
            p++)
        {
            // check whether the edge and the polygon plane are coincident:
            //-------------------------------------------------------------
            //first let us compute the plane equation.
            oface = (WFace*)(*p)->userdata;
            Vec3r v1(((*p)->getVertices())[0]);
            Vec3r normal((*p)->getNormal());
            real d = -(v1 * normal);
            real t,t_u,t_v;

            if(0 != face)
            {
                skipFace = false;

                if(face == oface)
                    continue;

                if(faceVertices.empty())
                    continue;

                for(vector<WVertex*>::iterator fv=faceVertices.begin(), fvend=faceVertices.end();
                    fv!=fvend;
                    ++fv)
                {
                    if((*fv)->isBoundary())
                        continue;
                    WVertex::incoming_edge_iterator iebegin=(*fv)->incoming_edges_begin();
                    WVertex::incoming_edge_iterator ieend=(*fv)->incoming_edges_end();
                    for(ie=iebegin;ie!=ieend; ++ie)
                    {
                        if((*ie) == 0)
                            continue;

                        WFace * sface = (*ie)->GetbFace();
                        if(sface == oface)
                        {
                            skipFace = true;
                            break;
                        }
                    }
                    if(skipFace)
                        break;
                }
                if(skipFace)
                    continue;
            }
            else
            {
                if(GeomUtils::COINCIDENT == GeomUtils::intersectRayPlane(origin, edge, normal, d, t, epsilon))
                    continue;
            }
            if((*p)->rayIntersect(A, v, t,t_u,t_v))
            {
                if (fabs(v * normal) > 0.0001)
                    if ((t>0.0)) // && (t<1.0))
                    {
                        if (t<mint)
                        {
                            *oaPolygon = (*p);
                            mint = t;
                            noIntersection = false;
                            fe->SetOccludeeIntersection(Vec3r(A+t*v));
                        }
                    }
            }
        }

        if(noIntersection)
            *oaPolygon = 0;
    }
}

void ViewMapBuilder::FindOccludee(FEdge *fe, Grid* iGrid, real epsilon, Polygon3r** oaPolygon, unsigned timestamp)
{
    OccludersSet occluders;

    Vec3r A;
    Vec3r edge;
    Vec3r origin;
    A = Vec3r(((fe)->vertexA()->point3D() + (fe)->vertexB()->point3D())/2.0);
    edge = Vec3r((fe)->vertexB()->point3D()-(fe)->vertexA()->point3D());
    origin = Vec3r((fe)->vertexA()->point3D());
    Vec3r u(_viewpoint-A);
    u.normalize();
    if(A < iGrid->getOrigin())
        cerr << "Warning: point is out of the grid for fedge " << fe->getId().getFirst() << "-" << fe->getId().getSecond()
             << "in FindOccludee" << endl;

    vector<WVertex*> faceVertices;

    WFace *face = 0;
    if(fe->isSmooth()){
        FEdgeSmooth * fes = dynamic_cast<FEdgeSmooth*>(fe);
        face = (WFace*)fes->face();
    }
    if(0 != face)
        face->RetrieveVertexList(faceVertices);

    return FindOccludee(fe,iGrid, epsilon, oaPolygon, timestamp,
                        u, A, origin, edge, faceVertices);
}




int ViewMapBuilder::ComputeRayCastingVisibility(ViewMap *ioViewMap, FEdge *fe, Grid* iGrid, real epsilon, set<ViewShape*>& oOccluders,
                                                Polygon3r** oaPolygon, unsigned timestamp)
{
    // return -1 for "can't tell"

    OccludersSet occluders;
    int qi = 0;

    Vec3r center;
    Vec3r edge;
    Vec3r origin;

    bool ignoreOneOccluder = false;

    WXFace * face1 = dynamic_cast<WXFace*>(fe->getFace1());
    WXFace * face2 = dynamic_cast<WXFace*>(fe->getFace2());

    if (fe->isSmooth())
    {
        // check if this edge is on a back-face
        if (!(fe->getNature() & Nature::SILHOUETTE))
        {
            // may have two source faces
            if (face1 != NULL && !face1->front(_useConsistency))
                return 101;
            if (face2 != NULL && !face2->front(_useConsistency))
                return 101;
        }
        //      else
        //	if (!face1->front(_useConsistency)) // smooth silhouette on a back-face
        //	  ignoreOneOccluder = true;

        if (face1 != NULL && _useConsistency && !face1->consistent())
            return -1;

        if (face2 != NULL && _useConsistency && !face2->consistent())
            return -1;
    }
    else
    {
        WEdge * edge = ((FEdgeSharp*)fe)->edge();

        // mesh edge
        //        if (_useConsistency && edge->GetaFace() != NULL && !((WXFace*)edge->GetaFace())->consistent())
        //            return -1;
        //        if (_useConsistency && edge->GetbFace() != NULL && !((WXFace*)edge->GetbFace())->consistent())
        //            return -1;

        assert(dynamic_cast<FEdgeSharp*>(fe) != NULL);


        // don't do visibility on any fedge adjacent to a cusp.  they can be unreliable.
        // we could also tag cusps as unreliable.
        //        if (_useConsistency && fe->vertexA()->viewvertex() != NULL && (fe->vertexA()->viewvertex()->getNature() & Nature::CUSP))
        //            return -1;

        //        if (_useConsistency && fe->vertexB()->viewvertex() != NULL && (fe->vertexB()->viewvertex()->getNature() & Nature::CUSP))
        //            return -1;

        // New Heuristic
        // check if it's a back-facing contour
        if (NEW_SILHOUETTE_HEURISTIC && fe->getNature() & Nature::SILHOUETTE)
        {
            if (!DiscreteRadialSign( (FEdgeSmooth*)fe, _viewpoint) )
                return 175;

            if (face1 != NULL && !face1->front(_useConsistency))
                ignoreOneOccluder = true;
        }

        // markosian test
        if (!USE_SHEWCHUCK_TESTS)
        {

            if (fe->getNature() & Nature::SILHOUETTE)
            {
                real discriminant;
                WXFace * nearFace = (WXFace*)GetNearFace(edge, _viewpoint, &discriminant);

                // this test was not reliable in certain very high curvature regions (e.g., right sole, frame 25-27 of red)
                //	  if (!nearFace->front(_useConsistency)) // might not be reliable when inconsistent?
                //	    return 100;

                if ((discriminant > 1 || discriminant < -1) && !nearFace->front(false))
                {	    //	    ignoreOneOccluder = true;
                    DebugPoint * dp = ioViewMap->addDebugPoint(DebugPoint::INVISIBLE_ONE_RING_OVERLAP, fe->center3d(), true);
                    dp->debugString = new char[200];
                    sprintf(dp->debugString, "Discriminant: %f\n", discriminant);
                    return 123;
                }
            }
        }
        else
        {
            // check if this edge is occluded by an adjacent face
            int r = OneRingOcclusion((FEdgeSharp*)fe, _viewpoint, _useConsistency);
            if (r == 1)
                return 123;
            if (r == -1)
                return -1;
        }

    }

    center = fe->center3d();
    edge = Vec3r(fe->vertexB()->point3D() - fe->vertexA()->point3D());
    origin = Vec3r(fe->vertexA()->point3D());
    //
    //   // Is the edge outside the view frustum ?
    // Aaron: I think the above comment is wrong; this code checks if the point lies within the grid data structure
    Vec3r gridOrigin(iGrid->getOrigin());
    Vec3r gridExtremity(iGrid->getOrigin()+iGrid->gridSize());
    //  cerr << "grid origin: " << gridOrigin << "; gridExtremity: " << gridExtremity << endl;

    if( (center.x() < gridOrigin.x()) || (center.y() < gridOrigin.y()) || (center.z() < gridOrigin.z())
            ||(center.x() > gridExtremity.x()) || (center.y() > gridExtremity.y()) || (center.z() > gridExtremity.z())){
        cerr << "Warning: point is out of the grid for fedge " << fe->getId() << ": " << center << " in ComputeRayCastingVisibility" << endl;

        //return 0;
    }

    // Aaron: check against clipping planes
    if (!SilhouetteGeomEngine::IsInClippingPlanes(origin) ||
            !SilhouetteGeomEngine::IsInClippingPlanes(fe->vertexB()->point3D()))
        return 100;  // outside of clipping planes


    //  Vec3r A(fe->vertexA()->point2d());
    //  Vec3r B(fe->vertexB()->point2d());
    //  int viewport[4];
    //  SilhouetteGeomEngine::retrieveViewport(viewport);
    //  if( (A.x() < viewport[0]) || (A.x() > viewport[2]) || (A.y() < viewport[1]) || (A.y() > viewport[3])
    //    ||(B.x() < viewport[0]) || (B.x() > viewport[2]) || (B.y() < viewport[1]) || (B.y() > viewport[3])){
    //    cerr << "Warning: point is out of the grid for fedge " << fe->getId() << endl;
    //    //return 0;
    //  }

    Vec3r u(_viewpoint - center);
    real raylength = u.norm();
    u.normalize();
    //cout << "grid origin " << iGrid->getOrigin().x() << "," << iGrid->getOrigin().y() << "," << iGrid->getOrigin().z() << endl;
    //cout << "center " << center.x() << "," << center.y() << "," << center.z() << endl;

    //  printf("_viewpoint = %f %f %f\n", _viewpoint[0], _viewpoint[1], _viewpoint[2]);

    // the faces this intersection came from

    // for smooth edges: the face the edge came from
    WXFace *face = 0;

    if (fe->isSmooth() && (fe->getNature() & Nature::SILHOUETTE))
    {
        face = (WXFace*)(dynamic_cast<FEdgeSmooth*>(fe)->face());
        assert(face != NULL);
    }

    iGrid->castRay(center, Vec3r(_viewpoint), occluders, timestamp);

    vector<WVertex*> faceVertices;
    WVertex::incoming_edge_iterator ie;

    WXFace * oface;
    bool skipFace;
    OccludersSet::iterator p, pend;
    if(face)
        face->RetrieveVertexList(faceVertices);

    for(p=occluders.begin(),pend=occluders.end();
        p!=pend;
        p++)
    {
        // If we're dealing with an exact silhouette, check whether
        // we must take care of this occluder of not.
        // (Indeed, we don't consider the occluders that
        // share at least one vertex with the face containing
        // this edge).
        //-----------
        oface = (WXFace*)(*p)->userdata;
        Vec3r v1(((*p)->getVertices())[0]);
        Vec3r normal((*p)->getNormal());
        real d = -(v1 * normal);
        real t, t_u, t_v;

        if (oface == face || oface == face1 || oface == face2)
            continue;

        // Aaron: this is Freestyle's original heuristic for the backfacing-sil problem for smooth silhouettes
        if(face != NULL && fe->isSmooth() && (fe->getNature() & Nature::SILHOUETTE) && !NEW_SILHOUETTE_HEURISTIC)
        {

            skipFace = false;

            for(vector<WVertex*>::iterator fv=faceVertices.begin(), fvend=faceVertices.end();
                fv!=fvend;
                ++fv)
            {
                if((*fv)->isBoundary())
                    continue;

                WVertex::incoming_edge_iterator iebegin=(*fv)->incoming_edges_begin();
                WVertex::incoming_edge_iterator ieend=(*fv)->incoming_edges_end();
                for(ie=iebegin;ie!=ieend; ++ie)
                {
                    if((*ie) == 0)
                        continue;

                    WFace * sface = (*ie)->GetbFace();
                    //WFace * sfacea = (*ie)->GetaFace();
                    //if((sface == oface) || (sfacea == oface))
                    if(sface == oface)
                    {
                        skipFace = true;
                        break;
                    }
                }
                if(skipFace)
                    break;
            }
            if(skipFace)
                continue;
        }
        else
        {
            // check whether the edge and the polygon plane are coincident:
            //-------------------------------------------------------------
            //first let us compute the plane equation.

            if(GeomUtils::COINCIDENT == GeomUtils::intersectRayPlane(origin, edge, normal, d, t, epsilon))
                continue;
        }

        if((*p)->rayIntersect(center, u, t, t_u, t_v))
        {
            if (fabs(u * normal) > 0.0001)
                if ((t>0.0) && (t<raylength))
                {
                    // check if the face is inconsistent
                    if (_useConsistency && !oface->consistent()){
                        return -1;
                    }

                    if (ignoreOneOccluder)
                    {
                        ignoreOneOccluder = false;
                        continue;
                    }

                    WFace *f = (WFace*)((*p)->userdata);
                    ViewShape *vshape = _ViewMap->viewShape(f->GetVertex(0)->shape()->GetId());
                    oOccluders.insert(vshape);
                    ++qi;
                    if(!_EnableQI)
                        break;
                }
        }
    }

    // Find occludee
    FindOccludee(fe,iGrid, epsilon, oaPolygon, timestamp,
                 u, center, edge, origin, faceVertices);

    return qi;
}





int ViewMapBuilder::ComputeRayCastingVisibilityPunchOut(FEdge *fe, Grid* iGrid, real epsilon, set<ViewShape*>& oOccluders,
                                                        Polygon3r** oaPolygon, unsigned timestamp)
{
    OccludersSet occluders;
    int qi = 0;

    Vec3r center;
    Vec3r edge;
    Vec3r origin;

    center = fe->center3d();
    edge = Vec3r(fe->vertexB()->point3D() - fe->vertexA()->point3D());
    origin = Vec3r(fe->vertexA()->point3D());
    //
    //   // Is the edge outside the view frustum ?
    // Aaron: I think the above comment is wrong; this code checks if the point lies within the grid data structure
    Vec3r gridOrigin(iGrid->getOrigin());
    Vec3r gridExtremity(iGrid->getOrigin()+iGrid->gridSize());
    //  cerr << "grid origin: " << gridOrigin << "; gridExtremity: " << gridExtremity << endl;

    if( (center.x() < gridOrigin.x()) || (center.y() < gridOrigin.y()) || (center.z() < gridOrigin.z())
            ||(center.x() > gridExtremity.x()) || (center.y() > gridExtremity.y()) || (center.z() > gridExtremity.z())){
        cerr << "Warning: point is out of the grid for fedge " << fe->getId() << ": " << center << " in ComputeRayCastingVisibility" << endl;

        //return 0;
    }

    //  NodeShape * debugNode = new NodeShape();
    //  visDebugNode->AddChild(debugNode);

    NodeShape * igdg = new NodeShape();
    visDebugNode->AddChild(igdg);

    // Aaron: check against clipping planes
    if (!SilhouetteGeomEngine::IsInClippingPlanes(origin) ||
            !SilhouetteGeomEngine::IsInClippingPlanes(fe->vertexB()->point3D()))
        return 100;  // outside of clipping planes

    // check if this is an Inconsistent or Punch-Out point
    // (silhouette points are not considered inconsistent)
    if ( !fe->vertexA()->POProjected() && !fe->vertexB()->POProjected())
    {

        if (fe->getFace1() != NULL &&
                ( (fe->getNature() != Nature::SILHOUETTE && IsInconsistentPoint(fe->getFace1(),center)) ||
                  IsPunchOutPoint(fe->getFace1(),center)))
        {
            //      addDebugPoint(center, debugNode);
            return 100;
        }

        // check if this is an Inconsistent or Punch-Out point
        // (silhouette points are not considered inconsistent)
        if (fe->getFace2() != NULL &&
                ( (fe->getNature() != Nature::SILHOUETTE && IsInconsistentPoint(fe->getFace2(),center)) ||
                  IsPunchOutPoint(fe->getFace2(),center)))
        {
            //      addDebugPoint(center, debugNode);
            return 100;
        }
    }

    //  Vec3r A(fe->vertexA()->point2d());
    //  Vec3r B(fe->vertexB()->point2d());
    //  int viewport[4];
    //  SilhouetteGeomEngine::retrieveViewport(viewport);
    //  if( (A.x() < viewport[0]) || (A.x() > viewport[2]) || (A.y() < viewport[1]) || (A.y() > viewport[3])
    //    ||(B.x() < viewport[0]) || (B.x() > viewport[2]) || (B.y() < viewport[1]) || (B.y() > viewport[3])){
    //    cerr << "Warning: point is out of the grid for fedge " << fe->getId() << endl;
    //    //return 0;
    //  }

    Vec3r u(_viewpoint - center);
    real raylength = u.norm();
    u.normalize();
    //cout << "grid origin " << iGrid->getOrigin().x() << "," << iGrid->getOrigin().y() << "," << iGrid->getOrigin().z() << endl;
    //cout << "center " << center.x() << "," << center.y() << "," << center.z() << endl;

    //  printf("_viewpoint = %f %f %f\n", _viewpoint[0], _viewpoint[1], _viewpoint[2]);

    iGrid->castRay(center, Vec3r(_viewpoint), occluders, timestamp);

    // the faces this intersection came from
    WXFace * face1 = (WXFace*)fe->getFace1();
    WXFace * face2 = (WXFace*)fe->getFace2();

    // for smooth edges: the face the edge came from
    WXFace *face = 0;

    FEdgeIntersection * fei = dynamic_cast<FEdgeIntersection*>(fe);
    if (fei != NULL)
    {
        face1 = (WXFace*)fei->getFace1();
        face2 = (WXFace*)fei->getFace2();
    }
    else
    {
        if(fe->isSmooth())
        {
            FEdgeSmooth * fes = dynamic_cast<FEdgeSmooth*>(fe);
            face = (WXFace*)fes->face();
        }
    }



    // if this is a back-facing smooth silhouette, then we need to ignore occluders between the silhouette and the
    // corresponding near face, in order to handle their visibility correctly w.r.t. the "chopped-off" surface.
    // this is mainly in order to handle an issue that arises with surface intersections.
    real ignoreDist = -1;
    if (fe->getNature() == Nature::SILHOUETTE && fe->isSmooth())// && !((WXFace*)face)->front())
    {
        assert(face != NULL);
        ignoreDist = DistToPOFace(face,center);

        if (ignoreDist != -1)
        {
            assert(ignoreDist >= 0);

            /*
      LineRep * line = new LineRep(center, center + ignoreDist * u);
      line->SetWidth(2);
      line->ComputeBBox();
      igdg->AddRep(line);
      */
        }
    }

    vector<WVertex*> faceVertices;
    WVertex::incoming_edge_iterator ie;

    WXFace * oface;
    bool skipFace;
    OccludersSet::iterator p, pend;
    if(face)
        face->RetrieveVertexList(faceVertices);

    // find occluding points, ignoring occluder if it is:
    //    (a) inconsistent or punch-out
    //    (b) for silhouette visibility only: on triangles being punched-out by this source triangle
    //    (c) within distance "ignoreDist" (if ignoreDist > 0)
    //    (d) for PO-intersection-curve points: on source triangles
    //    (e) on a PO-intersction-curve surface

    for(p=occluders.begin(),pend=occluders.end();
        p!=pend;
        p++)
    {
        oface = (WXFace*)(*p)->userdata;  // possibly occluding face

        if (oface == NULL || oface == face1 || oface == face2)
            continue;

        // ignore occluders that are part of PO cuspregion geometry
        if (oface->sourcePOB() != NULL)
            continue;

        Vec3r v1(((*p)->getVertices())[0]);
        Vec3r normal((*p)->getNormal());
        real d = -(v1 * normal);
        real t, t_u, t_v;
        POType pt;

        bool geomResult = (*p)->rayIntersect(center, u, t, t_u, t_v) &&
                (fabs(u * normal) > 0.0001) && (t>0.0) && (t<raylength);

        if (!geomResult)
            continue;

        Vec3r oPoint = center + u * t;

        if (IsInconsistentPoint(oface, oPoint) ||
                IsPunchOutPointRay(oface, center) ||
                //IsPunchOutPoint(oface, oPoint) ||
                (fe->getNature() == Nature::SILHOUETTE && _ViewMap->facePOData(oface)->hasSourceFace(face1)) ||
                (ignoreDist != -1 && (oPoint - center).norm() < ignoreDist))
            continue;

        // check case (d) above
        if (face1 != NULL && face1->sourcePOB() != NULL &&
                (face1->sourcePOB()->sourceFace == oface || face1->sourcePOB()->myFace == oface))
            continue;

        if (face2 != NULL && face2->sourcePOB() != NULL &&
                (face2->sourcePOB()->sourceFace == oface || face2->sourcePOB()->myFace == oface))
            continue;

        // we have an occluder; process it
        WFace *f = (WFace*)((*p)->userdata);
        ViewShape *vshape = _ViewMap->viewShape(f->GetVertex(0)->shape()->GetId());
        oOccluders.insert(vshape);
        ++qi;
        if(!_EnableQI)
            break;


        // visualize the occlusion
        // uncomment this to put it back
        LineRep * line = new LineRep(center, oPoint);
        line->SetWidth(2);
        line->ComputeBBox();
        igdg->AddRep(line);

    }

    // Find occludee
    FindOccludee(fe,iGrid, epsilon, oaPolygon, timestamp,
                 u, center, edge, origin, faceVertices);


    // accurate QI computation is messed-up
    if (qi > 0)
        qi = 1;

    return qi;
}







struct less_SVertex2D : public binary_function<SVertex*, SVertex*, bool> 
{
    real epsilon;
    less_SVertex2D(real eps)
        : binary_function<SVertex*,SVertex*,bool>()
    {
        epsilon = eps;
    }
    bool operator()(SVertex* x, SVertex* y)
    {
        Vec3r A = x->point2D();
        Vec3r B = y->point2D();
        for(unsigned int i=0; i<3; i++)
        {
            if((fabs(A[i] - B[i])) < epsilon)
                continue;
            if(A[i] < B[i])
                return true;
            if(A[i] > B[i])
                return false;
        }

        return false;
    }
};

typedef Segment<FEdge*,Vec3r > segment;
typedef Intersection<segment> intersection;

struct less_Intersection : public binary_function<intersection*, intersection*, bool> 
{
    segment *edge;
    less_Intersection(segment *iEdge)
        : binary_function<intersection*,intersection*,bool>()
    {
        edge = iEdge;
    }
    bool operator()(intersection* x, intersection* y)
    {
        real tx = x->getParameter(edge);
        real ty = y->getParameter(edge);
        if(tx > ty)
            return true;
        return false;
    }
};


struct silhouette_binary_rule : public binary_rule<segment,segment>
{
    silhouette_binary_rule() : binary_rule<segment,segment>() {}
    virtual bool operator() (segment& s1, segment& s2)
    {
        FEdge * f1 = s1.edge();
        FEdge * f2 = s2.edge();

        if((!(((f1)->getNature() & Nature::SILHOUETTE) ||
              ((f1)->getNature() & Nature::BORDER))) &&
                (!(((f2)->getNature() & Nature::SILHOUETTE) ||
                   ((f2)->getNature() & Nature::BORDER))))
            return false;

        return true;
    }
};



// we only want image-space intersections between any pair of edges except for smooth-sharp pairs that intersect on the surface

struct silhouette_binary_rule_no_same_face : public binary_rule<segment,segment>
{
    silhouette_binary_rule_no_same_face() : binary_rule<segment,segment>() {}
    virtual bool operator() (segment& s1, segment& s2)
    {
        FEdge * f1 = s1.edge();
        FEdge * f2 = s2.edge();

        if (f1->isSmooth() && f2->isSmooth())
            return true;

        if (!f1->isSmooth() && !f2->isSmooth())
            return true;

        WXFace * face1[2] = { (WXFace*) f1->getFace1(), (WXFace*) f1->getFace2() };
        WXFace * face2[2] = { (WXFace*) f2->getFace1(), (WXFace*) f2->getFace2() };

        assert(face1[0] != NULL ||face1[1] != NULL);
        assert(face2[0] != NULL ||face2[1] != NULL);

        // check if the two fedges lie on the same face
        // that is, the curves intersect on the surface, not just in the 2D projection

        for(int i=0;i<2;i++)
            if (face1[i] != NULL)
                for(int j=0;j<2;j++)
                    if (face2[j] != NULL)
                        if (face1[i] == face2[j])
                            return false;  // don't compute intersections in this case



        return true;
    }
};


/*
// we only want image-space intersections between pairs of edges if
// (a) at least one of them is a silhouette, crease, or border
// (b) the intersection is not on the surface (those will be detected separately)

struct silhouette_binary_rule_no_same_face : public binary_rule<segment,segment>
{
  silhouette_binary_rule_no_same_face() : binary_rule<segment,segment>() {}
  virtual bool operator() (segment& s1, segment& s2)
  {
    FEdge * f1 = s1.edge();
    FEdge * f2 = s2.edge();

    Nature::EdgeNature n1 = f1->getNature();
    Nature::EdgeNature n2 = f2->getNature();

    if (! (n1 & Nature::SILHOUETTE || n1 & Nature::BORDER || n1 & Nature::CREASE ||
       n2 & Nature::SILHOUETTE || n2 & Nature::BORDER || n2 & Nature::CREASE))
      return false;


    WXFace * face1[2] = { (WXFace*) f1->getFace1(), (WXFace*) f1->getFace2() };
    WXFace * face2[2] = { (WXFace*) f2->getFace1(), (WXFace*) f2->getFace2() };
    
    assert(face1[0] != NULL ||face1[1] != NULL);
    assert(face2[0] != NULL ||face2[1] != NULL);

    // check if this is a PO surface intersection and its source silhouette
    POBoundaryEdge * posrc[2] = {NULL,NULL};
    WFace * otherface = NULL;
    if ( (n1 & Nature::PO_SURFACE_INTERSECTION) && (n2 & Nature::SILHOUETTE) )
      {
    posrc[0] = face1[0]->sourcePOB();
    posrc[1] = face1[1]->sourcePOB();
    otherface = face2[0];
      }
    else
    if ( (n2 & Nature::PO_SURFACE_INTERSECTION) && (n1 & Nature::SILHOUETTE) )
      {
    posrc[0] = face2[0]->sourcePOB();
    posrc[1] = face2[1]->sourcePOB();
    otherface = face1[0];
      }

    if (posrc[0] != NULL && otherface != NULL &&
    (posrc[0]->sourceFace == otherface || shareEdge(posrc[0]->sourceFace, otherface)))
      return false;
    if (posrc[1] != NULL && otherface != NULL &&
    (posrc[1]->sourceFace == otherface || shareEdge(posrc[1]->sourceFace, otherface)))
      return false;

    // if both are smooth, then we want the intersection
    if (f1->isSmooth() && f2->isSmooth())
      return true;

    // this case should probably be handled separately on the surface since it might be more stable
    if (!f1->isSmooth() && !f2->isSmooth())
      return true;


    // check if the two fedges lie on the same face
    // that is, the curves intersect on the surface, not just in the 2D projection
    
    for(int i=0;i<2;i++)
      if (face1[i] != NULL)
    for(int j=0;j<2;j++)
      if (face2[j] != NULL)
          if (face1[i] == face2[j])
        return false;  // don't compute intersections in this case



    return true;
  };
};
*/


void ViewMapBuilder::ComputeCurveIntersections(ViewMap *ioViewMap, visibility_algo iAlgo, real epsilon)
{
    printf("Computing image-space intersections\n");

    vector<SVertex*>& svertices = ioViewMap->SVertices();
    bool progressBarDisplay = false;
    unsigned sVerticesSize = svertices.size();
    unsigned fEdgesSize = ioViewMap->FEdges().size();
    //  ViewMap::fedges_container& fedges = ioViewMap->FEdges();
    //  for(ViewMap::fedges_container::const_iterator f=fedges.begin(), end=fedges.end();
    //  f!=end;
    //  ++f){
    //    cout << (*f)->aMaterialIndex() << "-" << (*f)->bMaterialIndex() << endl;
    //  }

    /*
  NodeShape * intNode1 = new NodeShape();
  visDebugNode->AddChild(intNode1);
  Material intmat1;
  intmat1.SetDiffuse(0,1,0,1);
  intmat1.SetEmission(0,0,0,0);
  intmat1.SetAmbient(0,0,0,0);

  intNode1->SetMaterial(intmat1);


  NodeShape * intNode2 = new NodeShape();
  visDebugNode->AddChild(intNode2);
  Material intmat2;
  intmat2.SetDiffuse(0,.5,0,1);
  intmat2.SetEmission(0,0,0,0);
  intmat2.SetAmbient(0,0,0,0);

  intNode2->SetMaterial(intmat2);
  */


    //------------------------- setup progress bar --------------------------------------
    
    unsigned progressBarStep = 0;

    if(_pProgressBar != NULL && fEdgesSize > gProgressBarMinSize) {
        unsigned progressBarSteps = min(gProgressBarMaxSteps, sVerticesSize);
        progressBarStep = sVerticesSize / progressBarSteps;
        _pProgressBar->reset();
        _pProgressBar->setLabelText("Computing Sweep Line Intersections");
        _pProgressBar->setTotalSteps(progressBarSteps);
        _pProgressBar->setProgress(0);
        progressBarDisplay = true;
    }

    unsigned counter = progressBarStep;

#if 1

    // ------------------------ compute sweep line (all-pairs 2D intersections) ---------
    printf("\t sweep line\n");

    sort(svertices.begin(), svertices.end(), less_SVertex2D(0));//epsilon));

    SweepLine<FEdge*,Vec3r> SL;

    vector<segment* > segments;

    vector<FEdge*>::iterator fe,fend;

    for(fe=ioViewMap->FEdges().begin(), fend=ioViewMap->FEdges().end(); fe!=fend; fe++)
    {
        segment * s = new segment((*fe), (*fe)->vertexA()->point2D(), (*fe)->vertexB()->point2D());
        (*fe)->userdata = s;
        segments.push_back(s);
    }

    const real SLepsilon =0.0;

    vector<segment*> vsegments;
    for(vector<SVertex*>::iterator sv=svertices.begin(),svend=svertices.end();
        sv!=svend;
        sv++)
    {
        const vector<FEdge*>& vedges = (*sv)->fedges();

        for(vector<FEdge*>::const_iterator sve=vedges.begin(), sveend=vedges.end();
            sve!=sveend;
            sve++)
        {
            assert( (*sve)->userdata != NULL );
            vsegments.push_back((segment*)((*sve)->userdata));
        }

        Vec3r evt((*sv)->point2D());

        // use a binary rule that only detects certain intersections (see comments for it above)
        silhouette_binary_rule_no_same_face sbr;
        SL.process(evt, vsegments, _viewpoint, sbr);

        if(progressBarDisplay) {
            counter--;
            if (counter <= 0) {
                counter = progressBarStep;
                _pProgressBar->setProgress(_pProgressBar->getProgress() + 1);
            }
        }
        vsegments.clear();
    }

    // retrieve the intersected edges:
    set<segment* >& iedges = SL.intersectedEdges();
    // retrieve the intersections:
    vector<intersection*>& intersections = SL.intersections();
#else
    // ----------------- Brute force intersection test O(n^2) ----------------------
    printf("\t brute force\n");

    vector<segment* > segments;

    vector<FEdge*>::iterator fe,fend;

    for(fe=ioViewMap->FEdges().begin(), fend=ioViewMap->FEdges().end(); fe!=fend; fe++)
    {
        segment * s = new segment((*fe), (*fe)->vertexA()->point2D(), (*fe)->vertexB()->point2D());
        (*fe)->userdata = s;
        segments.push_back(s);
    }

    // the intersected edges:
    set<segment*> iedges;
    // the intersections:
    vector<intersection*> intersections;

    // use a binary rule that only detects certain intersections (see comments for it above)
    silhouette_binary_rule_no_same_face sbr;

    for(vector<segment*>::iterator s1_it=segments.begin(); s1_it!=segments.end(); s1_it++)
    {
        segment* S = (*s1_it);

        real t,u;
        Vec3r CP;
        Vec2r v0, v1, v2, v3;
        if(true == S->order())
        {
            v0[0] = ((*S)[0])[0];
            v0[1] = ((*S)[0])[1];
            v1[0] = ((*S)[1])[0];
            v1[1] = ((*S)[1])[1];
        }
        else
        {
            v1[0] = ((*S)[0])[0];
            v1[1] = ((*S)[0])[1];
            v0[0] = ((*S)[1])[0];
            v0[1] = ((*S)[1])[1];
        }
        for(vector<segment*>::iterator s2_it=(s1_it+1); s2_it!=segments.end(); s2_it++)
        {
            segment* currentS = (*s2_it);
            if(true != sbr(*S, *currentS))
                continue;

            if(S->CommonVertex(*currentS, CP))
                continue; // the two edges have a common vertex->no need to check

            if(true == currentS->order())
            {
                v2[0] = ((*currentS)[0])[0];
                v2[1] = ((*currentS)[0])[1];
                v3[0] = ((*currentS)[1])[0];
                v3[1] = ((*currentS)[1])[1];
            }
            else
            {
                v3[0] = ((*currentS)[0])[0];
                v3[1] = ((*currentS)[0])[1];
                v2[0] = ((*currentS)[1])[0];
                v2[1] = ((*currentS)[1])[1];
            }

            if(GeomUtils::intersect2dSeg2dSegParametric(v0, v1, v2, v3, t, u) == GeomUtils::DO_INTERSECT){
                // create the intersection
                Intersection<segment> * inter = new Intersection<segment>(S,t,currentS,u);
                // add it to the intersections list
                intersections.push_back(inter);
                // add this intersection to the first edge intersections list
                S->AddIntersection(inter);
                // add this intersection to the second edge intersections list
                currentS->AddIntersection(inter);

                iedges.insert(S);
                iedges.insert(currentS);
            }
        }
    }
#endif


    // list containing the new edges resulting from splitting operations.
    vector<FEdge*> newEdges;

    // -------------------------- create T-Vertices for image-space intersections -------------

    int id=0;
    // create a view vertex for each intersection and linked this one
    // with the intersection object
    for(vector<intersection*>::iterator i=intersections.begin(); i!=intersections.end(); i++)
    {
        FEdge *fA = (*i)->EdgeA->edge();
        FEdge *fB = (*i)->EdgeB->edge();

        Vec3r A1 = fA->vertexA()->point3D();
        Vec3r A2 = fA->vertexB()->point3D();
        Vec3r B1 = fB->vertexA()->point3D();
        Vec3r B2 = fB->vertexB()->point3D();

        /*
      Vec3r a1 = fA->vertexA()->point2D();
      Vec3r a2 = fA->vertexB()->point2D();
      Vec3r b1 = fB->vertexA()->point2D();
      Vec3r b2 = fB->vertexB()->point2D();

      real ta = (*i)->tA;
      real tb = (*i)->tB;
      if ( ta < -epsilon || ta > 1+epsilon)
    cerr << "Warning: intersection out of range for edge " << fA->vertexA()->getId() << " - " << fA->vertexB()->getId() << endl;

      if(tb < -epsilon || tb > 1+epsilon)
    cerr << "Warning: intersection out of range for edge " << fB->vertexA()->getId() << " - " << fB->vertexB()->getId() << endl;

      real Ta = SilhouetteGeomEngine::ImageToWorldParameter(fA, ta);
      real Tb = SilhouetteGeomEngine::ImageToWorldParameter(fB, tb);
      TVertex * tvertex = ioViewMap->CreateTVertex(Vec3r(A1 + Ta*(A2-A1)), Vec3r(a1 + ta*(a2-a1)), fA,
                           Vec3r(B1 + Tb*(B2-B1)), Vec3r(b1 + tb*(b2-b1)), fB, id);
      addDebugPoint(A1 + Ta*(A2-A1), intNode1);
      addDebugPoint(B1 + Tb*(B2-B1), intNode1);
      */


        real Ta = (*i)->tA;
        real Tb = (*i)->tB;

        real eps2 = .01;
        assert(Ta>= -eps2 && Ta<=1+eps2 && Tb>=-eps2 && Tb<=1+eps2);

        Vec3r intA = A1 + Ta*(A2-A1);
        Vec3r intB = B1 + Tb*(B2-B1);

        TVertex * tvertex = ioViewMap->CreateTVertex(intA, SilhouetteGeomEngine::WorldToImage(intA), fA,
                                                     intB, SilhouetteGeomEngine::WorldToImage(intB), fB, id);

        (*i)->userdata = tvertex;
        ++id;
    }



    // -------------------------- do smooth-sharp intersections on the surface --------

    // check that all vvertices on edges are non-t-vertices at this point
    //  for(vector<SVertex*>::iterator s = ioViewMap->SVertices().begin(); s != ioViewMap->SVertices().end();++s)
    //      assert( (*s)->GetSourceEdge() == NULL ||  (*s)->viewvertex() == NULL ||
    //              dynamic_cast<NonTVertex*>((*s)->viewvertex()) != NULL);

    // because we haven't yet done any splits, there should be at most one sharp edge per mesh edge
    // (but what about boundaries next to silhouettes? not a very important case?)
    map<WEdge*, FEdgeSharp*>  edgemap;

    // create edgemap
    for(vector<FEdge*>::iterator feit = ioViewMap->FEdges().begin();
        feit != ioViewMap->FEdges().end(); ++feit)
        if (! (*feit)->isSmooth())
        {
            assert(dynamic_cast<FEdgeSharp*>(*feit) != NULL);
            FEdgeSharp * fes = (FEdgeSharp*)*feit;

            assert(edgemap.find(fes->edge()) == edgemap.end());
            edgemap.insert(pair<WEdge*,FEdgeSharp*>(fes->edge(), fes));
        }

    // find SVertex-fedge pairs, such that
    //  * the SVertex lies on a mesh edge (and thus is from a smooth curve)
    //  * the fedge is sharp
    //  * the two come from the same edge

    vector<SVertex*> newSVertices;

    for(vector<SVertex*>::iterator svit = ioViewMap->SVertices().begin();
        svit != ioViewMap->SVertices().end(); ++ svit)
    {
        SVertex * svSmooth = *svit;

        WEdge * meshEdge = svSmooth->GetSourceEdge();

        if (meshEdge == NULL) // only smooth curves have sources
            continue;

        map<WEdge*,FEdgeSharp*>::iterator it = edgemap.find(meshEdge);

        if (it == edgemap.end())
            continue;

        assert(svSmooth->viewvertex() == NULL || dynamic_cast<NonTVertex*>(svSmooth->viewvertex())!=NULL);

        FEdgeSharp * feSharp = (*it).second;

        Vec3r P3d = svSmooth->getPoint3D();
        Vec2f P2d2 = svSmooth->getPoint2D();
        Vec3r P2d(P2d2.x(), P2d2.y(), svSmooth->getProjectedZ());

        real t_sm = -1;
        real t_sh = GeomUtils::segmentParam(feSharp->vertexA()->getPoint2D(),
                                            feSharp->vertexB()->getPoint2D(), P2d2);

        segment * segsm = NULL;
        segment * segsh = (segment*)feSharp->userdata;
        Intersection<segment> * inter = new Intersection<segment>(segsm, t_sm, segsh, t_sh);

        intersections.push_back(inter);
        iedges.insert(segsh);
        segsh->AddIntersection(inter);

        ioViewMap->addDebugPoint(DebugPoint::INTERSECTION_2D_ON_SURFACE, P3d, false, svSmooth);

        SVertex * svSharp = feSharp->shape()->CreateSVertex(P3d,P2d,feSharp->vertexA()->getId());
        svSharp->SetSourceEdge(svSmooth->GetSourceEdge());
        // smooth vertex is put in "front" position

        ViewVertex * oldVV = svSmooth->viewvertex();


        TVertex * tvertex = new TVertex(svSmooth, svSharp);

        tvertex->SetId(id);
        tvertex->SetSameFace(true);

        ioViewMap->AddViewVertex(tvertex);
        newSVertices.push_back(svSharp);

        FEdge * feSmooth = svSmooth->fedges()[0]; // any edge on the smooth chain

        if (oldVV != NULL)
        {
            // if we're at a boundary, replace the viewvertex
            assert(dynamic_cast<NonTVertex*>(oldVV) != NULL);
            NonTVertex * ntv = (NonTVertex*)oldVV;
            assert(ntv->viewedges().size() == 1);
            assert(ntv->viewedges()[0].first == feSmooth->viewedge());

            ioViewMap->RemoveVertex(ntv);

            ViewEdge * viewedge = feSmooth->viewedge();
            assert (viewedge->A() == ntv || viewedge->B() == ntv);
            if (viewedge->A() == ntv)
            {
                viewedge->SetA(tvertex);
                //              tvertex->SetFrontEdgeA(NULL, true);
                tvertex->SetFrontEdgeB(viewedge, false);
            }
            else
            {
                viewedge->SetB(tvertex);
                tvertex->SetFrontEdgeA(viewedge, true);
                //              tvertex->SetFrontEdgeB(NULL, true);
            }
        }
        else
        {
            // split the smooth chain into two chains
            ViewEdge * newVEdge;
            feSmooth->viewedge()->viewShape()->SplitEdge(tvertex, feSmooth->viewedge(), newVEdge, true );
            if (newVEdge != feSmooth->viewedge())
                ioViewMap->AddViewEdge(newVEdge);
        }

        feSharp->viewedge()->viewShape()->AddVertex(tvertex);
        if (feSharp->viewedge()->viewShape() != feSmooth->viewedge()->viewShape())
            feSmooth->viewedge()->viewShape()->AddVertex(tvertex);

        svSmooth->SetViewVertex(tvertex);
        svSharp->SetViewVertex(tvertex);

        inter->userdata = tvertex;
        ++id;
    }

    // do this outside the above loop to avoid messing it up
    for(vector<SVertex*>::iterator sit = newSVertices.begin(); sit != newSVertices.end(); ++sit)
        ioViewMap->AddSVertex(*sit);


    edgemap.clear();


    progressBarStep = 0;

    if(progressBarDisplay) {
        unsigned iEdgesSize = iedges.size();
        unsigned progressBarSteps = min(gProgressBarMaxSteps, iEdgesSize);
        progressBarStep = iEdgesSize / progressBarSteps;
        _pProgressBar->reset();
        _pProgressBar->setLabelText("Splitting intersected edges");
        _pProgressBar->setTotalSteps(progressBarSteps);
        _pProgressBar->setProgress(0);
    }

    counter = progressBarStep;


    // -------- for each edge that has at least one intersection, find its intersections and split it ----

    // TODO: this should only be for edges that are not begin intersected by a sharp edge
    // e.g., when intersecting a mesh silhouette with a surface intersection, different code should
    // split the silhouette

    vector<TVertex*> edgeVVertices;
    vector<ViewEdge*> newVEdges;
    set<segment* >::iterator s, send;
    for(s=iedges.begin(),send=iedges.end();
        s!=send;
        s++)
    {
        edgeVVertices.clear();
        newEdges.clear();
        newVEdges.clear();

        FEdge* fedge = (*s)->edge();
        ViewEdge *vEdge = fedge->viewedge();
        ViewShape *shape = vEdge->viewShape();

        vector<intersection*>& eIntersections = (*s)->intersections();

        // we first need to sort these intersections from farther to closer to A
        sort(eIntersections.begin(), eIntersections.end(), less_Intersection(*s));

        bool POProjected = fedge->vertexA()->POProjected() || fedge->vertexB()->POProjected();

        for(vector<intersection*>::iterator i=eIntersections.begin();
            i!=eIntersections.end();
            i++)
        {
            // this curve should be split if:
            //  - the rear curve is always split
            //  - it's a same-face intersection (and it hasn't been split above in the smooth-sharp case)
            //  - the front curve is split if the rear one is a silhouette that might punch out the near one
            // I don't not split all curves, since there seems to be a problem with the chaining operators
            //  that isn't able to fix all splits.
            TVertex * tvert = (TVertex*)(*i)->userdata;

            bool isBack = tvert->backSVertex()->getId() == fedge->vertexA()->getId();

            assert(isBack || tvert->frontSVertex()->getId() == fedge->vertexA()->getId());

            /*
          // skip the case of a smooth edge split by a sharp edge, because it's handled above
          if (tvert->sameFace() && !isBack)
          {
              real t = (*i)->getParameter((segment*)fedge->userdata);

              // make sure we're catching the case I think we are
              assert(fedge->isSmooth() && (t == 0 || t == 1));

              // could also just not add this intersection to the edge in the first place
              continue;
          }
*/
            bool split = true;
            //	  bool split = tvert->sameFace() || isBack;

            if (!split && iAlgo == punch_out) // this is the front curve. check if the rear curve punches out this triangle
            {
                FEdge *rearEdge = (fedge == (*i)->EdgeA->edge() ? (*i)->EdgeB->edge() : (*i)->EdgeA->edge());
                assert(tvert->frontSVertex()->getId() == fedge->vertexA()->getId());

                split = (rearEdge->getNature() & Nature::SILHOUETTE) &&
                        (_ViewMap->facePOData(rearEdge->getFace1())->hasTargetFace(fedge->getFace1()) ||
                         _ViewMap->facePOData(rearEdge->getFace1())->hasTargetFace(fedge->getFace2()));
            }

            if (split)
            {
                edgeVVertices.push_back(tvert);
                SVertex * sv = isBack ? tvert->backSVertex() : tvert->frontSVertex();
                SVertex * svother = isBack ? tvert->frontSVertex() : tvert->backSVertex();
                ioViewMap->addDebugPoint(DebugPoint::INTERSECTION_2D, sv->getPoint3D(), svother->getPoint3D(),
                                         sv, false, svother);
                //	    addDebugPoint(isBack ? tvert->backSVertex()->getPoint3D() :
                //			  tvert->frontSVertex()->getPoint3D(), intNode1);

                if (POProjected)
                {
                    tvert->frontSVertex()->SetPOProjected();
                    tvert->backSVertex()->SetPOProjected();
                }
            }
        }

        shape->SplitEdge(fedge, edgeVVertices, ioViewMap->FEdges(), ioViewMap->ViewEdges());

        /*
    printf("Edge split: %08X, %d splits\n", fedge, edgeVVertices.size());
    for(int i=0;i<edgeVVertices.size();i++)
      {
    TVertex * tvertex = edgeVVertices[i];
    if (tvertex->sameFace())
        printf("split tvert: %08X, front: %08X, %08X, back: %08X, %08X\n",
           tvertex,
           tvertex->frontEdgeA().first, tvertex->frontEdgeB().first,
           tvertex->backEdgeA().first, tvertex->backEdgeB().first);
      }
    */

        if(progressBarDisplay) {
            counter--;
            if (counter <= 0) {
                counter = progressBarStep;
                _pProgressBar->setProgress(_pProgressBar->getProgress() + 1);
            }
        }
    }

    // ------------------------ clean up data-structures ------------------------------------

    // reset userdata:
    for(fe=ioViewMap->FEdges().begin(), fend=ioViewMap->FEdges().end();
        fe!=fend;
        fe++)
        (*fe)->userdata = NULL;


    // delete segments
    for(vector<segment*>::iterator s=segments.begin(); s!= segments.end(); s++)
        delete *s;
    segments.clear();

    ViewMap::viewvertices_container& vvertices = ioViewMap->ViewVertices();
    for(ViewMap::viewvertices_container::iterator vv=vvertices.begin(), vvend=vvertices.end();
        vv!=vvend;
        ++vv)
    {
        if((*vv)->getNature() == Nature::T_VERTEX)
        {
            TVertex *tvertex = (TVertex*)(*vv);
            cout << "TVertex " << tvertex->getId() << " has :" << endl;
            cout << "FrontEdgeA: " << tvertex->frontEdgeA().first << endl;
            cout << "FrontEdgeB: " << tvertex->frontEdgeB().first << endl;
            cout << "BackEdgeA: " << tvertex->backEdgeA().first << endl;
            cout << "BackEdgeB: " << tvertex->backEdgeB().first << endl << endl;
        }
    }
}


void ViewMapBuilder::ResetGroupingData(WingedEdge& we)
{
    for(vector<WShape*>::iterator it = we.getWShapes().begin(); it != we.getWShapes().end(); ++it)
    {
        WShape * w= *it;

        for(vector<WVertex*>::iterator vit=w->GetVertexList().begin();
            vit != w->GetVertexList().end(); vit++)
        {
            assert(dynamic_cast<WXVertex*>(*vit) != NULL);
        }

        for(vector<WEdge*>::iterator eit=w->GetEdgeList().begin();
            eit != w->GetEdgeList().end(); eit++)
        {
            assert(dynamic_cast<WXEdge*>(*eit) != NULL);
            ((WXEdge*)(*eit))->SetRegionEdge(true);
        }

    }
}

void ViewMapBuilder::PropagateVisibilty(ViewMap *ioViewMap)
{
    printf("Propagating visibility to ambiguous edges\n");

    // gather all chains that do not have any visibility votes
    set<ViewEdge*> ambiguousEdges;

    // mark chains with short image-space extent as ambiguous
    for(vector<ViewEdge*>::iterator vit = ioViewMap->ViewEdges().begin(); vit != ioViewMap->ViewEdges().end(); ++vit){
        if (_graftThreshold > 0)
            if (ArcLength2D(*vit, 2*_graftThreshold) < _graftThreshold)
                (*vit)->MarkAmbiguous();

        // Spurious cusps heuristic
        NonTVertex * vertA = dynamic_cast<NonTVertex*>((*vit)->A());
        NonTVertex * vertB = dynamic_cast<NonTVertex*>((*vit)->B());
        if (vertA != NULL && vertB != NULL)
            if((vertA->getNature() & Nature::CUSP) && (vertB->getNature() & Nature::CUSP)
                    && ((*vit)->fedgeA()->nextEdge() == NULL)){
                (*vit)->MarkAmbiguous();
            }
    }

    for(vector<ViewEdge*>::iterator vit = ioViewMap->ViewEdges().begin(); vit != ioViewMap->ViewEdges().end(); ++vit)
        if ( (*vit)->ambiguousVisibility())
            ambiguousEdges.insert(*vit);

    bool changed = true;
    while (ambiguousEdges.size() > 0 && changed)
    {
        changed = false;

        // copy over the current list of ambiguous edges so that we can iterate over it
        vector<ViewEdge*> edgeList;
        for(set<ViewEdge*>::iterator it = ambiguousEdges.begin(); it!= ambiguousEdges.end(); ++it)
            edgeList.push_back(*it);
        //      copy(ambiguousEdges.begin(), ambiguousEdges.end(), edgeList.begin());

        for(vector<ViewEdge*>::iterator vit = edgeList.begin(); vit != edgeList.end(); ++vit)
        {
            // see if we can resolve the ambiguity
            ViewEdge * edge = *vit;

            if (!edge->ambiguousVisibility())
                continue;

            // Spurious cusps heuristic
            NonTVertex * vertA = dynamic_cast<NonTVertex*>(edge->A());
            NonTVertex * vertB = dynamic_cast<NonTVertex*>(edge->B());
            if (vertA != NULL && vertB != NULL)
                if(vertA->getNature() & Nature::CUSP && vertB->getNature() & Nature::CUSP){
                    ViewEdge * mateA = NULL;
                    ViewEdge * mateB = NULL;
                    if(vertA->viewedges().size() == 2)
                        mateA = (vertA->viewedges()[0].first != edge ? vertA->viewedges()[0].first : vertA->viewedges()[1].first);
                    if(vertB->viewedges().size() == 2)
                        mateB = (vertB->viewedges()[0].first != edge ? vertB->viewedges()[0].first : vertB->viewedges()[1].first);
                    if(mateA != NULL && mateB != NULL)
                        if(mateA->qi() == 0 && mateB->qi() == 0){
                            edge->SetQI(0);
                            ambiguousEdges.erase(ambiguousEdges.find(edge));
                            edge->FixAmbiguous();
                            changed = true;
                            break;
                        }
                }

            // Fix visibility of tiny bits
            real arcLength = ArcLength2D(edge,FLT_MAX);
            if(arcLength<=0.01){
                TVertex * vertA = dynamic_cast<TVertex*>(edge->A());
                TVertex * vertB = dynamic_cast<TVertex*>(edge->B());
                if (vertA != NULL && vertB != NULL){
                    ViewEdge * mateA = vertA->mate(edge);
                    ViewEdge * mateB = vertB->mate(edge);
                    if(mateA && mateB && mateA->qi()==0 && mateB->qi()==0){
                        edge->SetQI(0);
                        ambiguousEdges.erase(ambiguousEdges.find(edge));
                        edge->FixAmbiguous();
                        changed = true;
                        break;
                    }
                }
            }

            ViewVertex * v[2] = {edge->A(), edge->B() };

            for(int i=0;i<2;i++)
            {
                ViewEdge * mate = NULL;

                if (v[i] == NULL)
                    continue;

                TVertex * tvert = dynamic_cast<TVertex*>(v[i]);
                if (tvert != NULL)
                {
                    if (tvert->frontEdgeA().first == edge || tvert->frontEdgeB().first == edge)
                    {
                        mate = tvert->mate(edge);

                        if (mate == NULL || mate->ambiguousVisibility())
                            continue;
//                        {
//                            if (tvert->backEdgeA().first->qi() == 0)
//                                mate = tvert->backEdgeA().first;
//                            else
//                                if (tvert->backEdgeB().first->qi() == 0)
//                                    mate = tvert->backEdgeB().first;
//                        }
                    }
                }
                else
                {
                    NonTVertex * ntv = (NonTVertex*)v[i];
                    assert(ntv);

                    if (ntv->viewedges().size() == 2 && (!(ntv->getNature() & Nature::CUSP) || edge->getNature() == Nature::BORDER ))
                        mate = (ntv->viewedges()[0].first != edge ? ntv->viewedges()[0].first : ntv->viewedges()[1].first);
                }

                if (mate != NULL && mate->qi() == 0)
                {
                    printf("%08X found visible mate %08X\n",edge,mate);
                    edge->SetQI(0);
                    ambiguousEdges.erase(ambiguousEdges.find(edge));
                    edge->FixAmbiguous();
                    changed = true;
                    break;
                }
            }
        }
    }
}

/*
// for all visible mesh silhouette curves that are bounded by a cusp on one end and a T-junction on the other, check if the curve is below an image-space arc-length threshold.  if so, mark it invisible
void ViewMapBuilder::TrimCuspCurves(ViewMap * ioViewMap)
{
  if (_cuspTrimThreshold == 0)
    return;

  for(vector<ViewEdge*>::iterator it = ioViewMap->ViewEdges().begin(); it != ioViewMap->ViewEdges().end(); ++it)
    {
      ViewEdge * ve = *it;
      FEdge * feA = ve->fedgeA();

      if (feA->isSmooth() || !(feA->getNature() & Nature::SILHOUETTE) || ve->qi() > 0)
    continue;

      TVertex * tvert = NULL;
      NonTVertex * cuspVertex = NULL;

      // figure out which end is which

      if (ve->A()->getNature() & Nature::CUSP)
    {
      assert(dynamic_cast<NonTVertex*>(ve->A()) != NULL);
      cuspVertex = (NonTVertex*)ve->A();
    }
      else
    if (ve->A()->getNature() & Nature::T_VERTEX)
      {
        assert(dynamic_cast<TVertex*>(ve->A()) != NULL);
        tvert = (TVertex*)ve->A();
      }
    else
      continue;

      if (ve->B()->getNature() & Nature::CUSP)
    {
      assert(dynamic_cast<NonTVertex*>(ve->B()) != NULL);
      cuspVertex = (NonTVertex*)ve->B();
    }
      else
    if (ve->B()->getNature() & Nature::T_VERTEX)
      {
        assert(dynamic_cast<TVertex*>(ve->B()) != NULL);
        tvert = (TVertex*)ve->B();
      }
    else
      continue;

      if (tvert == NULL || cuspVertex == NULL)
    continue;

      if (tvert->sameFace())
    continue;

      // find the other edge connecting to the cusp
      ViewEdge *cuspEdge;

      assert(cuspVertex->viewedges().size() == 2);
      if ( cuspVertex->viewedges()[0].first != ve)
    cuspEdge = cuspVertex->viewedges()[0].first;
      else
    cuspEdge = cuspVertex->viewedges()[1].first;

      assert(cuspEdge != ve);

      if (cuspEdge->qi() == 0)  // only trim if the next bit is invisible
    continue;

      real arcLength = 0;

      FEdge * fe = feA;

      do
    {
      arcLength += fe->getLength2D();
      fe = fe->nextEdge();
    } while (fe != NULL && fe != feA && arcLength < _cuspTrimThreshold);

      if (arcLength < _cuspTrimThreshold)
    ve->SetQI(1337);
    }
}
*/


/*
void ViewMapBuilder::TrimCuspCurves(ViewMap * ioViewMap)
{
// version 1:
  // start from any cusp. if one edge of the cusp is visible and one is invisible, see if there's a small loop that contains this vertex and has certain properties

  // could modify it to start from any dead-end: a view-vertex attached to just one visible curve.
  // In normal operation, this should only occur at cusps, but inconsistency + heuristics might play a role

  for(vector<ViewVertex*>::iterator vit = ioViewMap->ViewVertices().begin(); vit != ioViewMap->ViewVertices().end(); ++vit)
    {
      if (!( (*vit)->getNature() & Nature::CUSP))
    continue;

      assert(dynamic_cast<NonTVertex*>(*vit) != NULL);
      NonTVertex * cuspVertex = (NonTVertex*)*vit;

      assert(cuspVertex->viewedges().size() == 2);

      // determine which is the visible edge and which is the invisible one

      ViewEdge * visStart = NULL;
      ViewEdge * invisStart = NULL;

      // TODO: think about what happens with ambiguous visibility
      
      if (cuspVertex->viewedges()[0].first->qi() == 0)
    {
      visStart  = cuspVertex->viewedges()[0].first;
      invisStart  = cuspVertex->viewedges()[1].first;
    }
      else
    {
      visStart  = cuspVertex->viewedges()[1].first;
      invisStart  = cuspVertex->viewedges()[0].first;
    }

      // check if we have one vis and one invis
      if (visStart->qi() > 0 || invisStart->qi() == 0)
    continue;


      ViewEdge * currentEdge = visStart;
      ViewVertex * lastVertex = cuspVertex;

      bool found = false;

      // trace from the invis edge to see if we get back to an occluding T-vertex

      real arcLength = ArcLength2D(currentEdge, 2*_cuspTrimThreshold);

      vector<ViewEdge*> visEdges;
      while (currentEdge->qi() == 0 && !found && arcLength < _cuspTrimThreshold)
    {
      visEdges.push_back(currentEdge);

      //    advance to the next edge
      ViewVertex * nextVertex = currentEdge->A() == lastVertex ? currentEdge->B() : currentEdge->A();

      assert(nextVertex != NULL);

      TVertex * tvert = dynamic_cast<TVertex*>(nextVertex);
      if (tvert != NULL)
        {
          ViewEdge * mate = tvert->mate(currentEdge);
          // if this is a T-Vertex, we have to be coming in through a visible occluding side and leaving through an invisible occluded side.  otherwise keep going
          if ( (!(currentEdge == tvert->frontEdgeA().first || currentEdge == tvert->frontEdgeB().first) ) ||
           tvert->backEdgeA().first == NULL || tvert->backEdgeB().first == NULL)
        currentEdge = tvert->mate(currentEdge);
          else
        // check if there is exactly one visible non-mate edge
        if ( (tvert->backEdgeA().first->qi() == 0 && tvert->backEdgeB().first->qi() > 0) ||
             (tvert->backEdgeA().first->qi() > 0 && tvert->backEdgeB().first->qi() == 0) )
          found = true;
          else
        currentEdge = tvert->mate(currentEdge);
        }
      else
        {
          assert(dynamic_cast<NonTVertex*>(nextVertex) != NULL);
          NonTVertex * ntv = (NonTVertex*)(nextVertex);

          if (ntv->viewedges().size() != 2)
        break;

          currentEdge = (ntv->viewedges()[0].first == currentEdge ? ntv->viewedges()[1].first : ntv->viewedges()[0].first);
        }

      arcLength += ArcLength2D(currentEdge, 2*_cuspTrimThreshold);

      lastVertex = nextVertex;
    }

      // mark the visible edges in the loop as invisible.

      if (found)
    for(vector<ViewEdge*>::iterator it = visEdges.begin(); it != visEdges.end(); ++it)
      (*it)->SetQI(1337);
    }
}
*/

/*
// version 2: start from a T-vertex. if there's a loop (with small image-space arc length) formed around this t-vertex, mark the loop invisible
void ViewMapBuilder::TrimCuspCurves(ViewMap * ioViewMap)
{
  printf("Trimming small loops\n");
  
  // For each loop, we keep track of all the T-Vertices within the loop.  Each loop is organized according to the T-Vertex at the start of the loop
  multimap<TVertex*, TVertex*> loopMembers; // each loop is "rooted" at a particular TVertex.  This multimap keeps track of all the TVertices within the loop

  int i =0;

  bool changed;
  do
    {
      //      printf("\tPass %d\n", i++);
      changed = false;  // this could be made a lot more efficient by keeping a queue of vertices to be visited

      for(vector<ViewVertex*>::iterator vit = ioViewMap->ViewVertices().begin(); vit != ioViewMap->ViewVertices().end(); ++vit)
    {
      TVertex * tv = dynamic_cast<TVertex*>(*vit);
      if (tv == NULL)// || tv->sameFace())
        continue;

      //	  if (tv->frontEdgeA().first == tv->frontEdgeB().first) // extremely unlikely special case
      //	    {
      //	      if (tv->frontEdgeA().first->qi() == 0)
      //		tv->frontEdgeA().first->SetQI(1337);
      //	      continue;
      //	    }

      for(int i=0;i<2;i++)
        {
          ViewEdge * startEdge = (i == 0 ? tv->frontEdgeA().first : tv->frontEdgeB().first);

          if (startEdge == NULL || startEdge->qi() != 0) // starting edge must be visible
        continue;

          // trace from the start edge and see if we get back to the original T-Vertex

          set<ViewEdge*> loopEdges;
          ViewVertex * lastVertex = tv;
          ViewEdge * currentEdge = startEdge;
          loopEdges.insert(currentEdge);
          real arcLength = 0;

          if (currentEdge->qi() == 0)
        arcLength = ArcLength2D(currentEdge, 2*_cuspTrimThreshold);
          bool foundLoop = false;

          set<TVertex*> insideTVerts;

          while(arcLength < _cuspTrimThreshold)
        {
          ViewVertex * nextVertex = currentEdge->A() != lastVertex ? currentEdge->A() : currentEdge->B();
          if (nextVertex == tv)
            {
              // only want to catch loops that connect front-to-back (i.e., the curve crosses over itself)
              if (tv->backEdgeA().first == currentEdge || tv->backEdgeB().first == currentEdge)
            foundLoop = true;
              break;
            }

          // continue along the next viewedge
          TVertex * tvert = dynamic_cast<TVertex*>(nextVertex);
          if (tvert != NULL)
            {
              if (!tvert->sameFace())
            currentEdge = tvert->mate(currentEdge);
              else
            {
              // find a visible outgoing edge.  there should normally be at most one (other than the one we came from).  (what if there's more than one?)
              // not sure this handles all important cases

              bool found = false;
              for(int j=0;j<tvert->numEdges();j++)// maybe should skip mate of the current edge?
                if (tvert->getEdge(j)->first != currentEdge && tvert->getEdge(j)->first->qi() == 0)
                  {
                currentEdge = tvert->getEdge(j)->first;
                found = true;
                  }

              if (!found)
                break;
            }

              insideTVerts.insert(tvert);

              //		      if (tvert->sameFace()) // give up if we hit an same-face intersecti... not sure what to do about them.
              //			break;
            }
          else
            {
              assert(dynamic_cast<NonTVertex*>(nextVertex) != NULL);
              NonTVertex * ntv = (NonTVertex*)(nextVertex);

              if (ntv->viewedges().size() != 2) // not sure if there's something sensible to do here. the purpose of this iteration is silhouettes, which shoudn't have these bifurcations
            break;

              currentEdge = (ntv->viewedges()[0].first != currentEdge ? ntv->viewedges()[0].first : ntv->viewedges()[1].first);
            }
          if (currentEdge == NULL)
            break;

          if (loopEdges.find(currentEdge) != loopEdges.end())
            {
              printf("FOUND INFINITE LOOP???\n\tNatures: ");
              for(set<ViewEdge*>::iterator it = loopEdges.begin(); it != loopEdges.end(); ++it)
            printf(" %d", (*it)->getNature());
              printf("\n");
              break;
            }

          if (currentEdge->qi() == 0)
            arcLength += ArcLength2D(currentEdge, 2*_cuspTrimThreshold);
          loopEdges.insert(currentEdge);
          lastVertex = nextVertex;
        }

          if (foundLoop)
        {
          for(set<ViewEdge*>::iterator it = loopEdges.begin(); it != loopEdges.end(); ++it)
            {
              if ( (*it)->qi() ==0)
            changed = true;
              (*it)->SetQI(1337);
            }
          if (insideTVerts.size() > 0)
            for(set<TVertex*>::iterator tvit = insideTVerts.begin(); tvit != insideTVerts.end(); ++tvit)
              loopMembers.insert(pair<TVertex*,TVertex*>(tv, *tvit));

          //		  printf("Found loop. insideTVerts: %d, loopMembers: %d\n", insideTVerts.size(), loopMembers.size());
        }
        }
    }
    } while (changed);


  // merge all T-Vertices with mutually overlapping loops. not sure this will ever be more than 2 per loop...
  set<TVertex*> visited;

  vector<set<TVertex*> > mergeGroups;

  printf("Computing groups\n");

  for(vector<ViewVertex*>::iterator vit = ioViewMap->ViewVertices().begin(); vit != ioViewMap->ViewVertices().end(); ++vit)
    {
      TVertex * tv = dynamic_cast<TVertex*>(*vit);
      if (tv == NULL || visited.find(tv) != visited.end())
    continue;

      // build transitive closure: all tvertices to merge with this one
      set<TVertex*> tvGroup;

      vector<TVertex*> tvqueue;
      tvqueue.push_back(tv);

      while(tvqueue.size() > 0)
    {
      TVertex * next = tvqueue.back();
      tvqueue.pop_back();

      if (visited.find(next) != visited.end())
        continue;

      visited.insert(next);
      tvGroup.insert(next);

      for(multimap<TVertex*,TVertex*>::iterator lit = loopMembers.lower_bound(next); lit != loopMembers.upper_bound(next); ++lit)
        {
          TVertex * opp = (*lit).second;

          // check if it's mutual or the other one is a sameFace... (TBD)
          bool mutual = opp->sameFace();
          for(multimap<TVertex*,TVertex*>::iterator it2 = loopMembers.lower_bound(opp);
          it2 != loopMembers.upper_bound(opp); ++it2)
        if ((*it2).second == next)
          {
            mutual = true;
            break;
          }

          if (mutual)
        tvqueue.push_back(opp);
        }
    }

      if (tvGroup.size() > 1)
    mergeGroups.push_back(tvGroup);
      // point from each vertex in the group to all others ...  or merge them
    }

  printf("Merging %d groups\n", mergeGroups.size());
  
  for(vector<set<TVertex*> >::iterator it = mergeGroups.begin(); it != mergeGroups.end(); ++it)
  ioViewMap->MergeTVertices(*it);
    }
 */



ViewEdge * AdvanceAlongVertex(ViewVertex * nextVertex, ViewEdge * lastEdge)
{
    TVertex * tvert = dynamic_cast<TVertex*>(nextVertex);

    if (tvert != NULL)
    {
        if (!tvert->sameFace())
        {
            ViewEdge * mate = tvert->mate(lastEdge);
            if (mate->qi() == 0)
                return mate;
            else
                return NULL;
        }

        // find a visible outgoing edge.  there should normally be at most one (other than the one we came from).  (what if there's more than one?)
        // not sure this handles all important cases

        bool found = false;
        for(int j=0;j<tvert->numEdges();j++)// maybe should skip mate of the current edge?
            if (tvert->getEdge(j)->first != lastEdge && tvert->getEdge(j)->first->qi() == 0)
                return tvert->getEdge(j)->first;

        return NULL;
    }

    assert(dynamic_cast<NonTVertex*>(nextVertex) != NULL);
    NonTVertex * ntv = (NonTVertex*)(nextVertex);

    if (ntv->viewedges().size() != 2) // not sure if there's something sensible to do here. the purpose of this iteration is silhouettes, which shoudn't have these bifurcations
        return NULL;

    if (ntv->viewedges()[0].first != lastEdge && ntv->viewedges()[0].first->qi() == 0)
        return ntv->viewedges()[0].first;

    if (ntv->viewedges()[1].first != lastEdge && ntv->viewedges()[1].first->qi() == 0)
        return ntv->viewedges()[1].first;

    return NULL;
}


inline int NumEdges(ViewVertex * vv)
{
    TVertex * tv = dynamic_cast<TVertex*>(vv);
    if (tv != NULL)
        return tv->numEdges();
    return  ((NonTVertex*)vv)->viewedges().size();
}

inline ViewEdge * GetEdge(ViewVertex * vv, int i)
{
    TVertex * tv = dynamic_cast<TVertex*>(vv);
    if (tv != NULL)
        return tv->getEdge(i)->first;
    return ((NonTVertex*)vv)->viewedges()[i].first;
}


inline
int NumVisibleEdgesTV(TVertex * tv)
{
    int vis = 0;

    for(int i=0;i<tv->numEdges();i++)
        if ( tv->getEdge(i)->first->qi() == 0)
            vis ++;
    return vis;
}

inline
int NumVisibleEdgesNTV(NonTVertex * ntv)
{
    int vis =0;
    for(int i=0;i<ntv->viewedges().size();i++)
        if (ntv->viewedges()[i].first->qi() == 0)
            vis ++;

    return vis;
}
inline
int NumVisibleEdges(ViewVertex * vv)
{
    TVertex * tv = dynamic_cast<TVertex*>(vv);

    if (tv != NULL)
        return NumVisibleEdgesTV(tv);

    assert(dynamic_cast<NonTVertex*>(vv) != NULL);
    return NumVisibleEdgesNTV((NonTVertex*)vv);
}

// if this vertex has a SINGLE visible outgoing edge, return it.  otherwise, return NULL
ViewEdge * GetSoleVisibleEdge(ViewVertex * vv)
{
    TVertex * tv = dynamic_cast<TVertex*>(vv);

    if (tv != NULL)
    {
        ViewEdge * ve = NULL;
        for(int i=0;i<tv->numEdges();i++)
            if ( tv->getEdge(i)->first->qi() == 0)
            {
                if (ve != NULL)
                    return NULL;

                ve = tv->getEdge(i)->first;
            }

        return ve;
    }

    assert(dynamic_cast<NonTVertex*>(vv) != NULL);
    NonTVertex * ntv = (NonTVertex*)vv;

    ViewEdge * ve = NULL;
    for(int i=0;i<ntv->viewedges().size();i++)
        if (ntv->viewedges()[i].first->qi() == 0)
        {
            if (ve != NULL)
                return NULL;
            ve = ntv->viewedges()[i].first;
        }

    return ve;
}

// define a TrueTVertex as one that has three visible edges emanating from it.  (thus, cant' be a SameFace TVert)
// erase a set of edges that connect the front of a TrueTVertex to a cusp, if they are all visible AND there is no other TrueTVertex along the way.
// treat inconsistent as visible (?)
//
// also, remove short curves that connect dead-ends to dead-ends
bool ViewMapBuilder::HideDeadEnds(ViewMap * ioViewMap)
{
    bool changed = false;

    for(vector<ViewVertex*>::iterator vit = ioViewMap->ViewVertices().begin(); vit != ioViewMap->ViewVertices().end(); ++vit)
    {
        ViewEdge * startEdge = GetSoleVisibleEdge(*vit);

        // only start from this vertex if there's a sole visible outgoing edge
        if (startEdge == NULL)
            continue;

        assert(startEdge->qi() == 0);

        // trace from the start edge and see if we get to a True T-Vertex
        set<ViewEdge*> visitedEdges;
        ViewVertex * lastVertex = *vit;
        ViewEdge * currentEdge = startEdge;
        real arcLength = 0;

        bool foundEdgeToDelete = false;

        arcLength += ArcLength2D(currentEdge, 2*_cuspTrimThreshold);
        while(arcLength < _cuspTrimThreshold)
        {
            if (currentEdge->qi() != 0)
                break;

            visitedEdges.insert(currentEdge);

            ViewVertex * nextVertex = currentEdge->A() != lastVertex ? currentEdge->A() : currentEdge->B();

            assert(nextVertex != NULL);

            if (NumVisibleEdges(nextVertex) == 3)  // if this is a true T-junction, then we've succeeded in finding a dead-end curve to hide
            {
                foundEdgeToDelete = true;
                break;
            }

            /*
      TVertex * tv = dynamic_cast<TVertex*>(nextVertex);

      // if this is a TrueTVertex, we've succeeded in finding a dead-end curve to hide.
      if (tv != NULL && !tv->sameFace() && tv->frontEdgeA().first->qi() == 0 && tv->frontEdgeB().first->qi() == 0 &&
          (tv->backEdgeA().first->qi() == 0 || tv->backEdgeB().first->qi() == 0))
        {
          foundEdgeToDelete = true;
          break;
        }
      */
            // continue along the next viewedge
            currentEdge = AdvanceAlongVertex(nextVertex, currentEdge);

            if (currentEdge == NULL) // this only happens if we reached a dead-end (or ended in a T-vertex)
            {
                foundEdgeToDelete = true;
                break;
            }

            if (visitedEdges.find(currentEdge) != visitedEdges.end())
            {
                //		      printf("FOUND INFINITE LOOP???\n\tNatures: ");
                //		      for(set<ViewEdge*>::iterator it = loopEdges.begin(); it != loopEdges.end(); ++it)
                //			printf(" %d", (*it)->getNature());
                //		      printf("\n");
                break;
            }

            lastVertex = nextVertex;
            arcLength += ArcLength2D(currentEdge, 2*_cuspTrimThreshold);
        }

        if (foundEdgeToDelete)
        {
            for(set<ViewEdge*>::iterator it = visitedEdges.begin(); it != visitedEdges.end(); ++it)
            {
                if ( (*it)->qi() == 0)
                    changed = true;
                (*it)->SetQI(1337);
            }
        }

    }

    return changed;
}


ViewEdge * AdvanceAlongVertex2(ViewVertex * nextVertex, ViewEdge * lastEdge)
{
    NonTVertex * ntv = dynamic_cast<NonTVertex*>(nextVertex);
    if (ntv != NULL)
    {
        for(int i=0;i<ntv->viewedges().size();i++)
            if (ntv->viewedges()[i].first != lastEdge && ntv->viewedges()[i].first->qi() == 0)
                return ntv->viewedges()[i].first;

        return NULL;
    }

    TVertex * tv = (TVertex*)nextVertex;
    for(int i=0;i<tv->numEdges();i++)
        if (tv->getEdge(i)->first != lastEdge && tv->getEdge(i)->first->qi() == 0)
            return tv->getEdge(i)->first;

    return NULL;
}

bool ViewMapBuilder::HideSmallBits(ViewMap * ioViewMap)
{
    bool changed = false;

    for(vector<ViewVertex*>::iterator vit = ioViewMap->ViewVertices().begin(); vit != ioViewMap->ViewVertices().end(); ++vit)
    {
        ViewVertex * startVertex = *vit;
        TVertex* startTVertex = dynamic_cast<TVertex*>(startVertex);
        if(!startTVertex)
            continue;

        int numVisibleEdges = NumVisibleEdges(startVertex);
        if(startTVertex->numEdges()==4 && numVisibleEdges == 3){
            ViewEdge* startEdge = NULL;
            ViewVertex* endVertex = NULL;
            ViewEdge* otherEdges[2];
            if(startTVertex->backEdgeA().first->qi()==0 && startTVertex->backEdgeB().first->qi()!=0){
                startEdge = startTVertex->backEdgeA().first;
                endVertex = startEdge->A();
                otherEdges[0] = startTVertex->frontEdgeA().first;
                otherEdges[1] = startTVertex->frontEdgeB().first;
            }else if(startTVertex->backEdgeA().first->qi()!=0 && startTVertex->backEdgeB().first->qi()==0){
                startEdge = startTVertex->backEdgeB().first;
                endVertex = startEdge->B();
                otherEdges[0] = startTVertex->frontEdgeA().first;
                otherEdges[1] = startTVertex->frontEdgeB().first;
            }else if(startTVertex->frontEdgeA().first->qi()==0 && startTVertex->frontEdgeB().first->qi()!=0){
                startEdge = startTVertex->frontEdgeA().first;
                endVertex = startEdge->A();
                otherEdges[0] = startTVertex->backEdgeA().first;
                otherEdges[1] = startTVertex->backEdgeB().first;
            }else if(startTVertex->frontEdgeA().first->qi()!=0 && startTVertex->frontEdgeB().first->qi()==0){
                startEdge = startTVertex->frontEdgeB().first;
                endVertex = startEdge->B();
                otherEdges[0] = startTVertex->backEdgeA().first;
                otherEdges[1] = startTVertex->backEdgeB().first;
            }
            assert(startEdge && endVertex);
            if(startVertex == endVertex)
                continue;

            real arclength = ArcLength2D(startEdge,2*_cuspTrimThreshold);
            if(arclength < _cuspTrimThreshold){
                TVertex* endTVertex = dynamic_cast<TVertex*>(endVertex);

                if(!endTVertex){
                    // NonTVertex case
                    NonTVertex* endNTVertex = dynamic_cast<NonTVertex*>(endVertex);
                    if(endNTVertex->viewedges().size()==3){
                        bool found = false;
                        ViewEdge* thirdEdge = NULL;
                        for(int i=0;i<3;i++){
                            ViewEdge* e = endNTVertex->viewedges()[i].first;
                            found = found || (e == otherEdges[0]) || (e == otherEdges[1]);
                            if(e!=otherEdges[0] && e!=otherEdges[1] && e!=startEdge)
                                thirdEdge = e;
                        }
                        if(found && thirdEdge && thirdEdge->qi()==0){
                            printf("HIDE SMALL BITS %08X\n",startEdge);
                            startEdge->SetQI(1339);
                            changed = true;
                            continue;
                        }
                    }
                    continue;
                }

                // TVertex case
                numVisibleEdges = NumVisibleEdges(endVertex);

                ViewEdge* midEdge = NULL;
                TVertex* midVertex = NULL;
                if(endTVertex->numEdges()==4 && (numVisibleEdges == 2 || numVisibleEdges == 4)){
                    midVertex = endTVertex;
                    if(endTVertex->frontEdgeA().first == startEdge && endTVertex->frontEdgeB().first->qi() == 0){
                        midEdge = startEdge;
                        startEdge = endTVertex->frontEdgeB().first;
                        endVertex = startEdge->B();
                    }else if(endTVertex->frontEdgeB().first == startEdge && endTVertex->frontEdgeA().first->qi() == 0){
                        midEdge = startEdge;
                        startEdge = endTVertex->frontEdgeA().first;
                        endVertex = startEdge->A();
                    }else if(endTVertex->backEdgeA().first == startEdge && endTVertex->backEdgeB().first->qi() == 0){
                        midEdge = startEdge;
                        startEdge = endTVertex->backEdgeB().first;
                        endVertex = startEdge->B();
                    }else if(endTVertex->backEdgeB().first == startEdge && endTVertex->backEdgeA().first->qi() == 0){
                        midEdge = startEdge;
                        startEdge = endTVertex->backEdgeA().first;
                        endVertex = startEdge->A();
                    }
                }
                if(midEdge){
                    endTVertex = dynamic_cast<TVertex*>(endVertex);
                    if(!endTVertex)
                        continue;
                    numVisibleEdges = NumVisibleEdges(endVertex);
                }

                if(endTVertex->numEdges()==4 && numVisibleEdges == 3){
                    bool valid = false;

                    //   o-o----o
                    //   |      |   Detect these kind of cases (one invisible TVertex allowed)x
                    // --o---o--o---
                    if((endTVertex->backEdgeA().first==startEdge && endTVertex->backEdgeB().first->qi()!=0)
                            || (endTVertex->backEdgeB().first==startEdge && endTVertex->backEdgeA().first->qi()!=0)){
                        valid = (endTVertex->frontEdgeA().first == otherEdges[0]) || (endTVertex->frontEdgeA().first == otherEdges[1])
                                || (endTVertex->frontEdgeB().first == otherEdges[0]) || (endTVertex->frontEdgeB().first == otherEdges[1]);
                        if(!valid){
                            TVertex* midV = dynamic_cast<TVertex*>(endTVertex->frontEdgeA().first->A());
                            if(midV && (midV == otherEdges[0]->A() || midV == otherEdges[1]->B())){
                                if(midV == midVertex){
                                    valid = true;
                                }else if(midV->frontEdgeA().first == endTVertex->frontEdgeA().first ||
                                        midV->frontEdgeB().first == endTVertex->frontEdgeA().first){
                                    valid = ((!midV->backEdgeA().first || midV->backEdgeA().first->qi()!=0)
                                             && (!midV->backEdgeB().first || midV->backEdgeB().first->qi()!=0));
                                }else{
                                    valid = ((!midV->frontEdgeA().first || midV->frontEdgeA().first->qi()!=0)
                                             && (!midV->frontEdgeB().first || midV->frontEdgeB().first->qi()!=0));
                                }
                            }
                        }
                        if(!valid){
                            TVertex* midV = dynamic_cast<TVertex*>(endTVertex->frontEdgeB().first->B());
                            if(midV && (midV == otherEdges[0]->A() || midV == otherEdges[1]->B())){
                                if(midV == midVertex){
                                    valid = true;
                                }else if(midV->frontEdgeA().first == endTVertex->frontEdgeB().first ||
                                        midV->frontEdgeB().first == endTVertex->frontEdgeB().first){
                                    valid = ((!midV->backEdgeA().first || midV->backEdgeA().first->qi()!=0)
                                              && (!midV->backEdgeB().first || midV->backEdgeB().first->qi()!=0));
                                }else{
                                    valid = ((!midV->frontEdgeA().first || midV->frontEdgeA().first->qi()!=0)
                                             && (!midV->frontEdgeB().first || midV->frontEdgeB().first->qi()!=0));
                                }
                            }
                        }
                    }
                    if(!valid && (endTVertex->frontEdgeA().first==startEdge && endTVertex->frontEdgeB().first->qi()!=0)
                            || (endTVertex->frontEdgeB().first==startEdge && endTVertex->frontEdgeA().first->qi()!=0)){
                        valid = (endTVertex->backEdgeA().first == otherEdges[0]) || (endTVertex->backEdgeA().first == otherEdges[1])
                                || (endTVertex->backEdgeB().first == otherEdges[0]) || (endTVertex->backEdgeB().first == otherEdges[1]);
                        if(!valid){
                            TVertex* midV = dynamic_cast<TVertex*>(endTVertex->backEdgeA().first->A());
                            if(midV && (midV == otherEdges[0]->A() || midV == otherEdges[1]->B())){
                                if(midV == midVertex){
                                    valid = true;
                                }else if(midV->frontEdgeA().first == endTVertex->backEdgeA().first ||
                                        midV->frontEdgeB().first == endTVertex->backEdgeA().first){
                                    valid = ((!midV->backEdgeA().first || midV->backEdgeA().first->qi()!=0)
                                             && (!midV->backEdgeB().first || midV->backEdgeB().first->qi()!=0));
                                }else{
                                    valid = ((!midV->frontEdgeA().first || midV->frontEdgeA().first->qi()!=0)
                                             && (!midV->frontEdgeB().first || midV->frontEdgeB().first->qi()!=0));
                                }
                            }
                        }
                        if(!valid){
                            TVertex* midV = dynamic_cast<TVertex*>(endTVertex->backEdgeB().first->B());
                            if(midV && (midV == otherEdges[0]->A() || midV == otherEdges[1]->B())){
                                if(midV == midVertex){
                                    valid = true;
                                }else if(midV->frontEdgeA().first == endTVertex->backEdgeB().first ||
                                        midV->frontEdgeB().first == endTVertex->backEdgeB().first){
                                    valid = ((!midV->backEdgeA().first || midV->backEdgeA().first->qi()!=0)
                                             && (!midV->backEdgeB().first || midV->backEdgeB().first->qi()!=0));
                                }else{
                                    valid = ((!midV->frontEdgeA().first || midV->frontEdgeA().first->qi()!=0)
                                             && (!midV->frontEdgeB().first || midV->frontEdgeB().first->qi()!=0));
                                }
                            }
                        }
                    }

                    //   o-o---o--
                    //   |     |   Detect these kind of cases (one invisible TVertex allowed)
                    // --o---o-o
                    if(!valid && (endTVertex->backEdgeA().first==startEdge || endTVertex->backEdgeB().first==startEdge)){
                        if(endTVertex->frontEdgeA().first->qi()==0 && endTVertex->frontEdgeB().first->qi()!=0)
                            valid = (endTVertex->frontEdgeA().first == otherEdges[0]) || (endTVertex->frontEdgeA().first == otherEdges[1]);
                        else if(endTVertex->frontEdgeA().first->qi()!=0 && endTVertex->frontEdgeB().first->qi()==0)
                            valid = (endTVertex->frontEdgeB().first == otherEdges[0]) || (endTVertex->frontEdgeB().first == otherEdges[1]);
                        if(!valid){
                            TVertex* midV = dynamic_cast<TVertex*>(endTVertex->frontEdgeA().first->A());
                            if(midV && (midV == otherEdges[0]->A() || midV == otherEdges[1]->B())){
                                if(midV == midVertex){
                                    valid = true;
                                }else if(midV->frontEdgeA().first == endTVertex->frontEdgeA().first ||
                                        midV->frontEdgeB().first == endTVertex->frontEdgeA().first){
                                    valid = ((!midV->backEdgeA().first || midV->backEdgeA().first->qi()!=0)
                                             && (!midV->backEdgeB().first || midV->backEdgeB().first->qi()!=0));
                                }else{
                                    valid = ((!midV->frontEdgeA().first || midV->frontEdgeA().first->qi()!=0)
                                             && (!midV->frontEdgeB().first || midV->frontEdgeB().first->qi()!=0));
                                }
                            }
                        }
                        if(!valid){
                            TVertex* midV = dynamic_cast<TVertex*>(endTVertex->frontEdgeB().first->B());
                            if(midV && (midV == otherEdges[0]->A() || midV == otherEdges[1]->B())){
                                if(midV == midVertex){
                                    valid = true;
                                }else if(midV->frontEdgeA().first == endTVertex->frontEdgeB().first ||
                                        midV->frontEdgeB().first == endTVertex->frontEdgeB().first){
                                    valid = ((!midV->backEdgeA().first || midV->backEdgeA().first->qi()!=0)
                                             && (!midV->backEdgeB().first || midV->backEdgeB().first->qi()!=0));
                                }else{
                                    valid = ((!midV->frontEdgeA().first || midV->frontEdgeA().first->qi()!=0)
                                             && (!midV->frontEdgeB().first || midV->frontEdgeB().first->qi()!=0));
                                }
                            }
                        }
                    }
                    if(!valid && (endTVertex->frontEdgeA().first==startEdge || endTVertex->frontEdgeB().first==startEdge)){
                        if(endTVertex->backEdgeA().first->qi()==0 && endTVertex->backEdgeB().first->qi()!=0)
                            valid = (endTVertex->backEdgeA().first == otherEdges[0]) || (endTVertex->backEdgeA().first == otherEdges[1]);
                        else if(endTVertex->backEdgeA().first->qi()!=0 && endTVertex->backEdgeB().first->qi()==0)
                            valid = (endTVertex->backEdgeB().first == otherEdges[0]) || (endTVertex->backEdgeB().first == otherEdges[1]);
                        if(!valid){
                            TVertex* midV = dynamic_cast<TVertex*>(endTVertex->backEdgeA().first->A());
                            if(midV && (midV == otherEdges[0]->A() || midV == otherEdges[1]->B())){
                                if(midV == midVertex){
                                    valid = true;
                                }else if(midV->frontEdgeA().first == endTVertex->backEdgeA().first ||
                                        midV->frontEdgeB().first == endTVertex->backEdgeA().first){
                                    valid = ((!midV->backEdgeA().first || midV->backEdgeA().first->qi()!=0)
                                             && (!midV->backEdgeB().first || midV->backEdgeB().first->qi()!=0));
                                }else{
                                    valid = ((!midV->frontEdgeA().first || midV->frontEdgeA().first->qi()!=0)
                                             && (!midV->frontEdgeB().first || midV->frontEdgeB().first->qi()!=0));
                                }
                            }
                        }
                        if(!valid){
                            TVertex* midV = dynamic_cast<TVertex*>(endTVertex->backEdgeB().first->B());
                            if(midV && (midV == otherEdges[0]->A() || midV == otherEdges[1]->B())){
                                if(midV == midVertex){
                                    valid = true;
                                }else if(midV->frontEdgeA().first == endTVertex->backEdgeB().first ||
                                        midV->frontEdgeB().first == endTVertex->backEdgeB().first){
                                    valid = ((!midV->backEdgeA().first || midV->backEdgeA().first->qi()!=0)
                                             && (!midV->backEdgeB().first || midV->backEdgeB().first->qi()!=0));
                                }else{
                                    valid = ((!midV->frontEdgeA().first || midV->frontEdgeA().first->qi()!=0)
                                            && (!midV->frontEdgeB().first || midV->frontEdgeB().first->qi()!=0));
                                }
                            }
                        }
                    }

                    if(valid){
                        printf("HIDE SMALL BITS %08X\n",startEdge);
                        startEdge->SetQI(1339);
                        if(midEdge){
                            printf("HIDE SMALL BITS %08X\n",midEdge);
                            midEdge->SetQI(1339);
                        }
                        changed = true;
                        continue;
                    }
                }
            }
        }
    }
    return changed;
}

// let:
//  * dead-end be a vertex with 1 visible outgoing edge
//  * junction be a vertex > 2 visible outgoing edges

// hide curve segments that:
// * connect a junction to a dead-end, 
// * connect a dead-end to a dead-end
// * connect any vertex to itself

// ONLY along segments that 
// * contain no other junctions, 
// * no invisible edges, and 
// * are within the length threshold
//
// some of the code below only looks at t-vertices, not general junctions
bool ViewMapBuilder::HideSmallLoopsAndDeadEnds(ViewMap * ioViewMap)
{
    bool changed = false;

    for(vector<ViewVertex*>::iterator vit = ioViewMap->ViewVertices().begin(); vit != ioViewMap->ViewVertices().end(); ++vit)
    {
        ViewVertex * startVertex = *vit;

        for(int i=0;i<NumEdges(startVertex);i++)
        {
            ViewEdge * startEdge = GetEdge(startVertex, i);

            if (startEdge->qi() != 0)
                continue;

            // hide single-edge loop (not sure if this case needs special treatment)
            if (startEdge->A() == startVertex && startEdge->B() == startVertex && ArcLength2D(startEdge, 2*_cuspTrimThreshold))
            {
                startEdge->SetQI(1338);
                changed = true;
                continue;
            }

            // trace from the start edge
            set<ViewEdge*> visitedEdges;
            ViewVertex * lastVertex = startVertex;
            ViewEdge * currentEdge = startEdge;
            real arcLength = 0;

            bool foundSegmentToDelete = false;

            arcLength += ArcLength2D(currentEdge, 2*_cuspTrimThreshold);
            while(arcLength < _cuspTrimThreshold)
            {
                if (currentEdge->qi() != 0)
                    break;

                visitedEdges.insert(currentEdge);

                ViewVertex * nextVertex = currentEdge->A() != lastVertex ? currentEdge->A() : currentEdge->B();

                assert(nextVertex != NULL);

                if (nextVertex == startVertex) // check if we found a small loop
                {
                    foundSegmentToDelete = true;
                    break;
                }

                int numVis = NumVisibleEdges(nextVertex);
                assert(numVis > 0);
                if (numVis == 1) // check if we found a dead-end
                {
                    if (NumVisibleEdges(startVertex) != 2) // delete dead-end to dead-end, or dead-end to junction
                        foundSegmentToDelete = true;
                    break;
                }

                if (numVis > 2) // check if found a junction
                {
                    if (NumVisibleEdges(startVertex) == 1)  // delete junction to dead-end
                        foundSegmentToDelete = true;
                    break;
                }

                // continue along the next viewedge
                currentEdge = AdvanceAlongVertex2(nextVertex, currentEdge);
                assert(currentEdge != NULL);

                if (visitedEdges.find(currentEdge) != visitedEdges.end())
                {
                    //		      printf("FOUND INFINITE LOOP???\n\tNatures: ");
                    //		      for(set<ViewEdge*>::iterator it = loopEdges.begin(); it != loopEdges.end(); ++it)
                    //			printf(" %d", (*it)->getNature());
                    //		      printf("\n");
                    break;
                }

                lastVertex = nextVertex;
                arcLength += ArcLength2D(currentEdge, 2*_cuspTrimThreshold);
            }

            if (foundSegmentToDelete)
            {
                for(set<ViewEdge*>::iterator it = visitedEdges.begin(); it != visitedEdges.end(); ++it)
                {
                    if ( (*it)->qi() == 0)
                        changed = true;
                    (*it)->SetQI(1337);
                }
            }
        }
    }

    return changed;
}

ViewEdge * AdvanceAlongVertex3(ViewVertex * nextVertex, ViewEdge * lastEdge)
{
    NonTVertex * ntv = dynamic_cast<NonTVertex*>(nextVertex);
    if (ntv != NULL)
    {
        for(int i=0;i<ntv->viewedges().size();i++)
            if (ntv->viewedges()[i].first != lastEdge)
                return ntv->viewedges()[i].first;

        return NULL;
    }

    TVertex * tv = (TVertex*)nextVertex;
    return tv->mate(lastEdge);

    return NULL;
}

// for a sequence of invisible curves bounded by two dead-ends, join the dead-ends, so long as the entire curve doesn't deviate much in image-space
void ViewMapBuilder::GraftDeadEnds(ViewMap * ioViewMap)
{
    vector<pair<ViewVertex*,ViewVertex*> > mergePairs;

    for(vector<ViewVertex*>::iterator vit = ioViewMap->ViewVertices().begin(); vit != ioViewMap->ViewVertices().end(); ++vit)
    {
        ViewVertex * startVertex = *vit;

        if (NumVisibleEdges(startVertex) != 1)
            continue;

        for(int i=0;i<NumEdges(startVertex);i++)
        {
            ViewEdge * startEdge = GetEdge(startVertex, i);

            if (startEdge->qi() == 0)
                continue;

            // trace from the start edge
            set<ViewEdge*> visitedEdges;
            ViewVertex * lastVertex = startVertex;
            ViewEdge * currentEdge = startEdge;

            while (currentEdge->qi() != 0)
            {
                visitedEdges.insert(currentEdge);
                ViewVertex * nextVertex = currentEdge->A() != lastVertex ? currentEdge->A() : currentEdge->B();

                // check if we've looped back to the original dead-end
                if (nextVertex == startVertex)
                    break;

                // check if we've gone too far in image space
                if ( (startVertex->getPoint2D() - nextVertex->getPoint2D()).norm()   > _graftThreshold)
                    break;

                currentEdge = AdvanceAlongVertex3(nextVertex, currentEdge);

                if (currentEdge == NULL)
                    break;

                if (visitedEdges.find(currentEdge) != visitedEdges.end())
                    break;

                lastVertex = nextVertex;
            }

            if (currentEdge != NULL && currentEdge->qi() == 0)
                mergePairs.push_back(pair<ViewVertex*,ViewVertex*>(startVertex,lastVertex));
        }
    }

    set<ViewVertex*> done;

    for(vector<pair<ViewVertex*,ViewVertex*> >::iterator it = mergePairs.begin(); it != mergePairs.end(); ++it)
    {
        //      assert (done.find( (*it).first) == done.end());
        //      assert (done.find( (*it).second) == done.end());

        if (done.find( (*it).first) != done.end() || done.find( (*it).second) != done.end())
            continue;


        ioViewMap->MergeVertices( (*it).first, (*it).second);

        done.insert( (*it).first);
        done.insert( (*it).second);
    }
}
