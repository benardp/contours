//
//  Copyright (C) : Please refer to the COPYRIGHT file distributed 
//   with this source distribution. 
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 2
//  of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
///////////////////////////////////////////////////////////////////////////////

# ifdef __MACH__
#  include <OpenGL/gl.h>
# else
#  include <GL/gl.h>
# endif

#include "ViewMap.h"
#include "../geometry/GeomUtils.h"
#include <float.h>
#include "ViewMapIterators.h"
#include "ViewMapAdvancedIterators.h"
#include "SilhouetteGeomEngine.h" // for an assertion
#include "PunchOut.h"

/**********************************/
/*                                */
/*                                */
/*             ViewMap            */
/*                                */
/*                                */
/**********************************/

ViewMap * ViewMap::_pInstance = 0;

bool hasEdge(ViewVertex *vv, ViewEdge *ve);
bool hasEdgeTwice(ViewVertex *vv, ViewEdge *ve);
void checkVertex(ViewVertex *);

bool checkAllPointers = false;


// ---------- stuff for naming and picking in OpenGL -----

map<void*,unsigned int> mapObjName;
map<unsigned int,int> mapNameRegion;

unsigned int getName(void*obj)
{
    if (mapObjName.find(obj) != mapObjName.end())
        return mapObjName[obj];
    int name = mapObjName.size();
    mapObjName[obj] = name;
    return name;
}

void setRegion(unsigned int name, int regionIndex) { mapNameRegion[name] = regionIndex; }
int getRegion(unsigned int name) { 
    map<unsigned int,int>::iterator it = mapNameRegion.find(name);
    if (it == mapNameRegion.end()) return -1; else return (*it).second; }
void resetNames() { mapObjName.clear(); mapNameRegion.clear(); }

//void *getObj(unsigned int name);


// ----------------------------------------

ViewMap::~ViewMap()  
{
    // The view vertices must be deleted here as some of them
    // are shared between two shapes:
    for(vector<ViewVertex*>::iterator vv=_VVertices.begin(), vvend=_VVertices.end();
        vv!=vvend;
        vv++)
    {
        delete (*vv);
    }
    _VVertices.clear();

    for(vector<ViewShape*>::iterator vs=_VShapes.begin(),vsend=_VShapes.end();
        vs!=vsend;
        vs++)
    {
        delete (*vs);
    }
    _VShapes.clear();

    for(vector<DebugPoint*>::iterator dp=_debugPoints.begin(); dp != _debugPoints.end(); ++dp)
        delete *dp;

    // deallocate facedata
    for(map<WFace*,FacePOData*>::iterator mfit = _facePOData.begin(); mfit != _facePOData.end(); mfit++)
        delete (*mfit).second;
    
    _facePOData.clear();

    for(multimap<int,InconsistentTri*>::iterator trit = _inconsistentTris.begin(); trit != _inconsistentTris.end(); ++trit)
        delete (*trit).second;
    _inconsistentTris.clear();


    _FEdges.clear();
    _SVertices.clear();
    _VEdges.clear();

    _pInstance = NULL;
}

ViewShape * ViewMap::viewShape(unsigned id) 
{
    int index = _shapeIdToIndex[id];
    return _VShapes[ index ];
}
void  ViewMap::AddViewShape(ViewShape *iVShape) {
    _shapeIdToIndex[iVShape->getId().getFirst()] = _VShapes.size();
    _VShapes.push_back(iVShape);
}
const FEdge * ViewMap::GetClosestFEdge(real x, real y) const
{
    // find the closest of this candidates:
    real minDist = DBL_MAX;
    FEdge * winner = 0;
    for(fedges_container::const_iterator fe=_FEdges.begin(),feend=_FEdges.end();
        fe!=feend;
        fe++)
    {
        Vec2d A((*fe)->vertexA()->point2D()[0], (*fe)->vertexA()->point2D()[1]);
        Vec2d B((*fe)->vertexB()->point2D()[0], (*fe)->vertexB()->point2D()[1]);
        real dist = GeomUtils::distPointSegment<Vec2r>(Vec2r(x,y),A, B);
        if(dist < minDist)
        {
            minDist = dist;
            winner = (*fe);
        }

    }
    if(winner==0)
        return 0;

    return winner;
}

const ViewEdge * ViewMap::GetClosestViewEdge(real x, real y) const
{
    // find the closest of this candidates:
    real minDist = DBL_MAX;
    FEdge * winner = 0;
    for(fedges_container::const_iterator fe=_FEdges.begin(),feend=_FEdges.end();
        fe!=feend;
        fe++)
    {
        Vec2d A((*fe)->vertexA()->point2D()[0], (*fe)->vertexA()->point2D()[1]);
        Vec2d B((*fe)->vertexB()->point2D()[0], (*fe)->vertexB()->point2D()[1]);
        real dist = GeomUtils::distPointSegment<Vec2r>(Vec2r(x,y),A, B);
        if(dist < minDist)
        {
            minDist = dist;
            winner = (*fe);
        }

    }
    if(winner==0)
        return 0;

    return winner->viewedge();
}


TVertex* ViewMap::CreateTVertex(const Vec3r& iA3D, const Vec3r& iA2D, FEdge *iFEdgeA,
                                const Vec3r& iB3D, const Vec3r& iB2D, FEdge *iFEdgeB,
                                const Id& id)
{
    ViewShape *vshapeA = iFEdgeA->viewedge()->viewShape();
    SShape *shapeA = iFEdgeA->shape();
    ViewShape *vshapeB = iFEdgeB->viewedge()->viewShape();
    SShape *shapeB = iFEdgeB->shape();

    SVertex * Ia = shapeA->CreateSVertex(iA3D, iA2D, iFEdgeA->vertexA()->getId());
    SVertex * Ib = shapeB->CreateSVertex(iB3D, iB2D, iFEdgeB->vertexA()->getId());

    // determine which intersection point is nearer to the viewer
    // (note: comparing Z values is currently unreliable; probably a bug somewhere in the geometry engine.)
    // (Note: Rman uses z pointing in +z direction.)

    Vec3r da = Ia->point3D() - SilhouetteGeomEngine::GetViewpoint();
    Vec3r db = Ib->point3D() - SilhouetteGeomEngine::GetViewpoint();

    real dista = da.norm();
    real distb = db.norm();


    TVertex * tvertex;
    if(dista < distb)
        tvertex = new TVertex(Ia, Ib);
    else
        tvertex = new TVertex(Ib,Ia);

    tvertex->SetId(id);
    
    // add these vertices to the view map
    AddViewVertex(tvertex);
    AddSVertex(Ia);
    AddSVertex(Ib);

    // and this T Vertex to the view shapes:
    vshapeA->AddVertex(tvertex);
    if (vshapeA != vshapeB)
        vshapeB->AddVertex(tvertex);

    // check if the two fedges lie on the same face
    // that is, the curves intersect on the surface, not just in the 2D projection
    bool sameFace = false;

    //  FEdgeSmooth * fes = dynamic_cast<FEdgeSmooth*>(iFEdgeB);

    WFace * fA[2] = { iFEdgeA->getFace1(), iFEdgeA->getFace2() };
    WFace * fB[2] = { iFEdgeB->getFace1(), iFEdgeB->getFace2() };

    //  printf("Natures: %d, %d\n", iFEdgeA->getNature(), iFEdgeB->getNature());

    assert(fA[0] != NULL ||fA[1] != NULL);
    assert(fB[0] != NULL ||fB[1] != NULL);

    for(int i=0;i<2;i++)
        if (fA[i] != NULL)
            for(int j=0;j<2;j++)
                if (fB[j] != NULL && fA[i] == fB[j])
                    sameFace = true;
    
    tvertex->SetSameFace(sameFace);
    /*
    printf("============================================\n");
  printf("Ia3D = [%f %f %f 1]', Ib3D = [%f %f %f 1]'\n",
     Ia->point3D().x(),Ia->point3D().y(),Ia->point3D().z(),
     Ib->point3D().x(),Ib->point3D().y(),Ib->point3D().z());
  printf("Ia2D = [%f %f %.16f 1]', Ib2D = [%f %f %.16f 1]'\n",
     Ia->point2D().x(),Ia->point2D().y(),Ia->point2D().z(),
     Ib->point2D().x(),Ib->point2D().y(),Ib->point2D().z());
  printf("Front: %s\n", dista < distb ? "A" : "B");
  printf("da = %f, db = %f\n", da.norm(), db.norm());

  assert (sameFace || ((da.norm() < db.norm()) ^ (olddista >= olddistb)));

  */

    return tvertex;
}


NonTVertex * 
ViewMap::CreateNonTVertex(Vec3r point3D, Vec3r point2D, FEdge * fe)
// this function added together by interpolating some other functions, might have a lot of bugs
{
    // create new ViewVertex and SVertex
    ViewShape * vshape = fe->viewedge()->viewShape();
    SShape * shape = fe->shape();
    SVertex * sv = shape->CreateSVertex(point3D, point2D, fe->vertexA()->getId());
    NonTVertex * vv = new NonTVertex(sv);

    AddViewVertex(vv);
    AddSVertex(sv);
    vshape->AddVertex(vv);

    return vv;
}


ViewVertex * ViewMap::InsertViewVertex(SVertex *iVertex, 
                                       vector<ViewEdge*>& newViewEdges){
    // is there already a vertex here?
    if (iVertex->viewvertex() != NULL)
        return iVertex->viewvertex();

    //  NonTVertex *vva = dynamic_cast<NonTVertex*>(iVertex->viewvertex());
    //  if(vva != 0)
    //    return vva;
    // beacuse it is not already a ViewVertex, this SVertex must have only
    // 2 FEdges. The incoming one still belongs to ioEdge, the outgoing one
    // now belongs to newVEdge
    const vector<FEdge*>& fedges = iVertex->fedges();
    //  assert(fedges.size() == 2);
    if(fedges.size()!=2){
        //    cerr << "ViewMap warning: Can't split the ViewEdge" << endl;
        return 0;
    }
    FEdge * fend(0), * fbegin(0);
    for(vector<FEdge*>::const_iterator fe=fedges.begin(), feend=fedges.end();
        fe!=feend;
        ++fe){
        if((*fe)->vertexB() == iVertex){
            fend   = (*fe);
        }
        if((*fe)->vertexA() == iVertex){
            fbegin = (*fe);
        }
        if((fbegin!=0) && (fend!=0))
            break;
    }

    assert(fbegin != NULL && fend != NULL);

    ViewEdge *ioEdge = fbegin->viewedge();
    ViewShape * vshape = ioEdge->viewShape();

    ViewVertex * oldA = ioEdge->A(), *oldB = ioEdge->B();


    NonTVertex * vva = new NonTVertex(iVertex);

    // if the ViewEdge is a closed loop, we don't create
    // a new VEdge
    if(ioEdge->A() == 0){
        // closed loop
        ioEdge->SetA(vva);
        ioEdge->SetB(vva);
        // update sshape
        vshape->sshape()->RemoveEdgeFromChain(ioEdge->fedgeA());
        vshape->sshape()->RemoveEdgeFromChain(ioEdge->fedgeB());

        ioEdge->SetFEdgeA(fbegin);
        ioEdge->SetFEdgeB(fend);

        // Update FEdges
        fend->SetNextEdge(0);
        fbegin->SetPreviousEdge(0);

        // update new View Vertex:
        vva->AddOutgoingViewEdge(ioEdge);
        vva->AddIncomingViewEdge(ioEdge);

        vshape->sshape()->AddChain(ioEdge->fedgeA());
        vshape->sshape()->AddChain(ioEdge->fedgeB());


        assert(ioEdge->A() != NULL || hasEdge(ioEdge->A(), ioEdge));
        assert(ioEdge->B() != NULL || hasEdge(ioEdge->B(), ioEdge));

    }else{
        // Create new ViewEdge
        ViewEdge * newVEdge = new ViewEdge(vva, ioEdge->B(), fbegin, ioEdge->fedgeB(), vshape);
        newVEdge->SetId(Id(ioEdge->getId().getFirst(), ioEdge->getId().getSecond()+1));
        newVEdge->SetNature(ioEdge->getNature());
        //newVEdge->UpdateFEdges(); // done in the ViewEdge constructor

        oldB->Replace(ioEdge,newVEdge);

        // Update old ViewEdge
        ioEdge->SetB(vva);
        ioEdge->SetFEdgeB(fend);


        // Update FEdges
        fend->SetNextEdge(0);
        fbegin->SetPreviousEdge(0);

        // update new View Vertex:
        vva->AddOutgoingViewEdge(newVEdge);
        vva->AddIncomingViewEdge(ioEdge);
        // update ViewShape
        //vshape->AddEdge(newVEdge);
        // update SShape
        vshape->sshape()->AddChain(fbegin);
        // update ViewMap
        //_VEdges.push_back(newVEdge);
        newViewEdges.push_back(newVEdge);

        assert(ioEdge->A() != NULL || hasEdge(ioEdge->A(), ioEdge));
        assert(ioEdge->B() != NULL || hasEdge(ioEdge->B(), ioEdge));
        assert(newVEdge->A() != NULL || hasEdge(newVEdge->A(), newVEdge));
        assert(newVEdge->B() != NULL || hasEdge(newVEdge->B(), newVEdge));

    }



    // update ViewShape
    vshape->AddVertex(vva);

    // update ViewMap
    _VVertices.push_back(vva);




    //  checkVertex(vva);
    //  checkVertex(oldA);
    //  checkVertex(oldB);

    return vva;
}



//FEdge * ViewMap::Connect(FEdge *ioEdge, SVertex *ioVertex, vector<ViewEdge*>& oNewVEdges){
//  SShape * sshape = ioEdge->shape();
//  FEdge *newFEdge = sshape->SplitEdgeIn2(ioEdge, ioVertex);
//  AddFEdge(newFEdge);
//  InsertViewVertex(ioVertex, oNewVEdges);
//  return  newFEdge;
//}

/**********************************/
/*                                */
/*                                */
/*             TVertex            */
/*                                */
/*                                */
/**********************************/

// is dve1 before dve2 ? (does it have a smaller angle ?)
bool ViewEdgeComp(ViewVertex::directedViewEdge& dve1, ViewVertex::directedViewEdge& dve2){
    FEdge *fe1;
    if(dve1.second)
        fe1 = dve1.first->fedgeB();
    else
        fe1 = dve1.first->fedgeA();
    FEdge *fe2;
    if(dve2.second)
        fe2 = dve2.first->fedgeB();
    else
        fe2 = dve2.first->fedgeA();

    Vec3r V1 = fe1->orientation2d();
    Vec2r v1(V1.x(), V1.y());v1.normalize();
    Vec3r V2 = fe2->orientation2d();
    Vec2r v2(V2.x(), V2.y());v2.normalize();
    if(v1.y() > 0){
        if(v2.y() < 0)
            return true;
        else
            return (v1.x() > v2.x());
    }else{
        if(v2.y() > 0)
            return false;
        else
            return (v1.x() < v2.x());
    }
    return false;
}
void TVertex::SetFrontEdgeA(ViewEdge *iFrontEdgeA, bool incoming) {
    if (!iFrontEdgeA) {
        cerr << "Warning: null pointer passed as argument of TVertex::SetFrontEdgeA()" << endl;
        return;
    }
    _FrontEdgeA = directedViewEdge(iFrontEdgeA, incoming);
    if(!_sortedEdges.empty()){
        edge_pointers_container::iterator dve = _sortedEdges.begin(), dveend = _sortedEdges.end();
        while((dve!=dveend) && ViewEdgeComp(**dve, _FrontEdgeA)){
            ++dve;
        }
        _sortedEdges.insert( dve, &_FrontEdgeA);
    }
    else
        _sortedEdges.push_back(&_FrontEdgeA);

    recreateEdgeList();
}
void TVertex::SetFrontEdgeB(ViewEdge *iFrontEdgeB, bool incoming) {
    if (!iFrontEdgeB) {
        cerr << "Warning: null pointer passed as argument of TVertex::SetFrontEdgeB()" << endl;
        return;
    }
    _FrontEdgeB = directedViewEdge(iFrontEdgeB, incoming);
    if(!_sortedEdges.empty()){
        edge_pointers_container::iterator dve = _sortedEdges.begin(), dveend = _sortedEdges.end();
        while((dve!=dveend) && ViewEdgeComp(**dve, _FrontEdgeB)){
            ++dve;
        }
        _sortedEdges.insert(dve, &_FrontEdgeB);
    }
    else
        _sortedEdges.push_back(&_FrontEdgeB);

    recreateEdgeList();
}
void TVertex::SetBackEdgeA(ViewEdge *iBackEdgeA, bool incoming) {
    if (!iBackEdgeA) {
        cerr << "Warning: null pointer passed as argument of TVertex::SetBackEdgeA()" << endl;
        return;
    }
    _BackEdgeA = directedViewEdge(iBackEdgeA, incoming);
    if(!_sortedEdges.empty()){
        edge_pointers_container::iterator dve = _sortedEdges.begin(), dveend = _sortedEdges.end();
        while((dve!=dveend) && ViewEdgeComp(**dve, _BackEdgeA)){
            ++dve;
        }
        _sortedEdges.insert(dve, &_BackEdgeA);
    }
    else
        _sortedEdges.push_back(&_BackEdgeA);

    recreateEdgeList();
}
void TVertex::SetBackEdgeB(ViewEdge *iBackEdgeB, bool incoming) {
    if (!iBackEdgeB) {
        cerr << "Warning: null pointer passed as argument of TVertex::SetBackEdgeB()" << endl;
        return;
    }
    _BackEdgeB = directedViewEdge(iBackEdgeB, incoming);
    if(!_sortedEdges.empty()){
        edge_pointers_container::iterator dve = _sortedEdges.begin(), dveend = _sortedEdges.end();
        while((dve!=dveend) && ViewEdgeComp(**dve, _BackEdgeB)){
            ++dve;
        }
        _sortedEdges.insert(dve, &_BackEdgeB);
    }
    else
        _sortedEdges.push_back(&_BackEdgeB);

    recreateEdgeList();
}
void TVertex::Replace(ViewEdge *iOld, ViewEdge *iNew)
{
    // theoritically, we only replace edges for which this
    // view vertex is the B vertex
    if((iOld == _FrontEdgeA.first) && (_FrontEdgeA.first->B() == this))
    {
        _FrontEdgeA.first = iNew;
        recreateEdgeList();
        return;
    }
    if((iOld == _FrontEdgeB.first) && (_FrontEdgeB.first->B() == this))
    {
        _FrontEdgeB.first = iNew;
        recreateEdgeList();
        return;
    }
    if((iOld == _BackEdgeA.first) && (_BackEdgeA.first->B() == this))
    {
        _BackEdgeA.first = iNew;
        recreateEdgeList();
        return;
    }
    if((iOld == _BackEdgeB.first) && (_BackEdgeB.first->B() == this))
    {
        _BackEdgeB.first = iNew;
        recreateEdgeList();
        return;
    }
}

void TVertex::recreateEdgeList()
{
    _sortedEdges.clear();
    set<directedViewEdge*> ves;
    if (_FrontEdgeA.first != NULL)
        ves.insert(&_FrontEdgeA);
    if (_FrontEdgeB.first != NULL)
        ves.insert(&_FrontEdgeB);
    if (_BackEdgeA.first != NULL)
        ves.insert(&_BackEdgeA);
    if (_BackEdgeB.first != NULL)
        ves.insert(&_BackEdgeB);

    for(set<directedViewEdge*>::iterator it = ves.begin(); it != ves.end(); ++it)
        _sortedEdges.push_back(*it);
}

/*
  if (ve == NULL)
    return;

  bool found;

  do
    {
      found = false;
    for(edge_pointers_container::iterator it = _sortedEdges.begin(); it != _sortedEdges.end(); ++it)
      if ( (*it)->first == ve)
    {
      // more than one sorted edge can match
      _sortedEdges.erase(it);
      found = true;
      break;
    }
    }
  while (found);

}
  */

/*! iterators access */
ViewVertex::edge_iterator TVertex::edges_begin()
{
    //return edge_iterator(_FrontEdgeA, _FrontEdgeB, _BackEdgeA, _BackEdgeB, _FrontEdgeA);
    return edge_iterator(_sortedEdges.begin(), _sortedEdges.end(), _sortedEdges.begin());
}
ViewVertex::const_edge_iterator TVertex::edges_begin() const
{
    //return const_edge_iterator(_FrontEdgeA, _FrontEdgeB, _BackEdgeA, _BackEdgeB, _FrontEdgeA);
    return const_edge_iterator(_sortedEdges.begin(), _sortedEdges.end(), _sortedEdges.begin());
}
ViewVertex::edge_iterator TVertex::edges_end()
{ 
    //return edge_iterator(_FrontEdgeA, _FrontEdgeB, _BackEdgeA, _BackEdgeB, directedViewEdge(0,true));
    return edge_iterator(_sortedEdges.begin(), _sortedEdges.end(), _sortedEdges.end());
}
ViewVertex::const_edge_iterator TVertex::edges_end() const
{ 
    //return const_edge_iterator(_FrontEdgeA, _FrontEdgeB, _BackEdgeA, _BackEdgeB, directedViewEdge(0, true));
    return const_edge_iterator(_sortedEdges.begin(), _sortedEdges.end(), _sortedEdges.end());
}
ViewVertex::edge_iterator TVertex::edges_iterator(ViewEdge *iEdge)
{ 
    for(edge_pointers_container::iterator it=_sortedEdges.begin(), itend=_sortedEdges.end();
        it!=itend;
        it++)
    {
        if((*it)->first == iEdge)
            return edge_iterator(_sortedEdges.begin(), _sortedEdges.end(), it);
    }
    return edge_iterator(_sortedEdges.begin(), _sortedEdges.end(), _sortedEdges.begin());

    //  directedViewEdge dEdge;
    //  if(_FrontEdgeA.first == iEdge)
    //    dEdge = _FrontEdgeA;
    //  else if(_FrontEdgeB.first == iEdge)
    //    dEdge = _FrontEdgeB;
    //  else if(_BackEdgeA.first == iEdge)
    //    dEdge = _BackEdgeA;
    //  else if(_BackEdgeB.first == iEdge)
    //    dEdge = _BackEdgeB;
    //  return edge_iterator(_FrontEdgeA, _FrontEdgeB, _BackEdgeA, _BackEdgeB, dEdge);
}
ViewVertex::const_edge_iterator TVertex::edges_iterator(ViewEdge *iEdge) const
{ 
    for(edge_pointers_container::const_iterator it=_sortedEdges.begin(), itend=_sortedEdges.end();
        it!=itend;
        it++)
    {
        if((*it)->first == iEdge)
            return const_edge_iterator(_sortedEdges.begin(), _sortedEdges.end(), it);
    }
    return const_edge_iterator(_sortedEdges.begin(), _sortedEdges.end(), _sortedEdges.begin());

    //  directedViewEdge dEdge;
    //  if(_FrontEdgeA.first == iEdge)
    //    dEdge = _FrontEdgeA;
    //  else if(_FrontEdgeB.first == iEdge)
    //    dEdge = _FrontEdgeB;
    //  else if(_BackEdgeA.first == iEdge)
    //    dEdge = _BackEdgeA;
    //  else if(_BackEdgeB.first == iEdge)
    //    dEdge = _BackEdgeB;
    //  return const_edge_iterator(_FrontEdgeA, _FrontEdgeB, _BackEdgeA, _BackEdgeB, dEdge);
}

ViewVertexInternal::orientedViewEdgeIterator TVertex::edgesBegin() {
    return ViewVertexInternal::orientedViewEdgeIterator(_sortedEdges.begin(), _sortedEdges.end(), _sortedEdges.begin());
}
ViewVertexInternal::orientedViewEdgeIterator TVertex::edgesEnd() {
    return ViewVertexInternal::orientedViewEdgeIterator(_sortedEdges.begin(), _sortedEdges.end(), _sortedEdges.end());
}
ViewVertexInternal::orientedViewEdgeIterator TVertex::edgesIterator(ViewEdge *iEdge) {
    for(edge_pointers_container::iterator it=_sortedEdges.begin(), itend=_sortedEdges.end();
        it!=itend;
        it++)
    {
        if((*it)->first == iEdge)
            return ViewVertexInternal::orientedViewEdgeIterator(_sortedEdges.begin(), _sortedEdges.end(), it);
    }
    return ViewVertexInternal::orientedViewEdgeIterator(_sortedEdges.begin(), _sortedEdges.end(), _sortedEdges.begin());
}
/**********************************/
/*                                */
/*                                */
/*             NonTVertex         */
/*                                */
/*                                */
/**********************************/

void NonTVertex::AddOutgoingViewEdge(ViewEdge * iVEdge){
    // let's keep the viewedges ordered in CCW order
    // in the 2D image plan
    directedViewEdge idve(iVEdge, false);
    if(!_ViewEdges.empty()){
        edges_container::iterator dve = _ViewEdges.begin(), dveend = _ViewEdges.end();
        while((dve!=dveend) && ViewEdgeComp(*dve, idve)){
            ++dve;
        }
        _ViewEdges.insert(dve, idve);
    }
    else
        _ViewEdges.push_back(idve);
}

void NonTVertex::AddIncomingViewEdge(ViewEdge * iVEdge){
    // let's keep the viewedges ordered in CCW order
    // in the 2D image plan
    directedViewEdge idve(iVEdge, true);
    if(!_ViewEdges.empty()){
        edges_container::iterator dve = _ViewEdges.begin(), dveend = _ViewEdges.end();
        while((dve!=dveend) && ViewEdgeComp(*dve, idve)){
            ++dve;
        }
        _ViewEdges.insert(dve, idve);
    }
    else
        _ViewEdges.push_back(idve);
}

/*! iterators access */
ViewVertex::edge_iterator NonTVertex::edges_begin()
{ 
    return edge_iterator(_ViewEdges.begin(), _ViewEdges.end(), _ViewEdges.begin());
}
ViewVertex::const_edge_iterator NonTVertex::edges_begin() const
{ 
    return const_edge_iterator(_ViewEdges.begin(), _ViewEdges.end(), _ViewEdges.begin());
}
ViewVertex::edge_iterator NonTVertex::edges_end()
{
    return edge_iterator(_ViewEdges.begin(), _ViewEdges.end(), _ViewEdges.end());
}
ViewVertex::const_edge_iterator NonTVertex::edges_end() const
{
    return const_edge_iterator(_ViewEdges.begin(), _ViewEdges.end(), _ViewEdges.end());
}
ViewVertex::edge_iterator NonTVertex::edges_iterator(ViewEdge *iEdge)
{
    for(edges_container::iterator it=_ViewEdges.begin(), itend=_ViewEdges.end();
        it!=itend;
        it++)
    {
        if((it)->first == iEdge)
            return edge_iterator(_ViewEdges.begin(), _ViewEdges.end(), it);
    }
    return edge_iterator(_ViewEdges.begin(), _ViewEdges.end(), _ViewEdges.begin());
}
ViewVertex::const_edge_iterator NonTVertex::edges_iterator(ViewEdge *iEdge) const
{
    for(edges_container::const_iterator it=_ViewEdges.begin(), itend=_ViewEdges.end();
        it!=itend;
        it++)
    {
        if((it)->first == iEdge)
            return const_edge_iterator(_ViewEdges.begin(), _ViewEdges.end(), it);
    }
    return const_edge_iterator(_ViewEdges.begin(), _ViewEdges.end(), _ViewEdges.begin());
}

ViewVertexInternal::orientedViewEdgeIterator NonTVertex::edgesBegin() {
    return ViewVertexInternal::orientedViewEdgeIterator(_ViewEdges.begin(), _ViewEdges.end(), _ViewEdges.begin());
}
ViewVertexInternal::orientedViewEdgeIterator NonTVertex::edgesEnd() {
    return ViewVertexInternal::orientedViewEdgeIterator(_ViewEdges.begin(), _ViewEdges.end(), _ViewEdges.end());
}
ViewVertexInternal::orientedViewEdgeIterator NonTVertex::edgesIterator(ViewEdge *iEdge) {
    for(edges_container::iterator it=_ViewEdges.begin(), itend=_ViewEdges.end();
        it!=itend;
        it++)
    {
        if((it)->first == iEdge)
            return ViewVertexInternal::orientedViewEdgeIterator(_ViewEdges.begin(), _ViewEdges.end(), it);
    }
    return ViewVertexInternal::orientedViewEdgeIterator(_ViewEdges.begin(), _ViewEdges.end(), _ViewEdges.begin());
}
/**********************************/
/*                                */
/*                                */
/*             ViewEdge           */
/*                                */
/*                                */
/**********************************/

real ViewEdge::getLength2D() const 
{
    float length = 0.f;
    ViewEdge::const_fedge_iterator itlast = fedge_iterator_last();
    ViewEdge::const_fedge_iterator it = fedge_iterator_begin(), itend=fedge_iterator_end();
    Vec2r seg;
    do{
        seg = Vec2r((*it)->orientation2d()[0], (*it)->orientation2d()[1]);
        length += seg.norm();
        ++it;
    }while((it!=itend) && (it!=itlast));
    return length;
}


//! view edge iterator
ViewEdge::edge_iterator ViewEdge::ViewEdge_iterator() {return edge_iterator(this);}
ViewEdge::const_edge_iterator ViewEdge::ViewEdge_iterator() const {return const_edge_iterator((ViewEdge*)this);}
//! feature edge iterator
ViewEdge::fedge_iterator ViewEdge::fedge_iterator_begin() {return fedge_iterator(this->_FEdgeA, this->_FEdgeB);}
ViewEdge::const_fedge_iterator ViewEdge::fedge_iterator_begin() const {return const_fedge_iterator(this->_FEdgeA, this->_FEdgeB);}
ViewEdge::fedge_iterator ViewEdge::fedge_iterator_last() {return fedge_iterator(this->_FEdgeB, this->_FEdgeB);}
ViewEdge::const_fedge_iterator ViewEdge::fedge_iterator_last() const {return const_fedge_iterator(this->_FEdgeB, this->_FEdgeB);}
ViewEdge::fedge_iterator ViewEdge::fedge_iterator_end() {return fedge_iterator(0, this->_FEdgeB);}
ViewEdge::const_fedge_iterator ViewEdge::fedge_iterator_end() const {return const_fedge_iterator(0, this->_FEdgeB);}
//! embedding vertex iterator
ViewEdge::const_vertex_iterator ViewEdge::vertices_begin() const {return const_vertex_iterator(this->_FEdgeA->vertexA(), 0, _FEdgeA);}
ViewEdge::vertex_iterator ViewEdge::vertices_begin() {return vertex_iterator(this->_FEdgeA->vertexA(), 0, _FEdgeA);}
ViewEdge::const_vertex_iterator ViewEdge::vertices_last() const {return const_vertex_iterator(this->_FEdgeB->vertexB(), _FEdgeB, 0);}
ViewEdge::vertex_iterator ViewEdge::vertices_last() {return vertex_iterator(this->_FEdgeB->vertexB(), _FEdgeB, 0);}
ViewEdge::const_vertex_iterator ViewEdge::vertices_end() const {return const_vertex_iterator(0, _FEdgeB, 0);}
ViewEdge::vertex_iterator ViewEdge::vertices_end() {return vertex_iterator(0, _FEdgeB, 0);}


Interface0DIterator ViewEdge::verticesBegin() {
    Interface0DIterator ret(new ViewEdgeInternal::SVertexIterator(this->_FEdgeA->vertexA(), this->_FEdgeA->vertexA(), 0, _FEdgeA, 0.f));
    return ret;
}

Interface0DIterator ViewEdge::verticesEnd() {
    Interface0DIterator ret(new ViewEdgeInternal::SVertexIterator(0, this->_FEdgeA->vertexA(), _FEdgeB, 0, getLength2D()));
    return ret;
}

Interface0DIterator ViewEdge::pointsBegin(float t) {
    return verticesBegin();
}

Interface0DIterator ViewEdge::pointsEnd(float t) {
    return verticesEnd();
}



/**********************************/
/*                                */
/*                                */
/*             ViewShape          */
/*                                */
/*                                */
/**********************************/


ViewShape::~ViewShape()
{
    _Vertices.clear();

    if(!(_Edges.empty()))
    {
        // debugging: checking for duplicates
        //    set<ViewEdge*> s;
        //for(vector<ViewEdge*>::iterator e2=_Edges.begin(); e2 != _Edges.end(); ++e2)
        //  {
        //assert (s.find(*e2) == s.end());
        //	s.insert(*e2);
        //      }


        for(vector<ViewEdge*>::iterator e=_Edges.begin(), eend=_Edges.end();
            e!=eend;
            e++)
        {
            delete (*e);
        }

        _Edges.clear();
    }

    if(0 != _SShape)
    {
        delete _SShape;
        _SShape = 0;
    }
}

void ViewShape::RemoveEdge(ViewEdge * iViewEdge)
{
    FEdge * fedge = iViewEdge->fedgeA();
    for(vector<ViewEdge*>::iterator ve=_Edges.begin(),veend=_Edges.end();
        ve!=veend;
        ve++)
    {
        if(iViewEdge == (*ve))
        {
            _Edges.erase(ve);
            _SShape->RemoveEdge(fedge);
            break;
        }
    }
}

void ViewShape::RemoveVertex(ViewVertex * iViewVertex)
{
    for(vector<ViewVertex*>::iterator vv=_Vertices.begin(), vvend=_Vertices.end();
        vv!=vvend;
        vv++)
    {
        if(iViewVertex == (*vv))
        {
            _Vertices.erase(vv);
            break;
        }
    }
}
void ViewMap::RemoveVertex(ViewVertex * iViewVertex)
{
    for(vector<ViewVertex*>::iterator vv=_VVertices.begin(), vvend=_VVertices.end();
        vv!=vvend;
        vv++)
    {
        if(iViewVertex == (*vv))
        {
            _VVertices.erase(vv);
            break;
        }
    }
}

/**********************************/
/*                                */
/*                                */
/*             ViewEdge           */
/*                                */
/*                                */
/**********************************/


void ViewEdge::UpdateFEdges()
{
    FEdge *currentEdge = _FEdgeA;
    do
    {
        currentEdge->SetViewEdge(this);
        currentEdge = currentEdge->nextEdge();
    }while((currentEdge != NULL) && (currentEdge!= _FEdgeB));
    // last one
    _FEdgeB->SetViewEdge(this);

}

// merge a set of t-vertices into a single non-t-vertex, discarding all information about front-back and ignoring geometric differences between the points
void ViewMap::MergeTVertices(set<TVertex*> & tvGroup)
{
    TVertex * tv1 = *(tvGroup.begin());
    SVertex * sv1 = tv1->frontSVertex();

    NonTVertex * ntv = new NonTVertex(sv1);
    AddViewVertex(ntv);

    //  ViewShape * vshape = tv1->viewShape();
    //  vshape->AddVertex(ntv);

    // TODO: set nature


    // copy over all the edges
    for(set<TVertex*>::iterator it = tvGroup.begin(); it != tvGroup.end(); ++it)
    {
        TVertex * tv2 = *it;
        vector<pair<ViewEdge*,bool> > edges;

        //      for(TVertex::edge_iterator eit = (*it)->edges_begin(); eit != (*it)->edges_end(); ++eit)
        //	edges.push_back(*eit);

        if ( tv2->frontEdgeA().first != NULL)
            edges.push_back(pair<ViewEdge*,bool>(tv2->frontEdgeA().first, tv2->frontEdgeA().second));

        if ( tv2->frontEdgeB().first != NULL)
            edges.push_back(pair<ViewEdge*,bool>(tv2->frontEdgeB().first, tv2->frontEdgeB().second));

        if ( tv2->backEdgeA().first != NULL)
            edges.push_back(pair<ViewEdge*,bool>(tv2->backEdgeA().first, tv2->backEdgeA().second));

        if ( tv2->backEdgeB().first != NULL)
            edges.push_back(pair<ViewEdge*,bool>(tv2->backEdgeB().first, tv2->backEdgeB().second));

        for(vector<pair<ViewEdge*,bool> >::iterator eit = edges.begin(); eit != edges.end(); ++eit)
        {
            ViewEdge * ve = (*eit).first;
            ntv->AddViewEdge( ve, (*eit).second);
            // update pointers;

            if (ve->A() == tv2)
                ve->SetA(ntv);

            if (ve->B() == tv2)
                ve->SetB(ntv);

            if (ve->fedgeA()->vertexA() == tv2->frontSVertex() || ve->fedgeA()->vertexA() == tv2->backSVertex())
            {
                ve->fedgeA()->SetVertexA(sv1);
                sv1->AddFEdge(ve->fedgeA());
            }

            if (ve->fedgeA()->vertexB() == tv2->frontSVertex() || ve->fedgeA()->vertexB() == tv2->backSVertex())
            {
                ve->fedgeA()->SetVertexB(sv1);
                sv1->AddFEdge(ve->fedgeA());
            }

            if (ve->fedgeB()->vertexA() == tv2->frontSVertex() || ve->fedgeB()->vertexA() == tv2->backSVertex())
            {
                ve->fedgeB()->SetVertexA(sv1);
                sv1->AddFEdge(ve->fedgeB());
            }

            if (ve->fedgeB()->vertexB() == tv2->frontSVertex() || ve->fedgeB()->vertexB() == tv2->backSVertex())
            {
                ve->fedgeB()->SetVertexB(sv1);
                sv1->AddFEdge(ve->fedgeB());
            }
        }
    }

    // delete the old tvertices

    set<SVertex*> sverts;

    for(set<TVertex*>::iterator it = tvGroup.begin(); it != tvGroup.end(); ++it)
    {
        TVertex *tv = *it;
        if (tv->frontSVertex() != sv1)
            sverts.insert(tv->frontSVertex());
        if (tv->backSVertex() != sv1)
            sverts.insert(tv->backSVertex());
    }

    for(vector<ViewShape*>::iterator vsit=_VShapes.begin(); vsit != _VShapes.end(); ++vsit)
    {
        for(set<SVertex*>::iterator it = sverts.begin(); it != sverts.end(); ++it)
            (*vsit)->sshape()->RemoveVertex(*it);

        for(set<TVertex*>::iterator it = tvGroup.begin(); it != tvGroup.end(); ++it)
            (*vsit)->RemoveVertex(*it);
    }

    for(set<SVertex*>::iterator it = sverts.begin(); it != sverts.end(); ++it)
    {
        for(vector<SVertex*>::iterator it2 = _SVertices.begin(); it2 != _SVertices.end(); ++it2)
            if ((*it2) == (*it))
            {
                _SVertices.erase(it2);
                break;
            }

        delete *it;
    }
    
    for(set<TVertex*>::iterator it = tvGroup.begin(); it != tvGroup.end(); ++it)
    {
        RemoveVertex(*it);
        //      _VVertices.erase(_VVertices.find(*it));
        delete *it;
    }

}


void ViewMap::MergeNonTVertices(NonTVertex * ntv1, NonTVertex * ntv2)
{
    // it is assumed that they are topologically and geometrically the same point

    // --------- take everything from ntv2 and put it in ntv1 --------------

    SVertex * sv1 = ntv1->svertex();
    SVertex * sv2 = ntv2->svertex();


    ntv1->setNature ( ntv1->getNature() | ntv2->getNature() );

    for(NonTVertex::edges_container::iterator eit = ntv2->viewedges().begin();
        eit != ntv2->viewedges().end(); ++eit)
    {
        ViewEdge * ve = (*eit).first;
        bool incoming = (*eit).second;

        ntv1->AddViewEdge(ve,incoming);

        // modify all the pointers from ve to go to ntv1 instead of ntv2, and sv1 instead of sv2

        if (ve->A() == ntv2)
        {
            ve->SetA(ntv1);
        }

        if (ve->fedgeA()->vertexA() == sv2)
        {
            ve->fedgeA()->SetVertexA(sv1);
            sv1->AddFEdge(ve->fedgeA());
        }

        if (ve->fedgeA()->vertexB() == sv2)
        {
            ve->fedgeA()->SetVertexB(sv1);
            sv1->AddFEdge(ve->fedgeA());
        }


        if (ve->B() == ntv2)
        {
            ve->SetB(ntv1);
        }

        if (ve->fedgeB()->vertexA() == sv2)
        {
            ve->fedgeB()->SetVertexA(sv1);
            sv1->AddFEdge(ve->fedgeB());
        }

        if (ve->fedgeB()->vertexB() == sv2)
        {
            ve->fedgeB()->SetVertexB(sv1);
            sv1->AddFEdge(ve->fedgeB());
        }

    }

    // remove the redundant vertex
    viewvertices_container::iterator vit;
    svertices_container::iterator sit;

    for(vit = _VVertices.begin(); vit != _VVertices.end(); ++vit)
        if ( (*vit) == ntv2)
            break;

    for(sit = _SVertices.begin(); sit != _SVertices.end(); ++sit)
        if ( (*sit) == sv2)
            break;

    assert( vit != _VVertices.end() && sit != _SVertices.end());

    for(vector<ViewShape*>::iterator vsit=_VShapes.begin(); vsit != _VShapes.end(); ++vsit)
    {
        (*vsit)->RemoveVertex(ntv2);
        (*vsit)->sshape()->RemoveVertex(sv2);
    }

    _VVertices.erase(vit);
    _SVertices.erase(sit);


    delete ntv2;
    delete sv2;
}


void ViewMap::MergeVertices(ViewVertex * v1, ViewVertex * v2)
{
    NonTVertex * ntv1 = dynamic_cast<NonTVertex*>(v1);
    NonTVertex * ntv2 = dynamic_cast<NonTVertex*>(v2);

    // save time by using the functions I've already written
    if (ntv1 != NULL && ntv2 != NULL)
    {
        //        printf("a\n");
        //        fflush(stdout);
        MergeNonTVertices(ntv1, ntv2);
        return;
    }

    TVertex * tv1 = dynamic_cast<TVertex*>(v1);
    TVertex * tv2 = dynamic_cast<TVertex*>(v2);

    if (tv1 != NULL && tv2 != NULL)
    {
        //        printf("b\n");
        //        fflush(stdout);
        set<TVertex*> tvs;
        tvs.insert(tv1);
        tvs.insert(tv2);
        MergeTVertices(tvs);
        return;
    }

    // we have one t-vertex and one non-t-vertex.  merge everything from the t-vertex into the non-t-vertex

    TVertex * tv = (tv1 != NULL ? tv1 : tv2);
    NonTVertex * ntv = (ntv1 != NULL ? ntv1 : ntv2);
    assert(tv != NULL);
    assert(ntv != NULL);
    SVertex * sv = ntv->svertex();
    
    vector<pair<ViewEdge*,bool> > edges;

    //        printf("c\n");
    //        fflush(stdout);

    //      for(TVertex::edge_iterator eit = (*it)->edges_begin(); eit != (*it)->edges_end(); ++eit)
    //	edges.push_back(*eit);

    if ( tv->frontEdgeA().first != NULL)
        edges.push_back(pair<ViewEdge*,bool>(tv->frontEdgeA().first, tv->frontEdgeA().second));

    if ( tv->frontEdgeB().first != NULL)
        edges.push_back(pair<ViewEdge*,bool>(tv->frontEdgeB().first, tv->frontEdgeB().second));
    
    if ( tv->backEdgeA().first != NULL)
        edges.push_back(pair<ViewEdge*,bool>(tv->backEdgeA().first, tv->backEdgeA().second));
    
    if ( tv->backEdgeB().first != NULL)
        edges.push_back(pair<ViewEdge*,bool>(tv->backEdgeB().first, tv->backEdgeB().second));

    for(vector<pair<ViewEdge*,bool> >::iterator eit = edges.begin(); eit != edges.end(); ++eit)
    {
        ViewEdge * ve = (*eit).first;
        ntv->AddViewEdge( ve, (*eit).second);
        // update pointers;

        if (ve->A() == tv)
            ve->SetA(ntv);

        if (ve->B() == tv)
            ve->SetB(ntv);

        if (ve->fedgeA()->vertexA() == tv->frontSVertex() || ve->fedgeA()->vertexA() == tv->backSVertex())
        {
            ve->fedgeA()->SetVertexA(sv);
            sv->AddFEdge(ve->fedgeA());
        }

        if (ve->fedgeA()->vertexB() == tv->frontSVertex() || ve->fedgeA()->vertexB() == tv->backSVertex())
        {
            ve->fedgeA()->SetVertexB(sv);
            sv->AddFEdge(ve->fedgeA());
        }

        if (ve->fedgeB()->vertexA() == tv->frontSVertex() || ve->fedgeB()->vertexA() == tv->backSVertex())
        {
            ve->fedgeB()->SetVertexA(sv);
            sv->AddFEdge(ve->fedgeB());
        }

        if (ve->fedgeB()->vertexB() == tv->frontSVertex() || ve->fedgeB()->vertexB() == tv->backSVertex())
        {
            ve->fedgeB()->SetVertexB(sv);
            sv->AddFEdge(ve->fedgeB());
        }
    }

    //        printf("d\n");
    //        fflush(stdout);

    // delete the old vertices

    SVertex * svFront = tv->frontSVertex();
    SVertex * svBack = tv->backSVertex();

    assert(svFront != NULL && svBack != NULL);

    for(vector<ViewShape*>::iterator vsit=_VShapes.begin(); vsit != _VShapes.end(); ++vsit)
    {
        (*vsit)->sshape()->RemoveVertex(svFront);
        if (svBack != svFront)
            (*vsit)->sshape()->RemoveVertex(svBack);

        (*vsit)->RemoveVertex(tv);
    }

    //        printf("e\n");
    //        fflush(stdout);

    for(vector<SVertex*>::iterator it = _SVertices.begin(); it != _SVertices.end(); ++it)
        if ( (*it) == svFront)
        {
            _SVertices.erase(it);
            break;
        }

    //   printf("f\n");
    //   fflush(stdout);

    if (svBack != svFront)
        for(vector<SVertex*>::iterator it = _SVertices.begin(); it != _SVertices.end(); ++it)
            if ( (*it) == svBack)
            {
                _SVertices.erase(it);
                break;
            }

    delete svFront;
    if (svBack != svFront)
        delete svBack;

    RemoveVertex(tv);
    delete tv;
}


TVertex * ViewMap::MergeSNonTVertices(SVertex *silSV, NonTVertex * ntv)
// input: a silhouette SVertex which lies on an existing silhouette viewedge
//        an old NonTVertex lying at the end of a POCuspSI viewedge
// result: create a new TVertex and split the silhouette viewedge; update all the pointers
//
// Despite the generic-sounding name, this function is somewhat specialized for one specific case.
{
    // extract data from the old NonTVertex.  the caller must delete it, though, and add the new vertex,
    // because this is called from within a loop over the list of vvertices, which we can't modify within the loop



    // many assertions here because I was debugging, but most can be removed if it works


    SVertex * poSV = ntv->svertex();

    assert(ntv->viewedges().size() == 1);

    // There is a weird case in which the silhouette vertex might already have a NonTVertex.
    // I'm not sure it should be happening (might be a bug), but safer to just handle it.
    NonTVertex * oldSilNTV = NULL;

    if (silSV->viewvertex() != NULL)
    {
        //      TVertex * debugNTV = dynamic_cast<TVertex*>(oldSilNTV);

        assert(dynamic_cast<NonTVertex*>(silSV->viewvertex())!=NULL);
        oldSilNTV = (NonTVertex*)silSV->viewvertex();
    }

    pair<ViewEdge*,bool> poEdgeD = ntv->viewedges()[0];
    ViewEdge *poEdge = poEdgeD.first;

    bool VEA = poEdge->A() == ntv;
    assert(VEA || poEdge->B() == ntv);
    assert(VEA != poEdgeD.second);   // not sure exactly what poEdgeD.second (incoming/outgoing) means

    // make the new viewvertex

    bool silInFront = (silSV->getPoint3D() - SilhouetteGeomEngine::GetViewpoint()).norm() <
            (poSV->getPoint3D() - SilhouetteGeomEngine::GetViewpoint()).norm();

    if (silInFront) // the POSI curve won't be visible anyway.
        return NULL;

    TVertex * vv;
    if (silInFront)
        vv = new TVertex(silSV, poSV);
    else
        vv = new TVertex(poSV, silSV);

    //  printf("1 vv->numEdges: %d\n", vv->numEdges());
    assert(vv->numEdges() <= 4);

    vv->SetSameFace(false);

    // connect with the newVEdge

    if (VEA)
        poEdge->SetA(vv);
    else
        poEdge->SetB(vv);
    poEdge->viewShape()->AddVertex(vv);

    //  printf("2 vv->numEdges: %d\n", vv->numEdges());
    assert(vv->numEdges() <= 4);

    // split the silhouette edge and connect

    assert(silSV->fedges().size() == 2);

    ViewEdge * silEdge = silSV->fedges()[0]->viewedge();
    ViewEdge * newSilEdge = NULL;

    assert(silEdge->A() == NULL || hasEdge(silEdge->A(), silEdge));
    assert(silEdge->B() == NULL || hasEdge(silEdge->B(), silEdge));
    assert(silEdge->A() == NULL || silEdge->A() != silEdge->B() || hasEdgeTwice(silEdge->A(),silEdge));

    bool isLoop = oldSilNTV != NULL && (silEdge->A() == oldSilNTV && silEdge->B() == oldSilNTV);

    if (oldSilNTV == NULL)
    {
        assert(silSV->fedges()[0]->viewedge() == silSV->fedges()[1]->viewedge());
        assert(silSV->fedges()[0]->nextEdge() == silSV->fedges()[1] ||
               silSV->fedges()[0]->previousEdge() == silSV->fedges()[1]);

        assert(silEdge->A() == NULL || silEdge->A() != silEdge->B() || hasEdgeTwice(silEdge->A(),silEdge));

        silEdge->viewShape()->SplitEdge(vv,silEdge,newSilEdge,silInFront);

        if (newSilEdge != silEdge)
            AddViewEdge(newSilEdge);

        assert(silEdge->A() == NULL || silEdge->A() != silEdge->B() || hasEdgeTwice(silEdge->A(),silEdge));
        assert(newSilEdge->A() == NULL || newSilEdge->A() != newSilEdge->B() || hasEdgeTwice(silEdge->A(),newSilEdge));
    }
    else
    {
        int numEdges = 0;
        // update all the edges that point to the viewvertex
        for(NonTVertex::edge_iterator eit = oldSilNTV->edges_begin(); eit != oldSilNTV->edges_end();++eit)
        {
            ViewEdge * ve = (*eit).first;
            if (ve->A() == oldSilNTV)
                ve->SetA(vv);
            if (ve->B() == oldSilNTV)
                ve->SetB(vv);

            if (ve != silEdge)
                newSilEdge = ve;
            numEdges ++;
        }

        // can't currently handle more than 2 edges
        assert(numEdges <= 2);
    }


    assert(silEdge->fedgeA()->viewedge() == silEdge);

    assert(newSilEdge == NULL || newSilEdge->fedgeA()->viewedge() == newSilEdge);


    AddViewVertex(vv);

    //  printf("3 vv->numEdges: %d\n", vv->numEdges());
    assert(vv->numEdges() <= 4);

    // update the viewvertex data

    assert(!(isLoop && newSilEdge != NULL));

    if(silInFront)
    {
        vv->SetFrontEdgeA(silEdge,true);
        if (newSilEdge != NULL)
            vv->SetFrontEdgeB(newSilEdge,false);
        else if (isLoop)
            vv->SetFrontEdgeB(silEdge,false);
        vv->SetBackEdgeA(poEdge, poEdgeD.second);
    }
    else
    {
        vv->SetFrontEdgeA(poEdge, poEdgeD.second);
        vv->SetBackEdgeA(silEdge, true);
        if (newSilEdge != NULL)
            vv->SetBackEdgeB(newSilEdge, false);
        else if (isLoop)
            vv->SetBackEdgeB(silEdge, false);
    }

    poSV->SetViewVertex(vv);
    silSV->SetViewVertex(vv);

    vv->SetId(-1);  // not sure if this matters

    //  printf("4 vv->numEdges: %d\n", vv->numEdges());
    assert(vv->numEdges() <= 4);

    checkVertex(vv);


    // delete the old vertecies
    //  printf("removing ntv: %08X\n", ntv);
    RemoveVertex(ntv);
    for(vector<pair<ViewEdge*,bool> >::iterator it = ntv->viewedges().begin();
        it != ntv->viewedges().end(); ++it)
    {
        (*it).first->viewShape()->RemoveVertex(ntv);
        // TODO:	  (*it).first->viewShape()->sshape()->RemoveVertex(ntv);
    }


    // *** SOME DEBUGGING CHECKS
    for(NonTVertex::edge_iterator eit = ntv->edges_begin(); eit != ntv->edges_end(); ++eit)
    {
        ViewEdge *ve = (*eit).first;
        assert(ve->A() != ntv && ve->B() != ntv);
        checkVertex(ve->A());
        checkVertex(ve->B());
    }

    delete ntv;


    if (newSilEdge != NULL)
    {
        assert(newSilEdge->A() == NULL || hasEdge(newSilEdge->A(), newSilEdge));
        assert(newSilEdge->B() == NULL || hasEdge(newSilEdge->B(), newSilEdge));
    }

    //  printf("oldSilNTV: %08X\n", oldSilNTV);


    if (oldSilNTV != NULL)
    {
        /*      // debugging check
      for(NonTVertex::edge_iterator eit = ntv->edges_begin(); eit != ntv->edges_end(); ++eit)
    {
      ViewEdge *ve = (*eit).first;
      assert(ve->A() != ntv && ve->B() != ntv);
      checkVertex(ve->A());
      checkVertex(ve->B());
    }
      */

        RemoveVertex(oldSilNTV);
        for(vector<pair<ViewEdge*,bool> >::iterator it = oldSilNTV->viewedges().begin();
            it != oldSilNTV->viewedges().end(); ++it)
        {
            (*it).first->viewShape()->RemoveVertex(oldSilNTV);
            // TODO:	  (*it).first->viewShape()->sshape()->RemoveVertex(oldSilNTV);
        }
        delete oldSilNTV;
    }


    assert(silEdge->A() == NULL || hasEdge(silEdge->A(), silEdge));
    assert(silEdge->B() == NULL || hasEdge(silEdge->B(), silEdge));
    assert(poEdge->A() == NULL || hasEdge(poEdge->A(), poEdge));
    assert(poEdge->B() == NULL || hasEdge(poEdge->B(), poEdge));
    assert(silEdge->A() == NULL || silEdge->A() != silEdge->B() || hasEdgeTwice(silEdge->A(),silEdge));
    checkVertex(silEdge->A());
    checkVertex(silEdge->B());
    checkVertex(poEdge->A());
    checkVertex(poEdge->B());
    if (newSilEdge != NULL)
    {
        assert(newSilEdge->A() == NULL || hasEdge(newSilEdge->A(), newSilEdge));
        assert(newSilEdge->B() == NULL || hasEdge(newSilEdge->B(), newSilEdge));
        checkVertex(newSilEdge->A());
        checkVertex(newSilEdge->B());
        assert(newSilEdge->A() == NULL || newSilEdge->A() != newSilEdge->B() ||
                hasEdgeTwice(newSilEdge->A(),newSilEdge));
    }
    checkVertex(vv);



    /*
  for(NonTVertex::edge_iterator eit = vv->edges_begin(); eit != vv->edges_end(); ++eit)
    {
      ViewEdge *ve = (*eit).first;
      assert(ve->A() == vv || ve->B() == vv);

      checkVertex(ve->A());
      checkVertex(ve->B());
    }
  */
}







void ViewShape::SplitEdge(FEdge *fe,
                          const vector<TVertex*>& iViewVertices,
                          vector<FEdge*>& ioNewEdges,
                          vector<ViewEdge*>& ioNewViewEdges)
{ 
    ViewEdge *vEdge = fe->viewedge();
    
    // Assumign the view vertices are sorted from farther to closer to fe->vertexA

    SVertex *sv, *sv2;
    ViewVertex *vva, *vvb;
    vector<TVertex*>::const_iterator vv, vvend;
    for(vv=iViewVertices.begin(), vvend = iViewVertices.end();
        vv!=vvend;
        vv++)
    {
        // Add the viewvertices to the ViewShape
        AddVertex((*vv));

        // retrieve the correct SVertex from the view vertex
        //--------------------------------------------------
        sv = (*vv)->frontSVertex();
        sv2 = (*vv)->backSVertex();

        if(sv->shape() != sv2->shape())
        {
            if(sv->shape() != _SShape)
                sv = sv2;
        }
        else
        {
            // if the shape is the same we can safely differ
            // the two vertices using their ids:
            if(sv->getId() != fe->vertexA()->getId())
                sv = sv2;
        }

        assert(sv->getId() == fe->vertexA()->getId());

        vva = vEdge->A();
        vvb = vEdge->B();

        // We split Fedge AB into AA' and A'B. A' and A'B are created.
        // AB becomes (address speaking) AA'. B is updated.
        //--------------------------------------------------
        SShape * shape = fe->shape();

        // a new edge, A'B is created.
        FEdge *newEdge = shape->SplitEdgeIn2(fe, sv);
        ioNewEdges.push_back(newEdge);

        ViewEdge *newVEdge = NULL;

        if((vva == 0) || (vvb == 0)) // that means we're dealing with a closed viewedge (loop)
        {
            // remove the chain that was starting by the fedge A of vEdge (which is different from fe !!!!)
            shape->RemoveEdgeFromChain(vEdge->fedgeA());
            // we set
            vEdge->SetA(*vv);
            vEdge->SetB(*vv);
            vEdge->SetFEdgeA(newEdge);
            //FEdge *previousEdge = newEdge->previousEdge();
            vEdge->SetFEdgeB(fe);
            newVEdge = vEdge;
            vEdge->fedgeA()->SetViewEdge(newVEdge);
        }
        else
        {

            // while we create the view edge, it updates the "ViewEdge" pointer
            // of every underlying FEdges to this.
            newVEdge = new ViewEdge((*vv),vvb);//, newEdge, vEdge->fedgeB());
            newVEdge->SetNature((fe)->getNature());
            newVEdge->SetFEdgeA(newEdge);
            //newVEdge->SetFEdgeB(fe);
            // If our original viewedge is made of one FEdge,
            // then
            if (fe == vEdge->fedgeB())
                //        if((vEdge->fedgeA() == vEdge->fedgeB()) || (fe == vEdge->fedgeB()))  //Aaron commented out this line
                newVEdge->SetFEdgeB(newEdge);
            else
                newVEdge->SetFEdgeB(vEdge->fedgeB()); //MODIF

            Id * newId = vEdge->splittingId();
            if(newId == 0){
                newId = new Id(vEdge->getId());
                vEdge->setSplittingId(newId);
            }
            newId->setSecond(newId->getSecond()+1);
            newVEdge->SetId(*newId);
            newVEdge->setSplittingId(newId);
            //        Id id(vEdge->getId().getFirst(), vEdge->getId().getSecond()+1);
            //        newVEdge->SetId(vEdge->getId());
            //        vEdge->SetId(id);

            // note: this can create redundant IDs, right?


            AddEdge(newVEdge); // here this shape is set as the edge's shape

            // add new edge to the list of new edges passed as argument:
            ioNewViewEdges.push_back(newVEdge);



            if(0 != vvb)
                vvb->Replace((vEdge), newVEdge);

            // we split the view edge:
            vEdge->SetB((*vv));
            vEdge->SetFEdgeB(fe); //MODIF

            // Update fedges so that they point to the new viewedge:
            newVEdge->UpdateFEdges();

            // added by Aaron:
            //	newVEdge->SetA(*vv);
            //	newVEdge->SetB(vvb);
        }

        // check whether this vertex is a front vertex or a back
        // one

        //      if ( (*vv)->sameFace() ) // just trying this out
        //	{

        if(sv == (*vv)->frontSVertex())
        {
            // -- View Vertex A' --
            (*vv)->SetFrontEdgeA(vEdge, true);
            (*vv)->SetFrontEdgeB(newVEdge, false);

        }
        else
        {
            assert( sv == (*vv)->backSVertex());

            // -- View Vertex A' --
            (*vv)->SetBackEdgeA(vEdge, true);
            (*vv)->SetBackEdgeB(newVEdge, false);
        }
    }
} 




// added by Aaron, virtually the same as the above SplitEdge; they could probably be unified
void ViewShape::SplitEdge(FEdge *fe,
                          const vector<NonTVertex*>& iViewVertices,
                          vector<FEdge*>& ioNewEdges,
                          vector<ViewEdge*>& ioNewViewEdges)
{ 
    ViewEdge *vEdge = fe->viewedge();
    
    // We first need to sort the view vertices from farther to closer to fe->vertexA
    
    SVertex *sv, *sv2;
    ViewVertex *vva, *vvb;
    vector<NonTVertex*>::const_iterator vv, vvend;
    for(vv=iViewVertices.begin(), vvend = iViewVertices.end();
        vv!=vvend;
        vv++)
    {
        // Add the viewvertices to the ViewShape
        AddVertex((*vv));

        // retrieve the correct SVertex from the view vertex
        //--------------------------------------------------
        sv = (*vv)->svertex();

        assert(sv->getId() == fe->vertexA()->getId());

        vva = vEdge->A();
        vvb = vEdge->B();

        // We split Fedge AB into AA' and A'B. A' and A'B are created.
        // AB becomes (address speaking) AA'. B is updated.
        //--------------------------------------------------
        SShape * shape = fe->shape();

        // a new edge, A'B is created.
        FEdge *newEdge = shape->SplitEdgeIn2(fe, sv);
        ioNewEdges.push_back(newEdge);

        ViewEdge *newVEdge = NULL;

        if((vva == 0) || (vvb == 0)) // that means we're dealing with a closed viewedge (loop)
        {
            // remove the chain that was starting by the fedge A of vEdge (which is different from fe !!!!)
            shape->RemoveEdgeFromChain(vEdge->fedgeA());
            // we set
            vEdge->SetA(*vv);
            vEdge->SetB(*vv);
            vEdge->SetFEdgeA(newEdge);
            //FEdge *previousEdge = newEdge->previousEdge();
            vEdge->SetFEdgeB(fe);
            newVEdge = vEdge;
            vEdge->fedgeA()->SetViewEdge(newVEdge);
        }
        else
        {

            // while we create the view edge, it updates the "ViewEdge" pointer
            // of every underlying FEdges to this.
            newVEdge = new ViewEdge((*vv),vvb);//, newEdge, vEdge->fedgeB());
            newVEdge->SetNature((fe)->getNature());
            newVEdge->SetFEdgeA(newEdge);
            //newVEdge->SetFEdgeB(fe);
            // If our original viewedge is made of one FEdge,
            // then
            if (fe == vEdge->fedgeB())
                //        if((vEdge->fedgeA() == vEdge->fedgeB()) || (fe == vEdge->fedgeB()))  //Aaron commented out this line
                newVEdge->SetFEdgeB(newEdge);
            else
                newVEdge->SetFEdgeB(vEdge->fedgeB()); //MODIF

            Id * newId = vEdge->splittingId();
            if(newId == 0){
                newId = new Id(vEdge->getId());
                vEdge->setSplittingId(newId);
            }
            newId->setSecond(newId->getSecond()+1);
            newVEdge->SetId(*newId);
            newVEdge->setSplittingId(newId);
            //        Id id(vEdge->getId().getFirst(), vEdge->getId().getSecond()+1);
            //        newVEdge->SetId(vEdge->getId());
            //        vEdge->SetId(id);

            AddEdge(newVEdge); // here this shape is set as the edge's shape

            // add new edge to the list of new edges passed as argument:
            ioNewViewEdges.push_back(newVEdge);



            if(0 != vvb)
                vvb->Replace((vEdge), newVEdge);

            // we split the view edge:
            vEdge->SetB((*vv));
            vEdge->SetFEdgeB(fe); //MODIF

            // Update fedges so that they point to the new viewedge:
            newVEdge->UpdateFEdges();

            // added by Aaron:
            //	newVEdge->SetA(*vv);
            //	newVEdge->SetB(vvb);
        }

        (*vv)->AddIncomingViewEdge(vEdge); // how do I know which is incoming and which is outgoing? goes from A to B?
        (*vv)->AddOutgoingViewEdge(newVEdge);
    }
} 




void ViewShape::SplitEdge(ViewVertex * newVertex, ViewEdge * vEdge, ViewEdge * & newVEdge,
                          bool isFront) // for TVertices only: is this edge in front?
// this is based on the above SplitEdge procedures, but it's much simpler since we're splitting at an existing vertex, and no new FEdges are being created.
{
    TVertex * tvert = dynamic_cast<TVertex*>(newVertex);
    NonTVertex * ntv = dynamic_cast<NonTVertex*>(newVertex);

    SVertex * sv;

    if (tvert != NULL)
        sv = (isFront ? tvert->frontSVertex() : tvert->backSVertex());
    else
        sv = ntv->svertex();

    ViewVertex * vva = vEdge->A();
    ViewVertex * vvb = vEdge->B();

    assert(vEdge->A() == NULL || hasEdge(vEdge->A(), vEdge));
    assert(vEdge->B() == NULL || hasEdge(vEdge->B(), vEdge));

    assert(vva == NULL || vva != vvb || hasEdgeTwice(vvb,vEdge));


    // check if this view edge is a closed loop
    if (vva == NULL || vvb == NULL)
    {
        // viewedge is a loop
        AddVertex(newVertex);

        vEdge->fedgeA()->shape()->RemoveEdgeFromChain(vEdge->fedgeA());  // not sure this is right
        vEdge->SetA(newVertex);
        vEdge->SetB(newVertex);

        assert(sv->fedges().size() == 2);
        FEdge * f1 = sv->fedges()[0];
        FEdge * f2 = sv->fedges()[1];

        newVEdge = vEdge;

        if (f1->vertexB() == sv)
        {
            assert(f2->vertexA() == sv);
            vEdge->SetFEdgeA( f2 );
            vEdge->SetFEdgeB ( f1 );

            f1->SetNextEdge(NULL);
            f2->SetPreviousEdge(NULL);
        }
        else
        {
            assert(f2->vertexB() == sv && f1->vertexA() == sv);
            vEdge->SetFEdgeA( f1);
            vEdge->SetFEdgeB( f2);

            f2->SetNextEdge(NULL);
            f1->SetPreviousEdge(NULL);
        }

        if (ntv != NULL)
        {
            ntv->AddIncomingViewEdge(newVEdge);
            ntv->AddOutgoingViewEdge(newVEdge);
        }
        else
        {
            if (isFront)
            {
                tvert->SetFrontEdgeA(newVEdge);
                tvert->SetFrontEdgeB(newVEdge);
            }
            else
            {
                tvert->SetBackEdgeA(newVEdge);
                tvert->SetBackEdgeB(newVEdge);
            }
        }

        assert(hasEdge(newVEdge->A(), newVEdge));
        assert(hasEdge(newVEdge->B(), newVEdge));

        return;
    }

    assert(hasEdge(vEdge->A(), vEdge));

    newVEdge = new ViewEdge( newVertex, vvb); // connect the new vertex to vvb
    newVEdge->SetNature(vEdge->getNature());
    newVEdge->SetFEdgeB( vEdge->fedgeB() );
    AddEdge(newVEdge);

    assert(hasEdge(vEdge->A(), vEdge));

    newVEdge->SetA(newVertex);
    newVEdge->SetB(vvb);
    // set up the fedges.  i'm assuming that things are oriented to go from A to B. not entirely sure about that though.
    // an assertion to test my assumption:
    //      assert(vEdge->fedgeA()->vertexA() == vva

    assert(hasEdge(vEdge->A(), vEdge));

    //assert(sv->fedges().size() == 2);
    FEdge * f1 = sv->fedges()[0];
    FEdge * f2 = sv->fedges()[1];

    //  printf("f1: %08X, f1->nextEdge: %08X, f1->previousEdge: %08X\n",f1,f1->nextEdge(), f1->previousEdge());
    //  printf("f2: %08X, f1->nextEdge: %08X, f2->previousEdge: %08X\n",f2,f2->nextEdge(), f2->previousEdge());
    //  printf("f1->nature: %d, f2->nature: %d\n", f1->getNature(), f2->getNature());
    //  printf("sv: %08X, f1->vertexA: %08X, f1->vertexB: %08X, f2->vertexA: %08X, f2->vertexB: %08X\n", sv,
    //	 f1->vertexA(), f1->vertexB(), f2->vertexA(), f2->vertexB());

    assert(hasEdge(vEdge->A(), vEdge));

    if (f1->vertexB() == sv)
    {
        assert(f2->vertexA() == sv);

        vEdge->SetFEdgeB ( f1 );
        newVEdge->SetFEdgeA( f2 );

        f1->SetNextEdge(NULL);
        f2->SetPreviousEdge(NULL);

        sv->shape()->AddChain(f2);

        // this assertion is invalid when it's a chain closed except at this vertex?
        //      assert(vEdge->fedgeA() != newVEdge->fedgeA());
    }
    else
    {
        assert(f2->vertexB() == sv && f1->vertexA() == sv);
        vEdge->SetFEdgeB( f2);
        newVEdge->SetFEdgeA( f1);

        f2->SetNextEdge(NULL);
        f1->SetPreviousEdge(NULL);

        sv->shape()->AddChain(f1);

        // this assertion is invalid when it's a chain closed except at this vertex?
        assert(vEdge->fedgeA() != newVEdge->fedgeA());
    }

    assert(hasEdge(vEdge->A(), vEdge));
    assert(vva != vvb || hasEdgeTwice(vvb,vEdge));

    vvb->Replace(vEdge, newVEdge);

    vEdge->SetB(newVertex);  // has to happen after Replace because of Replace's test for B

    assert(hasEdge(vEdge->A(), vEdge));
    assert(hasEdge(newVEdge->B(), newVEdge));

    Id * newId = vEdge->splittingId();
    if (newId == NULL)
    {
        newId = new Id(vEdge->getId());
        vEdge->setSplittingId(newId);
    }


    // Update fedges so that they point to the new viewedge
    newVEdge->UpdateFEdges();


    assert(hasEdge(vEdge->A(), vEdge));
    assert(vEdge->fedgeA()->viewedge() == vEdge);
    assert(newVEdge->fedgeA()->viewedge() == newVEdge);

    // point from the new vertex to the new view edges
    if (ntv != NULL)
    {
        ntv->AddIncomingViewEdge(vEdge);
        ntv->AddOutgoingViewEdge(newVEdge);
    }
    else
    {
        if (isFront)
        {
            tvert->SetFrontEdgeA(vEdge, true);
            tvert->SetFrontEdgeB(newVEdge, false);
        }
        else
        {
            tvert->SetBackEdgeA(vEdge, true);
            tvert->SetBackEdgeB(newVEdge, false);
        }
    }

    assert(hasEdge(newVEdge->A(), newVEdge));
    assert(hasEdge(newVEdge->B(), newVEdge));
    assert(hasEdge(vEdge->A(), vEdge));
    assert(hasEdge(vEdge->B(), vEdge));
}




// inline Vec3r ViewEdge::orientation2d(int iCombination) const
// {
//  return edge_orientation2d_function<ViewEdge>(*this, iCombination);
// }

// inline Vec3r  ViewEdge::orientation3d(int iCombination) const
// {
//  return edge_orientation3d_function<ViewEdge>(*this, iCombination);
// }

// inline real ViewEdge::z_discontinuity(int iCombination) const 
// {
//   return z_discontinuity_edge_function<ViewEdge>(*this, iCombination);
// }

// inline float ViewEdge::local_average_depth(int iCombination ) const
// {
//   return local_average_depth_edge_function<ViewEdge>(*this, iCombination);
// }

// inline float ViewEdge::local_depth_variance(int iCombination) const 
// {
//   return local_depth_variance_edge_function<ViewEdge>(*this, iCombination);
// }

// inline real ViewEdge::local_average_density(float sigma, int iCombination) const 
// {
//   return density_edge_function<ViewEdge>(*this, iCombination);
// }

const SShape * ViewEdge::occluded_shape() const 
{
    if(0 == _aShape)
        return 0;
    return _aShape->sshape();
}  

// inline Vec3r ViewEdge::curvature2d_as_vector(int iCombination) const 
// {
//   return curvature2d_as_vector_edge_function<ViewEdge>(*this, iCombination);
// }

// inline real ViewEdge::curvature2d_as_angle(int iCombination) const 
// {
//   return curvature2d_as_angle_edge_function<ViewEdge>(*this, iCombination);
// }


template<class T>
bool has_key(const vector<T> & vec,T key)
{
    assert(key != NULL);
    for(typename vector<T>::const_iterator it = vec.begin(); it != vec.end(); ++it)
        if ( *it == key)
            return true;
    return false;
}

bool hasVertex(ViewVertex * vv, SVertex *sv)
{
    NonTVertex * ntv = dynamic_cast<NonTVertex*>(vv);

    if (ntv != NULL)
        return ntv->svertex() == sv;

    TVertex * tv = dynamic_cast<TVertex*>(vv);
    assert(tv != NULL);
    return sv == tv->frontSVertex() || sv == tv->backSVertex();
}

bool hasEdge(ViewVertex *vv, ViewEdge *ve)
{
    assert( vv != NULL && ve != NULL );
    NonTVertex * ntv = dynamic_cast<NonTVertex*>(vv);
    if (ntv != NULL)
    {
        for(NonTVertex::edges_container::iterator it = ntv->viewedges().begin(); it != ntv->viewedges().end(); ++it)
            if ((*it).first == ve)
                return true;
        return false;
    }
    TVertex * tv = dynamic_cast<TVertex*>(vv);
    assert(tv != NULL);
    return tv->frontEdgeA().first == ve || tv->frontEdgeB().first == ve ||
            tv->backEdgeA().first == ve || tv->backEdgeB().first == ve;
}

bool hasEdgeTwice(ViewVertex *vv, ViewEdge *ve)
{
    assert( vv != NULL && ve != NULL );
    NonTVertex * ntv = dynamic_cast<NonTVertex*>(vv);
    if (ntv != NULL)
    {
        int n = 0;
        for(NonTVertex::edges_container::iterator it = ntv->viewedges().begin(); it != ntv->viewedges().end(); ++it)
            if ((*it).first == ve)
                n++;
        return n == 2;
    }
    TVertex * tv = dynamic_cast<TVertex*>(vv);
    assert(tv != NULL);
    int n = 0;
    if (tv->frontEdgeA().first == ve)
        n++;
    if (tv->frontEdgeB().first == ve)
        n++;
    if (tv->backEdgeA().first == ve)
        n++;
    if (tv->backEdgeB().first == ve)
        n++;
    return n == 2;
}

template<typename T>
void compareSets(const set<T*> set1, const set<T*> set2)
{
    for(typename set<T*>::const_iterator it1 = set1.begin(); it1 != set1.end();it1++)
        assert(set2.has_key(*it1)); // check for duplicates too
    for(typename set<T*>::const_iterator it2 = set2.begin(); it2 != set2.end();it2++)
        assert(set1.has_key(*it2)); // check for duplicates too
}

template<typename T>
void setCopyVector(set<T> & outputSet, const vector<T> & vec)
{
    for(typename vector<T>::const_iterator it = vec.begin(); it != vec.end(); ++it)
    {
        assert(outputSet.find(*it) == outputSet.end());
        outputSet.insert(*it);
    }
}

template<class T>
void compareVectors(const vector<T*> vec1, const vector<T*> vec2)
{
    set<T*> set1, set2;
    setCopyVector<T*>(set1,vec1);
    setCopyVector<T*>(set2,vec2);
    compareSets(set1,set2);
}

bool hasFEdge(ViewEdge* ve, FEdge *fe)
{
    FEdge * f= ve->fedgeA();
    while (f != NULL)
    {
        if (f == fe)
            return true;
        f = f->nextEdge();
    }
    return false;
}

void checkVertex(ViewVertex * vv)
{
    TVertex * tv = dynamic_cast<TVertex*>(vv);
    if (tv != NULL)
    {
        assert(tv->numEdges() >= 1);
        assert(tv->numEdges() <= 4);
        set<ViewVertex::directedViewEdge> ses;
        for(int i=0;i<tv->numEdges();i++)
        {
            ViewVertex::directedViewEdge * dve = tv->getEdge(i);

            assert(dve != NULL && dve->first != NULL);
            assert( dve->first->A() == tv || dve->first->B() == tv); // TODO: check incoming/outgoing
            assert( tv->frontEdgeA() == *dve || tv->frontEdgeB() == *dve ||
                    tv->backEdgeA() == *dve || tv->backEdgeB() == *dve);
            ses.insert(*dve);
        }

        vector<ViewVertex::directedViewEdge> edges;
        if (tv->frontEdgeA().first != NULL)
            edges.push_back(tv->frontEdgeA());
        if (tv->frontEdgeB().first != NULL)
            edges.push_back(tv->frontEdgeB());
        if (tv->backEdgeA().first != NULL)
            edges.push_back(tv->backEdgeA());
        if (tv->backEdgeB().first != NULL)
            edges.push_back(tv->backEdgeB());


        assert(edges.size() == tv->numEdges()); // this could fail erroneously if there are duplicated edges?

        for(vector<ViewVertex::directedViewEdge>::iterator it = edges.begin(); it != edges.end(); ++it)
            assert(ses.find(*it) != ses.end());

        // surprisingly, this loop can fail even when the above test succeeds
        // TODO: make this loop work for the intersecting pillows
        /*	  TVertex::edge_iterator it=tv->edges_begin();

      while (it != tv->edges_end())
      {
      assert( (*it).first->A() == tv || (*it).first->B() == tv); // TODO: check incoming/outgoing
      assert( tv->frontEdgeA() == (*it) || tv->frontEdgeB() == (*it) ||
      tv->backEdgeA() == (*it) || tv->backEdgeB() == (*it));
      ++it;
      }
      */


        /*	  for(TVertex::edge_iterator it=tv->edges_begin();;it != tv->edges_end();++it)
    {
    assert( (*it).first->A() == tv || (*it).first->B() == tv); // TODO: check incoming/outgoing
    assert( tv->frontEdgeA() == (*it) || tv->frontEdgeB() == (*it) ||
    tv->backEdgeA() == (*it) || tv->backEdgeB() == (*it));
    }*/
        assert(tv->frontSVertex()->viewvertex() == tv);
        assert(tv->backSVertex()->viewvertex() == tv);
        assert(tv->frontEdgeA().first == NULL ||
               tv->frontEdgeA().first->A() == tv || tv->frontEdgeA().first->B() == tv);
        assert(tv->backEdgeA().first == NULL ||
               tv->backEdgeA().first->A() == tv || tv->backEdgeA().first->B() == tv);
        assert(tv->frontEdgeB().first == NULL ||
               tv->frontEdgeB().first->A() == tv || tv->frontEdgeB().first->B() == tv);
        assert(tv->backEdgeB().first == NULL ||
               tv->backEdgeB().first->A() == tv || tv->backEdgeB().first->B() == tv);

        // TODO: make sure the list of edges matches the front/back edges



        return;
    }


    NonTVertex * ntv = dynamic_cast<NonTVertex*>(vv);
    assert(ntv != NULL);
    if (ntv->svertex()->viewvertex() != ntv)
        printf("ntv: %08X\n",ntv);
    assert(ntv->svertex()->viewvertex() == ntv);

    for(NonTVertex::edge_iterator eit = ntv->edges_begin(); eit != ntv->edges_end(); ++eit)
    {
        ViewEdge *ve = (*eit).first;
        assert(ve->A() == ntv || ve->B() == ntv);
    }
}

void ViewMap::checkPointers(const char * stepName,bool forceCheck)
// iterate over the whole map and check consistency of the datastructure, for debugging
//
// TODO: check 'incoming' vs. 'outgoing' on viewedges and viewvertices
// TODO: check consistency between ViewShape and corresponding SShape elements
// TODO: check consistency of "chains" listed in each SShape
// TODO: check that each svertex's sshape matches the viewedge's sshape, and vice versa
// see also TODOs below
{
#ifndef NDEBUG
    // only run this when debugging a certain test.  useful for quick-coding different checks without being too slow
    if (!forceCheck && !checkAllPointers)
        return;


    printf("Checking pointers (%s) *****\n",stepName);

    // --- create sets instead of lists.  the original data structures really should be sets, not vectors. ---
    // also, check for duplicates

    set<ViewEdge*> vedges;
    set<SVertex*> svertices;
    set<ViewShape*> viewshapes;
    set<FEdge*> fedges;
    set<ViewVertex*> viewvertices;

    setCopyVector<ViewEdge*>(vedges,_VEdges);
    setCopyVector<SVertex*>(svertices,_SVertices);
    setCopyVector<FEdge*>(fedges,_FEdges);
    setCopyVector<ViewShape*>(viewshapes,_VShapes);
    setCopyVector<ViewVertex*>(viewvertices,_VVertices);

    // ----- main tests ---------

    for(vector<ViewEdge*>::iterator veit = _VEdges.begin(); veit != _VEdges.end(); ++veit)
    {
        ViewEdge * ve = *veit;
        assert(ve != NULL);
        assert(ve->fedgeA()->viewedge() == ve);
        assert(ve->fedgeB()->viewedge() == ve);
        assert(ve->A() == NULL || ve->fedgeA()->previousEdge() == NULL);
        assert(ve->A() == NULL || ve->fedgeB()->nextEdge() == NULL);

        assert((ve->A() == NULL) == (ve->B() == NULL));

        assert(ve->A() == NULL || hasEdge(ve->A(), ve));
        assert(ve->B() == NULL || hasEdge(ve->B(), ve));

        assert(ve->A() == NULL || hasVertex(ve->A(), ve->fedgeA()->vertexA()));
        assert(ve->B() == NULL || hasVertex(ve->B(), ve->fedgeB()->vertexB()));

        assert(viewshapes.find(ve->viewShape()) != viewshapes.end());
        assert(ve->A() == NULL || viewvertices.find(ve->A()) != viewvertices.end());
        assert(ve->B() == NULL || viewvertices.find(ve->B()) != viewvertices.end());
        assert(fedges.find(ve->fedgeA()) != fedges.end());
        assert(fedges.find(ve->fedgeB()) != fedges.end());

        assert(ve->A() == NULL || ve->A() != ve->B() || hasEdgeTwice(ve->A(), ve));

    }

    for(vector<SVertex*>::iterator svit = _SVertices.begin(); svit != _SVertices.end(); ++svit)
    {
        SVertex * sv = *svit;
        assert (sv->viewvertex() == NULL || hasVertex(sv->viewvertex(), sv));
        vector<SVertex*> & svlist = sv->shape()->GetVertexList();
        assert(has_key<SVertex*>(svlist, sv));
        for(SVertex::fedges_container::iterator it = sv->fedges_begin(); it != sv->fedges_end(); ++it)
        {
            FEdge * fe = *it;
            assert(fe->vertexA() == sv || fe->vertexB() == sv);
        }

        /*
      // make sure we're not missing an fedges we should have
      if (sv->viewvertex() != NULL)
    {
      TVertex * tv = dynamic_cast<TVertex*>(sv->viewvertex());
      if (tv != NULL)
        {


        }
      NonTVertex * nvt = dynamic_cast<NonTVertex*>(sv->viewvertex());
      if (ntv != NULL)
        {

        }
    }
      */

        /*
      const vector<FEdge*> & fes = sv->fedges();

      for(vector<FEdge*>::const_iterator it = fes.begin(); it != fes.end(); it++)
    {

    }
      */
    }
    for(vector<FEdge*>::iterator feit = _FEdges.begin(); feit != _FEdges.end(); ++feit)
    {
        FEdge * fe = *feit;

        assert(hasFEdge(fe->viewedge(),fe));
        assert(has_key<FEdge*>(fe->vertexA()->fedges(),fe));
        assert(has_key<FEdge*>(fe->vertexB()->fedges(),fe));

        assert(fe->nextEdge() == NULL     || fe->nextEdge()->previousEdge() == fe);
        assert(fe->previousEdge() == NULL || fe->previousEdge()->nextEdge() == fe);
        assert(fe->nextEdge() == NULL     || fe->nextEdge()->viewedge() == fe->viewedge());

        // These lines doesn't work for loops:
        //      assert(fe->nextEdge() == NULL || fe->vertexB() == fe->nextEdge()->vertexA());
        //      assert(fe->previousEdge() == NULL || fe->vertexA() == fe->previousEdge()->vertexB());
        // TODO: check that shape fe->shape() has fe
    }


    for(vector<ViewVertex*>::iterator vvit = _VVertices.begin(); vvit != _VVertices.end(); ++vvit)
    {
        checkVertex(*vvit);
    }

    set<FEdge*> sfedges;

    for(vector<ViewShape*>::iterator vsit = _VShapes.begin(); vsit != _VShapes.end(); ++vsit)
    {
        ViewShape * vs = *vsit;
        SShape * ss = vs->sshape();
        assert(ss->viewShape() == vs);

        // check for duplicates among all sshapes
        setCopyVector<FEdge*>(sfedges,ss->GetEdgeList());

        //      compareVectors<SVertex*>(ss->GetVertexList,vs->GetVertexList());

        // more elements to check and make sure they all point back to this sshape
    }

    printf("Done checking pointers (%s).\n",stepName);

#endif
}


DebugPoint * ViewMap::addDebugPoint(DebugPoint::PointType ptType, Vec3r point3D, bool knownInvis, SVertex *sv)
{
    DebugPoint *dp = new DebugPoint();

    dp->pointType = ptType;
    dp->point3D = point3D;
    dp->knownInvis = knownInvis;
    dp->sv = sv;
    dp->hasPair = false;
    dp->RIFpoint = false;
    dp->debugString = NULL;

    _debugPoints.push_back(dp);

    return dp;
}


DebugPoint * ViewMap::addDebugPoint(DebugPoint::PointType ptType, Vec3r point3D, Vec3r pairPoint3D, bool knownInvis, SVertex *sv, bool pairKnownInvis, SVertex * pairSV)
{
    DebugPoint *dp = new DebugPoint();

    dp->pointType = ptType;
    dp->point3D = point3D;
    dp->knownInvis = knownInvis;
    dp->sv = sv;
    dp->hasPair = true;
    dp->pairPoint3D = pairPoint3D;
    dp->pairKnownInvis = pairKnownInvis;
    dp->pairSV = pairSV;
    dp->RIFpoint = false;
    dp->debugString = NULL;

    _debugPoints.push_back(dp);

    return dp;
}

void ViewMap::addRIFDebugPoint(DebugPoint::PointType ptType, Vec3r point3D, char * debugString, real radialCurvature)
{
    DebugPoint *dp = new DebugPoint();

    dp->pointType = ptType;
    dp->point3D = point3D;
    dp->knownInvis = false;
    dp->sv = NULL;
    dp->hasPair = false;
    dp->RIFpoint = true;
    dp->debugString = debugString;
    dp->radialCurvature = radialCurvature;

    _debugPoints.push_back(dp);
}



void ViewMap::render3D(bool selectionMode, DebugVisOptions options)
{
    Vec3r selectionColor(1,1,0);

    if (!selectionMode)
    {
        glShadeModel(GL_FLAT);
        glDisable(GL_LIGHTING);
        glDisable(GL_COLOR_MATERIAL);
    }

    // ---------------------------------- DRAW VIEW EDGES  ----------------------------------

    if (options.showViewEdges)
    {
        for(vector<ViewEdge*>::iterator vit = _VEdges.begin(); vit != _VEdges.end(); ++vit)
        {
            ViewEdge * ve = *vit;

            if (ve->qi() != 0 && options.visibleOnly)
                continue;

            // get/generate a name for this ViewEdge
            int name = getName(ve);

            // determine a color for this viewedge

            if (!selectionMode)
            {
                if (name == options.selectionName)
                {
                    real arcLength = 0;
                    FEdge * feA = ve->fedgeA();
                    FEdge * fe = feA;

                    do
                    {
                        arcLength += fe->getLength2D();
                        fe = fe->nextEdge();
                    } while (fe != NULL && fe != feA);

                    glColor3f(selectionColor[0],selectionColor[1],selectionColor[2]);

                    printf("Selected edge: %08X.  Nature: %d.  QI: %d.   Id: %d-%d.  AmbigVis: %s but was %s,  InconsVis: %s.  Votes: vis: %d, invis: %d, Image ArcLength = %f\n",
                           ve, ve->getNature(), ve->qi(),
                           ve->getId().getFirst(), ve->getId().getSecond(),
                           ve->ambiguousVisibility() ? "true" : "false",
                           ve->wasAmbiguous() ? "true" : "false",
                           ve->inconsistentVisibility() ? "true" : "false",
                           ve->visVotes, ve->invisVotes,
                           arcLength);
                    FEdgeSharp* feAs = dynamic_cast<FEdgeSharp*>(feA);
                    if(feAs && feAs->edge()->nearerFace)
                        printf("Nearer face: %08X\n",feAs->edge()->nearerFace);

                    printf("Endpoints: %08X %08X\n", ve->A(), ve->B());
                }
                else
                {
                    if (options.edgeColors == DebugVisOptions::EC_ID_COLOR)
                        glColor3f(ve->colorID()[0], ve->colorID()[1], ve->colorID()[2]);
                    else
                    {
                        switch(ve->getNature())
                        {
                        case Nature::SILHOUETTE:
                            glColor3f(1,0,0); break;
                            //			  glColor3f(.2,.2,.2); break;
                        case Nature::BORDER:
                            glColor3f(0,0,1); break;
                        case Nature::CREASE:
                        case Nature::VALLEY:
                        case Nature::RIDGE:
                        case Nature::SUGGESTIVE_CONTOUR:
                            glColor3f(.5,0,.5); break;
                        case Nature::SURFACE_INTERSECTION:
                            glColor3f(0,.5,0); break;
                        case Nature::PO_SURFACE_INTERSECTION:
                            glColor3f(1,0,0); break;
                        default:
                            printf("Nature: %X\n", (int)ve->getNature());
                            assert(0);
                            glColor3f(1,1,0);
                            break;
                        }
                    }
                }
            }

            // draw the edge
            if (selectionMode)
                glPushName(name);

            glLineWidth(8);
            //	  glLineWidth( ve->inconsistentVisibility() || ve->ambiguousVisibility() ? 10 : 4);


            //	  if (name == options.selectionName)
            //	    printf("FEdge sources:");
            glBegin(GL_LINE_STRIP);
            FEdge * fe = ve->fedgeA();
            FEdge * festart = fe;
            Vec3r pt = fe->vertexA()->getPoint3D();
            glVertex3f(pt[0], pt[1], pt[2]);
            do
            {
                pt = fe->vertexB()->getPoint3D();
                glVertex3f(pt[0], pt[1], pt[2]);
                fe = fe->nextEdge();
                //	      if (name == options.selectionName)
                //		{
                //		  if (dynamic_cast<FEdgeSharp*>(fe) != NULL)
                //		    printf(" %08X", ((FEdgeSharp*)fe)->edge());
                //		  else
                //		    printf(" 0");
                //}
            }
            while (fe != NULL && fe != festart);
            //	  if (name == options.selectionName)
            //	    printf("\n");
            glEnd();

            /*
      // draw T-Vertex connections: draw a 3D line to visualize image-space intersections
      // (note: this might also be drawing intersections on the surface)
      for(int i=0;i<2;i++)
        {
          ViewVertex * vv = (i == 0 ? ve->A() : ve->B());
          TVertex * tv = dynamic_cast<TVertex*>(vv);

          if (tv == NULL)
        continue;

          // only draw this once
          if (tv->frontEdgeA().first != ve && tv->frontEdgeB().first != ve)
        continue;

          bool frontVis = ((tv->frontEdgeA().first != NULL && tv->frontEdgeA().first->qi() == 0) ||
                   (tv->frontEdgeB().first != NULL && tv->frontEdgeB().first->qi() == 0));
          bool backVis = ((tv->backEdgeA().first != NULL && tv->backEdgeA().first->qi() == 0) ||
                  (tv->backEdgeB().first != NULL && tv->backEdgeB().first->qi() == 0));

          // check if at least one part of front is visible and one part of back is visible
          if (frontVis && backVis)
        {
          Vec3f front = tv->frontSVertex()->point3D();
          Vec3f back = tv->backSVertex()->point3D();

          if (name == options.selectionName)
            glColor3f(1,1,0);
          else
            glColor3f(0,1,1);
          glLineWidth(1);
          glBegin(GL_LINES);
          glVertex3f(front.x(), front.y(), front.z());
          glVertex3f(back.x(), back.y(), back.z());
          glEnd();
        }
        }
      */

            if (selectionMode)
                glPopName();
        }
    }

    // ---------------------------------- DRAW DEBUGGING POINTS  ----------------------------------

    if (options.showPoints || options.showRIFPoints)
    {
        glPointSize(10);
        glLineWidth(2);

        for(vector<DebugPoint*>::iterator dpit = _debugPoints.begin(); dpit != _debugPoints.end(); ++dpit)
        {
            // check if this point is visible

            // note: these visibility computations may be approximate.
            if (options.visibleOnly && (*dpit)->knownInvis)
                continue;

            if (options.showSpecificPoints && options.pointsToShow != (*dpit)->pointType)
                continue;

            if ( (*dpit)->RIFpoint && ! options.showRIFPoints)
                continue;

            if ( !(*dpit)->RIFpoint && !options.showPoints)
                continue;


            if (options.limitRegion && (*dpit)->pointType != DebugPoint::ERROR)
                continue;

            /*
      if (options.visibleOnly && (*dpit)->sv != NULL)
        {
          bool vis = false;
          SVertex * sv = (*dpit)->sv;
          for(vector<FEdge*>::const_iterator it = sv->fedges().begin(); it != sv->fedges().end(); ++it)
        if ((*it) != NULL)
          if ((*it)->viewedge()->qi() == 0)
            {
              vis = true;
              break;
            }
          if (!vis)
        continue;
        }*/

            // get the "name" for this point
            int name = getName( *dpit);

            if (!selectionMode)
            {
                if (name == options.selectionName && (*dpit)->debugString != NULL)
                    printf("Selected point: %s\n",  (*dpit)->debugString);
                //		       (*dpit)->ndotv, (*dpit)->rootFindingFailed ? "true":"false",
                //		       (*dpit)->degenerate ? "true" : "false");

                if (name == options.selectionName)
                    glColor3f(selectionColor[0],selectionColor[1],selectionColor[2]);
                else
                {
                    if ( (*dpit)->RIFpoint && options.radialCurvaturePoints)
                    {
                        if ( (*dpit)->radialCurvature > 0)
                            glColor3f(1,0,0);
                        else
                            if ( (*dpit)->radialCurvature < 0)
                                glColor3f(0,0,1);
                            else
                                glColor3f(0,0,0);
                    }
                    else
                    {
                        switch( (*dpit)->pointType)
                        {
                        case DebugPoint::ERROR: glColor3f(1,0,0); break;
                        case DebugPoint::CUSP:  glColor3f(0,0.5,0); break;
                        case DebugPoint::RAY_TRACE_VISIBLE: glColor3f(0,0,1); break;
                        case DebugPoint::RAY_TRACE_INVISIBLE: glColor3f(1,1,0); break;
                        case DebugPoint::INTERSECTION_2D:     glColor3f(0,1,0); break;
                        case DebugPoint::INTERSECTION_2D_ON_SURFACE: glColor3f(0,.5,0); break;
                        case DebugPoint::INVISIBLE_BACK_FACE: glColor3f(0,1,1); break;
                        case DebugPoint::INVISIBLE_ONE_RING_OVERLAP: glColor3f(0,0,0); break;
                        case DebugPoint::PO_CROSSING_INSIDE_TRI:  glColor3f(0,1,1); break;
                        case DebugPoint::PO_CROSSING_ON_MESH_EDGE: glColor3f(1,.5,0); break;
                        case DebugPoint::SI_SIL_CONNECTION:  glColor3f(.5,.5,.5); break;
                        case DebugPoint::FRONT_FACING_VERTEX: glColor3f(1,.5,0); break;
                        case DebugPoint::BACK_FACING_VERTEX: glColor3f(0,1,1); break;
                        case DebugPoint::CONTOUR_VERTEX: glColor3f(.5,.5,.5); break;
                        case DebugPoint::ISOPHOTE: glColor3f(0,1,1); break;
                        default: assert(0);
                        }
                    }
                }
            }

            if (selectionMode)
                glPushName(name);

            glBegin(GL_POINTS);
            glVertex3f( (*dpit)->point3D[0],(*dpit)->point3D[1],(*dpit)->point3D[2]);

            if ( (*dpit)->hasPair )
            {
                glVertex3f( (*dpit)->pairPoint3D[0],(*dpit)->pairPoint3D[1],(*dpit)->pairPoint3D[2]);
                glEnd();

                if (options.showPairs && (*dpit)->pointType != DebugPoint::INTERSECTION_2D)
                {
                    glBegin(GL_LINES);
                    glVertex3f( (*dpit)->point3D[0],(*dpit)->point3D[1],(*dpit)->point3D[2]);
                    glVertex3f( (*dpit)->pairPoint3D[0],(*dpit)->pairPoint3D[1],(*dpit)->pairPoint3D[2]);
                    glEnd();
                }
            }
            else
                glEnd();

            if (selectionMode)
                glPopName();
        }
    }

    // ---------------------------------- DRAW VIEW VERTICES  ----------------------------------

    if (options.showViewVertices)
    {
        glPointSize(10);

        for(vector<ViewVertex*>::iterator it = _VVertices.begin(); it != _VVertices.end(); ++it)
        {
            TVertex * tv = dynamic_cast<TVertex*>(*it);
            NonTVertex * ntv = dynamic_cast<NonTVertex*>(*it);

            int name = getName(*it);

            if (selectionMode)
                glPushName(name);

            if (!selectionMode)
            {
                if (name == options.selectionName)
                {
                    glColor3f(selectionColor[0],selectionColor[1],selectionColor[2]);
                    printf("Selected view vertex: %08X.  Nature: %d\n",  *it, (*it)->getNature());

                    if (ntv != NULL)
                    {
                        printf("Edges: ");
                        for(NonTVertex::edges_container::iterator eit = ntv->viewedges().begin(); eit != ntv->viewedges().end(); ++eit)
                            printf("%08X%c ", (*eit).first, (*eit).first->qi() == 0 ? 'v' : 'i');
                        printf("\n");
                        if ( ntv->svertex() != NULL)
                            printf("SVertex Source Edge: %08X\n", ntv->svertex()->GetSourceEdge() );

                    }
                    else
                        if (tv != NULL)
                        {
                            printf("Front: %08X%c %08X%c, Back: %08X%c %08X%c\n",
                                   tv->frontEdgeA().first, tv->frontEdgeA().first->qi() == 0 ? 'v' : 'i',
                                   tv->frontEdgeB().first, tv->frontEdgeB().first->qi() == 0 ? 'v' : 'i',
                                   tv->backEdgeA().first, tv->backEdgeA().first->qi() == 0 ? 'v' : 'i',
                                   tv->backEdgeB().first, tv->backEdgeB().first->qi() == 0 ? 'v' : 'i' );
                        }
                }
                else
                    if (tv != NULL && tv->sameFace())
                        glColor3ub(0,128,0);
                    else
                        if (tv != NULL && !tv->sameFace())
                            glColor3ub(0,255,0);
                        else
                            if (ntv != NULL && (ntv->getNature() & Nature::CUSP))
                                glColor3ub(255,128,0);
                            else
                                glColor3ub(0,0,255);
            }

            if (tv != NULL)
            {
                bool frontVis = !options.visibleOnly || ((tv->frontEdgeA().first != NULL && tv->frontEdgeA().first->qi() == 0) ||
                                                         (tv->frontEdgeB().first != NULL && tv->frontEdgeB().first->qi() == 0));
                bool backVis = !options.visibleOnly || ((tv->backEdgeA().first != NULL && tv->backEdgeA().first->qi() == 0) ||
                                                        (tv->backEdgeB().first != NULL && tv->backEdgeB().first->qi() == 0));

                glBegin(GL_POINTS);
                if (frontVis)
                    glVertex3f(tv->frontSVertex()->getX(), tv->frontSVertex()->getY(), tv->frontSVertex()->getZ());

                if (backVis)
                    glVertex3f(tv->backSVertex()->getX(), tv->backSVertex()->getY(), tv->backSVertex()->getZ());
                glEnd();

                if (frontVis && backVis)
                {
                    Vec3f front = tv->frontSVertex()->point3D();
                    Vec3f back = tv->backSVertex()->point3D();

                    if (name == options.selectionName)
                        glColor3f(1,1,0);
                    else
                        glColor3f(0,1,1);
                    glLineWidth(1);
                    glBegin(GL_LINES);
                    glVertex3f(front.x(), front.y(), front.z());
                    glVertex3f(back.x(), back.y(), back.z());
                    glEnd();
                }
            }
            else
            {
                bool show = !options.visibleOnly;

                if (!show)
                    for(NonTVertex::edges_container::iterator eit = ntv->viewedges().begin(); eit != ntv->viewedges().end(); ++eit)
                        if ( (*eit).first->qi() == 0)
                            show = true;

                if (show)
                {
                    glBegin(GL_POINTS);
                    glVertex3f(ntv->getX(), ntv->getY(), ntv->getZ());
                    glEnd();
                }
            }


            if (selectionMode)
                glPopName();
        }
    }

    // ---------------------------------- DRAW PUNCHOUTS  ----------------------------------

    if (options.showPunchouts)
    {
        glLineWidth(5);

        if (!selectionMode)
        {
            glPolygonOffset(-1,1);
            glEnable(GL_POLYGON_OFFSET_FILL);
        }

        // draw the boundaries
        for(map<WFace*,FacePOData*>::iterator poit=_facePOData.begin(); poit!=_facePOData.end();++poit)
        {
            WFace * face = (*poit).first;
            FacePOData * data = (*poit).second;

            for(vector<POBoundaryEdge*>::iterator pobit=data->POboundary.begin();
                pobit != data->POboundary.end(); ++pobit)
            {
                POBoundaryEdge * bnd = *pobit;

                if (!bnd->isRealBoundary)
                    continue;

                if (options.limitRegion && options.regionIndex != bnd->POregionIndex)
                    continue;

                int name = getName(bnd);
                setRegion(name,bnd->POregionIndex);

                if (!selectionMode)
                {
                    if (name == options.selectionName)
                        glColor3f(selectionColor[0],selectionColor[1],selectionColor[2]);
                    else
                        glColor3f(0,0,0);
                }
                else
                    glPushName(name);

                glBegin(GL_LINES);
                glVertex3f(bnd->A[0],bnd->A[1],bnd->A[2]);
                glVertex3f(bnd->B[0],bnd->B[1],bnd->B[2]);
                glEnd();

                if (selectionMode)
                    glPopName();
            }
        }

        // draw the inconsistent triangles
        multimap<int,InconsistentTri*>::iterator begin, end;
        if (options.limitRegion)
        {
            begin = _inconsistentTris.lower_bound(options.regionIndex);
            end = _inconsistentTris.upper_bound(options.regionIndex);
        }
        else
        {
            begin = _inconsistentTris.begin();
            end = _inconsistentTris.end();
        }

        for(multimap<int,InconsistentTri*>::iterator it = begin; it != end; ++it)
        {
            InconsistentTri * tri = (*it).second;

            //	  if (options.limitRegion && options.regionIndex != tri->sourceFaceregionIndex)
            //	    continue;

            int name = getName(tri);
            setRegion(name,(*it).first);

            //	  printf("region: %d\n", (*it).first);

            if (selectionMode)
                glPushName(name);
            else
            {
                if (name == options.selectionName)
                    glColor3f(selectionColor[0],selectionColor[1],selectionColor[2]);
                else
                    glColor3f(1,0,0);
            }

            glBegin(GL_TRIANGLES);
            glVertex3f(tri->P[0][0], tri->P[0][1], tri->P[0][2]);
            glVertex3f(tri->P[1][0], tri->P[1][1], tri->P[1][2]);
            glVertex3f(tri->P[2][0], tri->P[2][1], tri->P[2][2]);
            glEnd();

            if (selectionMode)
                glPopName();

        }

        if (!selectionMode)
            glDisable(GL_POLYGON_OFFSET_FILL);
    }

    if (options.showPOCuspTesselation)
    {
        for(vector<pair<WFace*,Vec3r> >::iterator it=_poCuspFaces.begin(); it != _poCuspFaces.end(); ++it)
        {
            WFace * face = (*it).first;
            Vec3r color = (*it).second;

            int regionIndex = ((WXFace*)face)->sourcePOB()->POregionIndex;
            if (options.limitRegion && options.regionIndex != regionIndex)
                continue;

            int name = getName(face);
            setRegion(name,regionIndex);

            if (selectionMode)
                glPushName(name);
            else
            {
                if (name == options.selectionName)
                    glColor3f(selectionColor[0],selectionColor[1],selectionColor[2]);
                else
                    glColor3f(color[0],color[1],color[2]);
            }

            glBegin(GL_TRIANGLES);
            Vec3r v1 = face->GetVertex(0)->GetVertex();
            Vec3r v2 = face->GetVertex(1)->GetVertex();
            Vec3r v3 = face->GetVertex(2)->GetVertex();
            glVertex3f(v1[0],v1[1],v1[2]);
            glVertex3f(v2[0],v2[1],v2[2]);
            glVertex3f(v3[0],v3[1],v3[2]);
            glEnd();

            if (selectionMode)
                glPopName();
        }
    }


    if (!selectionMode)
    {
        glEnable(GL_LIGHTING);
        glEnable(GL_COLOR_MATERIAL);
        glShadeModel(GL_SMOOTH);
    }

}

