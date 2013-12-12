
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

#include "Silhouette.h"
#include "ViewMap.h"
#include "../winged_edge/WEdge.h"
#include "SilhouetteGeomEngine.h"

                  /**********************************/
                  /*                                */
                  /*                                */
                  /*             SVertex            */
                  /*                                */
                  /*                                */
                  /**********************************/

Nature::VertexNature SVertex::getNature() const {
  Nature::VertexNature nature = Nature::S_VERTEX;
  if (_pViewVertex)
    nature |= _pViewVertex->getNature();
  return nature;
}

SVertex * SVertex::castToSVertex(){
  return this;
}

ViewVertex * SVertex::castToViewVertex(){
  return _pViewVertex;
}

NonTVertex * SVertex::castToNonTVertex(){
  return dynamic_cast<NonTVertex*>(_pViewVertex);
}

TVertex * SVertex::castToTVertex(){
  return dynamic_cast<TVertex*>(_pViewVertex);
}

float SVertex::shape_importance() const 
{
  return shape()->importance();
}
 
//Material SVertex::material() const {return _Shape->material();}
Id SVertex::shape_id() const {return _Shape->getId();}
const SShape *  SVertex::shape() const {return _Shape;}

const int SVertex::qi() const 
{
  if (getNature() & Nature::T_VERTEX)
    Exception::raiseException();
  return (_FEdges[0])->qi();
}

occluder_container::const_iterator SVertex::occluders_begin() const 
{
  if (getNature() & Nature::T_VERTEX)
    Exception::raiseException();
  return (_FEdges[0])->occluders_begin();
}

occluder_container::const_iterator SVertex::occluders_end() const 
{
  if (getNature() & Nature::T_VERTEX)
    Exception::raiseException();
  return (_FEdges[0])->occluders_end();
}

bool SVertex::occluders_empty() const 
{
  if (getNature() & Nature::T_VERTEX)
    Exception::raiseException();
  return (_FEdges[0])->occluders_empty();
}

int SVertex::occluders_size() const 
{
  if (getNature() & Nature::T_VERTEX)
    Exception::raiseException();
  return (_FEdges[0])->occluders_size();
}

const Polygon3r& SVertex::occludee() const 
{
  if (getNature() & Nature::T_VERTEX)
    Exception::raiseException();
  return (_FEdges[0])->occludee();
}

const SShape* SVertex::occluded_shape() const 
{
  if (getNature() & Nature::T_VERTEX)
    Exception::raiseException();
  return (_FEdges[0])->occluded_shape();
}

const bool  SVertex::occludee_empty() const
{
  if (getNature() & Nature::T_VERTEX)
    Exception::raiseException();
  return (_FEdges[0])->occludee_empty();
}

real SVertex::z_discontinuity() const 
{
  if (getNature() & Nature::T_VERTEX)
    Exception::raiseException();
  return (_FEdges[0])->z_discontinuity();
}

FEdge* SVertex::fedge()
{
  if (getNature() & Nature::T_VERTEX)
    return 0;
  return _FEdges[0];
}

FEdge* SVertex::getFEdge(Interface0D& inter)
{
  FEdge * result = 0;
  SVertex* iVertexB = dynamic_cast<SVertex*>(&inter);
  if (!iVertexB)
    return result;
  vector<FEdge*>::const_iterator fe=_FEdges.begin(), feend=_FEdges.end();
  for(;
  fe!=feend;
  ++fe)
  {
    if(  (((*fe)->vertexA() == this) && ((*fe)->vertexB() == iVertexB))
      || (((*fe)->vertexB() == this) && ((*fe)->vertexA() == iVertexB)))
      result = (*fe);
  }
  if((result == 0) && (getNature() & Nature::T_VERTEX))
  {
    SVertex *brother;
    ViewVertex *vvertex = viewvertex();
    TVertex * tvertex = dynamic_cast<TVertex*>(vvertex);
    if(tvertex)
    {
      brother = tvertex->frontSVertex();
      if(this == brother)
        brother = tvertex->backSVertex();
      const vector<FEdge*>& fedges = brother->fedges();
      for(fe=fedges.begin(),feend=fedges.end();
      fe!=feend;
      ++fe)
      {
        if(  (((*fe)->vertexA() == brother) && ((*fe)->vertexB() == iVertexB))
          || (((*fe)->vertexB() == brother) && ((*fe)->vertexA() == iVertexB)))
        result = (*fe);
      }
    }
  }
  if((result == 0) && (iVertexB->getNature() & Nature::T_VERTEX))
  {
    SVertex *brother;
    ViewVertex *vvertex = iVertexB->viewvertex();
    TVertex * tvertex = dynamic_cast<TVertex*>(vvertex);
    if(tvertex)
    {
      brother = tvertex->frontSVertex();
      if(iVertexB == brother)
        brother = tvertex->backSVertex();
      for(fe=_FEdges.begin(),feend=_FEdges.end();
      fe!=feend;
      ++fe)
      {
        if(  (((*fe)->vertexA() == this) && ((*fe)->vertexB() == brother))
          || (((*fe)->vertexB() == this) && ((*fe)->vertexA() == brother)))
        result = (*fe);
      }
    }
  }
  
  return result;
}


                  /**********************************/
                  /*                                */
                  /*                                */
                  /*             FEdge              */
                  /*                                */
                  /*                                */
                  /**********************************/


int FEdge::viewedge_nature() const {return _ViewEdge->getNature();}
//float FEdge::viewedge_length() const {return _ViewEdge->viewedge_length();}
const SShape* FEdge::occluded_shape() const 
{
  ViewShape * aShape = _ViewEdge->aShape(); 
  if(aShape == 0) 
    return 0;
  return aShape->sshape();
}

float FEdge::shape_importance() const 
{
  return _VertexA->shape()->importance();
} 

int FEdge::invisibility() const 
{
  return _ViewEdge->qi();
}

occluder_container::const_iterator FEdge::occluders_begin() const {return _ViewEdge->occluders_begin();}
occluder_container::const_iterator FEdge::occluders_end() const {return _ViewEdge->occluders_end();}
bool FEdge::occluders_empty() const {return _ViewEdge->occluders_empty();}
int FEdge::occluders_size() const {return _ViewEdge->occluders_size();}
const bool  FEdge::occludee_empty() const
{
  return _ViewEdge->occludee_empty();
}



Id FEdge::shape_id() const 
{
  return _VertexA->shape()->getId();
}
const SShape* FEdge::shape() const 
{
  return _VertexA->shape();
}

real FEdge::z_discontinuity() const 
{
  if(!(getNature() & Nature::SILHOUETTE) && !(getNature() & Nature::BORDER))
  {
    return 0;
  }

  BBox<Vec3r> box = ViewMap::getInstance()->getScene3dBBox();

  Vec3r bbox_size_vec(box.getMax() - box.getMin());
  real bboxsize = bbox_size_vec.norm();
  if(occludee_empty())

  {
    //return FLT_MAX;

    return 1.0;

    //return bboxsize;

  }
  //  real result;
  //  z_discontinuity_functor<SVertex> _functor;

  //  Evaluate<SVertex,z_discontinuity_functor<SVertex> >(&_functor, iCombination, result )
  Vec3r middle((_VertexB->point3d()-_VertexA->point3d()));

  middle /= 2;
  Vec3r disc_vec(middle - _occludeeIntersection);
  real res = disc_vec.norm() / bboxsize;
  return res;
  
  //return fabs((middle.z()-_occludeeIntersection.z()));
}


//float FEdge::local_average_depth(int iCombination ) const 
//{
//
//  float result;
//  local_average_depth_functor<SVertex> functor;
//  Evaluate(&functor, iCombination, result);
//
//  return result;
//}
//float FEdge::local_depth_variance(int iCombination ) const
//{
//  float result;
//
//  local_depth_variance_functor<SVertex> functor;
//
//  Evaluate(&functor, iCombination, result);
//
//  return result;
//}
//
//
//real FEdge::local_average_density( float sigma, int iCombination) const 
//{
//  float result;
//
//  density_functor<SVertex> functor(sigma);
//
//  Evaluate(&functor, iCombination, result);
//
//  return result;
//}
//
////Vec3r FEdge::normal(int& oException /* = Exception::NO_EXCEPTION */)
////{
////  Vec3r Na = _VertexA->normal(oException);
////  if(oException != Exception::NO_EXCEPTION)
////    return Na;
////  Vec3r Nb = _VertexB->normal(oException);
////  if(oException != Exception::NO_EXCEPTION)
////    return Nb;
////  return (Na+Nb)/2.0;
////}  
//
//Vec3r FEdge::curvature2d_as_vector(int iCombination) const 
//{
//  Vec3r result;
//  curvature2d_as_vector_functor<SVertex> _functor;
//  Evaluate<Vec3r,curvature2d_as_vector_functor<SVertex> >(&_functor, iCombination, result );
//  return result;
//}
//
//real FEdge::curvature2d_as_angle(int iCombination) const 
//{
//  real result;
//  curvature2d_as_angle_functor<SVertex> _functor;
//  Evaluate<real,curvature2d_as_angle_functor<SVertex> >(&_functor, iCombination, result );
//  return result;
//}

                  /**********************************/
                  /*                                */
                  /*                                */
                  /*             FEdgeSharp        */
                  /*                                */
                  /*                                */
                  /**********************************/

//Material FEdge::material() const 
//{
//  return _VertexA->shape()->material();
//}
const Material& FEdgeSharp::aMaterial() const {
  return _VertexA->shape()->material(_aMaterialIndex);
}

const Material& FEdgeSharp::bMaterial() const {
  return _VertexA->shape()->material(_bMaterialIndex);
}

                 /**********************************/
                  /*                                */
                  /*                                */
                  /*             FEdgeSmooth         */
                  /*                                */
                  /*                                */
                  /**********************************/

const Material& FEdgeSmooth::material() const {
  return _VertexA->shape()->material(_MaterialIndex);
}


WFace * FEdgeSharp::getFace1() { return _edge->GetaFace(); }
WFace * FEdgeSharp::getFace2() { return _edge->GetbFace(); }





// Shewchuk's orientation code
extern "C"
{
  void exactinit(); // this must be called before any calls to orient
  //  real orient2d(real * real *, real*);
  real orient3d(real *,real*,real*,real*);
};


// determine which side of the triangle PQR point A lies on.
// positive for one side, negative for the other
// magnitude is approximately six times the volume of the tetrahedron (PQRA)
inline
real orient3d(Vec3r P, Vec3r Q, Vec3r R, Vec3r A)
{
  real p[3] = { P[0], P[1], P[2] };
  real q[3] = { Q[0], Q[1], Q[2] };
  real r[3] = { R[0], R[1], R[2] };
  real a[3] = { A[0], A[1], A[2] };

  return orient3d(p,q,r,a);
}

inline 
bool sameSide(Vec3r P, Vec3r Q, Vec3r R, Vec3r A1, Vec3r A2)
{
  bool a1side = orient3d(P,Q,R,A1) > 0;
  bool a2side = orient3d(P,Q,R,A2) > 0;

  return a1side == a2side;
}



// check if the 2D projection of these two edges intersect
//   if so, return the 1D parameter of the intersection point on each edge, w.r.t. 3D coordinates.

bool FEdge::intersectParametric(FEdge & fe2, Vec3r viewpoint, real t3D, real u3D)
{
  Vec3r A1 = vertexA()->getPoint3D();
  Vec3r B1 = vertexB()->getPoint3D();
  Vec3r A2 = fe2.vertexA()->getPoint3D();
  Vec3r B2 = fe2.vertexB()->getPoint3D();

  if (sameSide(A1,B1,viewpoint, A2, B2) || sameSide(A2, B2, viewpoint, A1, B1))
    return false;

  // now, there *must* be an intersection.

  // for each edge, the normal of the plane containing the edge and the viewpoint
  Vec3r N1 = (A1-viewpoint) ^ (B1-viewpoint);
  Vec3r N2 = (A2-viewpoint) ^ (B2-viewpoint);

  // direction vector of the intersection of the two planes.
  Vec3r V = N1 ^ N2;

  // check if the planes coincide (i.e., source edges are colinear)
  assert(V.norm() > 0);
  
  // ----- compute t parameter ------

  // form a plane for line 1, normal to the plane containing the viewpoint

  Vec3r BA1 = B1 - A1;
  Vec3r hsNormal1 = N1 ^ BA1;
  
  // intersect ray in direction of V through the plane
  real w1;
  GeomUtils::intersection_test res1 = GeomUtils::intersectLinePlanePN(viewpoint, V, hsNormal1, A1, w1);

  if (res1 != GeomUtils::DO_INTERSECT)
    {
      printf("res1 = %d\n", res1);
      printf("viewpoint = [%f %f %f]\n", viewpoint[0], viewpoint[1], viewpoint[2]);
      printf("A1 = [%f %f %f]\n", A1[0], A1[1], A1[2]);
      printf("B1 = [%f %f %f]\n", B1[0], B1[1], B1[2]);
      printf("A2 = [%f %f %f]\n", A2[0], A2[1], A2[2]);
      printf("B2 = [%f %f %f]\n", B2[0], B2[1], B2[2]);
      printf("N1 = [%f %f %f]\n", N1[0], N1[1], N1[2]);
      printf("N2 = [%f %f %f]\n", N2[0], N2[1], N2[2]);
      printf("V = [%f %f %f]\n", V[0], V[1], V[2]);
      printf("hsNormal1 = [%f %f %f]\n", hsNormal1[0], hsNormal1[1], hsNormal1[2]);
    }

  assert(res1 == GeomUtils::DO_INTERSECT);

  Vec3r pt1 = viewpoint + w1 * V;

  t3D = ((pt1 - A1) * BA1) / (BA1*BA1);

  assert(t3D >=0 && t3D <= 1);

  // if (t3D < 0 || t3D > 1)
    //    return false;


  // ----- compute u parameter ------

  // form a half-space plane for line 2

  Vec3r BA2 = B2 - A2;
  Vec3r hsNormal2 = N2 ^ BA2;
  
  real w2;
  GeomUtils::intersection_test res2 = GeomUtils::intersectLinePlanePN(viewpoint, V, hsNormal2, A2, w2);

  if (res2 != GeomUtils::DO_INTERSECT)
    {
      printf("res1 = %d\n", res1);
      printf("viewpoint = [%f %f %f]\n", viewpoint[0], viewpoint[1], viewpoint[2]);
      printf("A1 = [%f %f %f]\n", A1[0], A1[1], A1[2]);
      printf("B1 = [%f %f %f]\n", B1[0], B1[1], B1[2]);
      printf("A2 = [%f %f %f]\n", A2[0], A2[1], A2[2]);
      printf("B2 = [%f %f %f]\n", B2[0], B2[1], B2[2]);
      printf("N1 = [%f %f %f]\n", N1[0], N1[1], N1[2]);
      printf("N2 = [%f %f %f]\n", N2[0], N2[1], N2[2]);
      printf("V = [%f %f %f]\n", V[0], V[1], V[2]);
      printf("hsNormal2 = [%f %f %f]\n", hsNormal2[0], hsNormal2[1], hsNormal2[2]);
    }

  assert(res2 == GeomUtils::DO_INTERSECT);

  Vec3r pt2 = viewpoint + w2 * V;

  u3D = ((pt2 - A2) * BA2) / (BA2*BA2);

  assert( u3D >=0 && u3D <=1);

  //  if (u3D < 0 || u3D > 1)
  //    return false;


  
  return true;
}


  /* splits an edge into several edges. 
   *  The edge's vertices are passed rather than 
   *  the edge itself. This way, all feature edges (SILHOUETTE,
   *  CREASE, BORDER) are splitted in the same time.
   *  The processed edges are flagged as done (using the userdata
   *  flag).One single new vertex is created whereas 
   *  several splitted edges might created for the different 
   *  kinds of edges. These new elements are added to the lists
   *  maintained by the shape.
   *  new chains are also created.
   *    ioA
   *      The first vertex for the edge that gets splitted
   *    ioB
   *      The second vertex for the edge that gets splitted
   *    iParameters
   *      A vector containing 2D real vectors indicating the parameters
   *      giving the intersections coordinates in 3D and in 2D.
   *      These intersections points must be sorted from B to A.
   *      Each parameter defines the intersection point I as I=A+T*AB.
   *      T<0 and T>1 are then incorrect insofar as they give intersections
   *      points that lie outside the segment.
   *    ioNewEdges
   *      The edges that are newly created (the initial edges are not 
   *      included) are added to this list.
   */
void SShape::SplitEdge(FEdge *fe, const vector<Vec2r>& iParameters, vector<FEdge*>& ioNewEdges)
    {
    
    SVertex *ioA = fe->vertexA();
    SVertex *ioB = fe->vertexB();
    Vec3r A = ioA->point3D();
    Vec3r B = ioB->point3D();
    Vec3r a = ioA->point2D();
    Vec3r b = ioB->point2D();
    SVertex *svA, *svB;

    Vec3r newpoint3d,newpoint2d;
    vector<SVertex*> intersections;
    real t,T;
    for(vector<Vec2r>::const_iterator p=iParameters.begin(),pend=iParameters.end();
    p!=pend;
    p++)
    {
      T=(*p)[0];
      t=(*p)[1];

      if((t < 0) || (t > 1))
        cerr << "Warning: Intersection out of range for edge " << ioA->getId() << " - " << ioB->getId() << endl;
    
      // compute the 3D and 2D coordinates for the intersections points:
      newpoint3d = Vec3r(A + T*(B-A));
      newpoint2d = Vec3r(a + t*(b-a));

      // create new SVertex:
      // (we keep B's id)
      SVertex* newVertex = new SVertex(newpoint3d, ioB->getId());
      newVertex->SetPoint2D(newpoint2d);
      
      // Add this vertex to the intersections list:
      intersections.push_back(newVertex);

      // Add this vertex to this sshape:
      AddNewVertex(newVertex);
    }
    
    for(vector<SVertex*>::iterator sv=intersections.begin(),svend=intersections.end();
    sv!=svend;
    sv++)
    {
      svA = fe->vertexA();
      svB = fe->vertexB();
        
      // We split edge AB into AA' and A'B. A' and A'B are created.
      // AB becomes (address speaking) AA'. B is updated.
      //--------------------------------------------------
      // The edge AB becomes edge AA'.
      (fe)->SetVertexB((*sv));
      // a new edge, A'B is created.
      FEdge *newEdge;
      if (fe->getNature() & Nature::ALL_INTERSECTION)
	{
	  newEdge = new FEdgeIntersection((*sv), svB);
	  FEdgeIntersection * se = dynamic_cast<FEdgeIntersection*>(newEdge);
	  FEdgeIntersection * fes = dynamic_cast<FEdgeIntersection*>(fe);
	  se->SetMaterialIndex(fes->materialIndex());
	  se->SetFaces(fes->getFace1(), fes->getFace2());


#ifdef DEBUG_INTERSECTION
	  void debugFES(FEdgeIntersection * newEdge);
	  
	  debugFES(se);
	  debugFES(fes);
#endif



	}
      else
      if(fe->isSmooth()){
        newEdge = new FEdgeSmooth((*sv), svB);
        FEdgeSmooth * se = dynamic_cast<FEdgeSmooth*>(newEdge);
        FEdgeSmooth * fes = dynamic_cast<FEdgeSmooth*>(fe);
        se->SetMaterialIndex(fes->materialIndex());
      }else{
        newEdge = new FEdgeSharp((*sv), svB);
        FEdgeSharp * se = dynamic_cast<FEdgeSharp*>(newEdge);
        FEdgeSharp * fes = dynamic_cast<FEdgeSharp*>(fe);
        se->SetaMaterialIndex(fes->aMaterialIndex());
        se->SetbMaterialIndex(fes->bMaterialIndex());
	se->SetEdge(fes->edge());
      }
        
      newEdge->SetNature((fe)->getNature());
      

      // to build a new chain:
      AddChain(newEdge);
      // add the new edge to the sshape edges list.
      AddEdge(newEdge);
      // add new edge to the list of new edges passed as argument:
      ioNewEdges.push_back(newEdge);

      // update edge A'B for the next pointing edge
      newEdge->SetNextEdge((fe)->nextEdge());
      fe->nextEdge()->SetPreviousEdge(newEdge);
      Id id(fe->getId().getFirst(), fe->getId().getSecond()+1);
      newEdge->SetId(fe->getId());
      fe->SetId(id);

      // update edge AA' for the next pointing edge
      //ioEdge->SetNextEdge(newEdge);
      (fe)->SetNextEdge(NULL);

      // update vertex pointing edges list:
      // -- vertex B --
      svB->Replace((fe), newEdge);
      // -- vertex A' --
      (*sv)->AddFEdge((fe));
      (*sv)->AddFEdge(newEdge);
    }
          
    }

  /* splits an edge into 2 edges. The new vertex and edge are added
   *  to the sshape list of vertices and edges
   *  a new chain is also created.
   *  returns the new edge.
   *    ioEdge
   *      The edge that gets splitted
   *    newpoint
   *      x,y,z coordinates of the new point.
   */
FEdge* SShape::SplitEdgeIn2(FEdge* ioEdge, SVertex * ioNewVertex)
    {
      SVertex *A = ioEdge->vertexA();
      SVertex *B = ioEdge->vertexB();

      
      // We split edge AB into AA' and A'B. A' and A'B are created.
      // AB becomes (address speaking) AA'. B is updated.
      //--------------------------------------------------
      
      // a new edge, A'B is created.
      FEdge *newEdge;
      if (ioEdge->getNature() & Nature::ALL_INTERSECTION)
	{
	  newEdge = new FEdgeIntersection(ioNewVertex, B);
	  FEdgeIntersection * se = dynamic_cast<FEdgeIntersection*>(newEdge);
	  FEdgeIntersection * fes = dynamic_cast<FEdgeIntersection*>(ioEdge);
	  se->SetMaterialIndex(fes->materialIndex());
	  se->SetFaces(fes->getFace1(), fes->getFace2());

#ifdef DEBUG_INTERSECTION
	  void debugFES(FEdgeIntersection * newEdge);
	  
	  debugFES(se);
	  debugFES(fes);
#endif

	}else
      if(ioEdge->isSmooth()){
        newEdge = new FEdgeSmooth(ioNewVertex, B);
        FEdgeSmooth * se = dynamic_cast<FEdgeSmooth*>(newEdge);
        FEdgeSmooth * fes = dynamic_cast<FEdgeSmooth*>(ioEdge);
        se->SetMaterialIndex(fes->materialIndex());
	se->SetFace(fes->face());
      }else{
        newEdge = new FEdgeSharp(ioNewVertex, B);
        FEdgeSharp * se = dynamic_cast<FEdgeSharp*>(newEdge);
        FEdgeSharp * fes = dynamic_cast<FEdgeSharp*>(ioEdge);
        se->SetaMaterialIndex(fes->aMaterialIndex());
        se->SetbMaterialIndex(fes->bMaterialIndex());
	se->SetEdge(fes->edge());
      }
      newEdge->SetNature(ioEdge->getNature());
      
      
      if(ioEdge->nextEdge() != 0)
        ioEdge->nextEdge()->SetPreviousEdge(newEdge);

      // update edge A'B for the next pointing edge
      newEdge->SetNextEdge(ioEdge->nextEdge());
      // update edge A'B for the previous pointing edge
      newEdge->SetPreviousEdge(0); // because it is now a ViewVertex
      Id id(ioEdge->getId().getFirst(), ioEdge->getId().getSecond()+1);
      newEdge->SetId(ioEdge->getId());
      ioEdge->SetId(id);
   
      // update edge AA' for the next pointing edge
      ioEdge->SetNextEdge(0); // because it is now a TVertex

      // update vertex pointing edges list:
      // -- vertex B --
      B->Replace(ioEdge, newEdge);
      // -- vertex A' --
      ioNewVertex->AddFEdge(ioEdge);
      ioNewVertex->AddFEdge(newEdge);

      // to build a new chain:
      AddChain(newEdge);
      AddEdge(newEdge); // FIXME ??
      
      // The edge AB becomes edge AA'.
      ioEdge->SetVertexB(ioNewVertex);

      // added by Aaron (looks redundant now, can probably be deleted)
      newEdge->SetVertexA(ioNewVertex);
      newEdge->SetVertexB(B);
      
      //      if(ioEdge->isSmooth()){
      //  ((FEdgeSmooth*)newEdge)->SetFace(((FEdgeSmooth*)ioEdge)->face());
      //}
      

      return newEdge;
    }


SShape::~SShape()
  {
    vector<SVertex*>::iterator sv,svend;
    vector<FEdge*>::iterator e,eend;
    if(0 != _verticesList.size())
      {
        for(sv=_verticesList.begin(),svend=_verticesList.end();
            sv!=svend;
            sv++)
          {
            delete (*sv);
          }
        _verticesList.clear();
      }

    if(0 != _edgesList.size())
      {
        for(e=_edgesList.begin(),eend=_edgesList.end();
            e!=eend;
            e++)
          {
            delete (*e);
          }
        _edgesList.clear();
      }

    //! Clear the chains list
    //-----------------------
    if(0 != _chains.size())
      {
        _chains.clear();
      }
  }

Vec3r SVertex::getColorID()
{ return  (*_FEdges.begin())->viewedge()->colorID(); };

/*
float SVertex::GetIsophoteDistance() const
{

  if (_FEdges.size() == 0)
    {
      printf("WARNING: TRIED TO GET ISOPHOTE DISTANCE OF SVERTEX WITHOUT FEDGES\n");
      return 1;
    }

  // should do something special for surface intersections that have two faces
  FEdge * fe = _FEdges[0];
  WFace * face = fe->getFace1();

  if (face == NULL)
    face = fe->getFace2();

  if (face == NULL)
    {
      printf("WARNING: TRIED TO GET ISOPHOTE DISTANCE OF FEDGE WITHOUT FACE\n");
      return 1;
    }

  Vec3r bary = face->barycentricCoordinates(getPoint3D());
  real weight = 0;
  real result = 0;
  for(int i=0;i<3;i++)
    {
      real ID =  face->GetVertex(0)->GetSurfaceNdotV();
      if (ID >= 0)
	{
	  result += bary[i] * ID;
	  weight += bary[i];
	}
    }
  if (weight == 0)
    return -1;
  else
    return result/weight;
}

*/


float ComputeSurfaceNdotV(WEdge * edge, Vec3r point)
{
  Vec3r dA = edge->GetaVertex()->GetVertex() - point;
  real distA = sqrt(dA*dA);

  Vec3r dB = edge->GetbVertex()->GetVertex() - point;
  real distB = sqrt(dB*dB);

  return (edge->GetaVertex()->GetSurfaceNdotV() * distB + edge->GetbVertex()->GetSurfaceNdotV() * distA)/(distA+distB);
}

//Vec3r ProjectToFace(WFace * face, Vec3r point)
//{
//  Vec3r v0 = face->GetVertex(0)->GetVertex();
//  return   viewpoint + (v0 -viewpoint)*face->GetNormal()) ;
//}

bool AdvanceToEdge(WFace * face, Vec3r currentPoint, int edgeIndex, Vec3r & nextPoint)
{
  Vec3r viewpoint = SilhouetteGeomEngine::GetViewpoint();
  
  WOEdge * edge = face->GetOEdge(edgeIndex);

  Vec3r A = edge->GetaVertex()->GetVertex();
  Vec3r B = edge->GetbVertex()->GetVertex();

  Vec3r edgedir = B-A;
  Vec3r planeNormal = (viewpoint - currentPoint) ^ face->GetNormal();
  real t;
  GeomUtils::intersection_test result = GeomUtils::intersectLinePlanePN(A, edgedir, planeNormal, currentPoint, t);

  //  printf("\te = %d, result = %d, want: %d, t = %f\n", edgeIndex, result, GeomUtils::DO_INTERSECT, t);
  if (result != GeomUtils::DO_INTERSECT || t < 0 || t > 1)
    return false;

  nextPoint = A + t*edgedir;

  return true;
  /*
  real dp = (nextPoint - currentPoint) * (viewpoint - currentPoint) ;

  printf("\tdp = %f\n", dp);

  return dp > 0;
  */
}

real ImageSpaceDistance(Vec3r p1, Vec3r p2)
{
  Vec2r q1 = SilhouetteGeomEngine::WorldToImage2(p1);
  Vec2r q2 = SilhouetteGeomEngine::WorldToImage2(p2);
  
  Vec2r d = q1-q2;
  return sqrt(d*d);
}


real SVertex::GetIsophoteDistance(real isovalue, int maxDistance) const
{
  if (_sourceVertex == NULL && _sourceEdge == NULL)
    return -1; // not handling the case of intersection SVertices yet

  if (!(getNature() & Nature::SILHOUETTE)) // only handle silhouette curves
    return -1;

  Vec3r startPoint = _Point3D;

  set<pair<WFace*,int> > possibleStartEdges;
  real currentNdotV;

  //  if (!(getNature() & Nature::SURFACE_INTERSECTION))
  //    printf("Isophote search Nature: %d\n", getNature());
  if (_sourceVertex != NULL)
    {
      currentNdotV = _sourceVertex->GetSurfaceNdotV();

      //      printf("Starting at vertex with n dot v = %f\n", currentNdotV);

      for(vector<WEdge*>::iterator it = _sourceVertex->GetEdges().begin(); it != _sourceVertex->GetEdges().end(); ++it)
	{
	  WFace * aface = (*it)->GetaFace();
	  WFace * bface = (*it)->GetbFace();
	  if (aface != NULL)
	    possibleStartEdges.insert( pair<WFace*,int>(aface, (aface->GetIndex(_sourceVertex)+1)%3));
	  if (bface != NULL)
	    possibleStartEdges.insert(pair<WFace*,int>(bface, (bface->GetIndex(_sourceVertex)+1)%3));
	}
    }
  else
    {
      currentNdotV = ComputeSurfaceNdotV(_sourceEdge, startPoint);

      WFace * faces[2] = { _sourceEdge->GetaFace(), _sourceEdge->GetbFace() };

      for(int i=0;i<2;i++)
	if (faces[i] != NULL)
	{
	  int v;
	  for(v=0;v<3;v++)
	    if (faces[i]->GetOEdge(v)->GetOwner() == _sourceEdge)
	      break;
	  assert(v<3);
	  
	  possibleStartEdges.insert(pair<WFace*,int>(faces[i], (v+1)%3));
	  possibleStartEdges.insert(pair<WFace*,int>(faces[i], (v+2)%3));
	}
    }

  if (currentNdotV > isovalue)
    return 0;

  if (currentNdotV < -0.001)
    return 0;

  Vec3r currentPoint;
  WEdge * currentEdge = NULL;
  WFace * currentFace = NULL;

  Vec3r lastPoint = startPoint;
  real lastNdotV = currentNdotV;

  for(set<pair<WFace*,int> >::iterator it = possibleStartEdges.begin(); it != possibleStartEdges.end(); ++it)
    {
      WFace * face = (*it).first;
      int e = (*it).second;
      if (AdvanceToEdge( face, startPoint, e, currentPoint))
	{
	  currentNdotV = ComputeSurfaceNdotV(face->GetOEdge(e)->GetOwner(), currentPoint);

	  //	  printf("\tlastNdotV = %f, currentNdotV = %f\n", lastNdotV, currentNdotV);

	  if (currentNdotV < lastNdotV)
	    continue;

	  currentEdge = face->GetOEdge(e)->GetOwner(); 
	  currentFace = face->GetBordingFace(e);

	  assert(GeomUtils::distPointSegment<Vec3r>( currentPoint, currentEdge->GetaVertex()->GetVertex(), currentEdge->GetbVertex()->GetVertex()) < 0.01);
	  break;
	}
    }

  if (currentEdge == NULL)
    return -1;

  // march over the surface
  int numSteps = 0;
  while (currentNdotV < isovalue && currentNdotV > -0.001 && numSteps < 100 && currentFace != NULL && ImageSpaceDistance(startPoint, currentPoint) < maxDistance)
    {
      numSteps ++;
      lastPoint = currentPoint;
      lastNdotV = currentNdotV;

      //      printf("advancing, current NdotV = %f\n", currentNdotV);

      bool result = false;
      int e;
      Vec3r nextPoint;
      for(e=0;e<3;e++)
	{
	  if (currentFace->GetOEdge(e)->GetOwner() == currentEdge)
	    continue;
	  result = AdvanceToEdge(currentFace, currentPoint, e, nextPoint);
	  //	  printf("\t advance result = %s\n", result ? "true" : "false");
	  if (result)
	    break;
	}
      
      assert(result);
      
      currentPoint = nextPoint;
      currentEdge = currentFace->GetOEdge(e)->GetOwner();
      currentNdotV = ComputeSurfaceNdotV(currentEdge, currentPoint);
      currentFace = currentFace->GetBordingFace(e);
    }

  Vec3r endpoint;

  if (currentNdotV > isovalue)
    {
      real t = (isovalue - lastNdotV)/(currentNdotV - lastNdotV);
      endpoint = (1-t) * lastPoint + t * currentPoint;
    }
  else
    if (currentNdotV < -0.001)
      {
	real t = -lastNdotV/(currentNdotV - lastNdotV);  // endpoint is the zero-crossing
	endpoint = (1-t)*lastPoint + t * currentPoint;
      }
    else
      endpoint = currentPoint;

  ViewMap * vm = ViewMap::getInstance();

  vm->addDebugPoint(DebugPoint::ISOPHOTE, endpoint, startPoint);

  real dist = ImageSpaceDistance(startPoint, endpoint);

  //  printf("\t result: dist = %f, lastNdotV = %f, currentNdotV = %f, numSteps = %d, currentFace = %08X\n", dist, lastNdotV, currentNdotV, numSteps, currentFace);

  return dist;
}


Vec2r SVertex::ImageSpaceNormal() const
{
  if (_Normals.size() == 0)
    return Vec2r(0,0);

  Vec3r normal = *(_Normals.begin());

  Vec3r pos = _Point3D;

  Vec2r imgNormal = SilhouetteGeomEngine::WorldToImage2(pos + normal) - SilhouetteGeomEngine::WorldToImage2(pos);

  imgNormal.normalize();

  return imgNormal;
}
