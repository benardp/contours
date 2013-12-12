#ifndef __PUNCHOUT_H__
#define __PUNCHOUT_H__

#include <vector>

using namespace std;
//typedef enum { VALID, INCONSISTENT, PUNCHOUT } POType;

#include "../winged_edge/WXEdge.h"
 
// Shewchuk's orientation code
extern "C"
{
  void exactinit(); // this must be called before any calls to orient
  //  real orient2d(real * real *, real*);
  real orient3d(real *,real*,real*,real*);
}


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
//  bool a1side = orient3d(P,Q,R,A1) > 0;
//  bool a2side = orient3d(P,Q,R,A2) > 0;

//  return a1side == a2side;

    real orient1 = orient3d(P,Q,R,A1);
    real orient2 = orient3d(P,Q,R,A2);

    if (orient1 == 0 || orient2 == 0)  // not sure this helps
        return false;

    return (orient1 > 0) == (orient2 > 0);
}

inline
bool sameSide(WFace * face, Vec3r A1, Vec3r A2)
{
  return sameSide(face->GetVertex(0)->GetVertex(), face->GetVertex(1)->GetVertex(), face->GetVertex(2)->GetVertex(), A1, A2);
}

extern "C"
{
  int tri_tri_intersection_test_3d(real p1[3], real q1[3], real r1[3], 
				   real p2[3], real q2[3], real r2[3],
				   int * coplanar, 
				   real source[3],real target[3]);
}

inline
bool intersectTriangleTriangle(WFace * face1, Vec3r P[3], bool & coplanar, Vec3r & A, Vec3r & B, 
			       int & faceIndA, int & edgeIndA, int & faceIndB, int & edgeIndB)
// faceInd/edgeInd tell us which edge each of A and B come from
// faceInd == 0 means it comes from face1, 1 means from P
{
  int coplanar1 = 0;
  real source[3], target[3];
  real t1[3][3], t2[3][3];
  for(int m=0;m<3;m++)
    for(int n=0;n<3;n++)
      {
	t1[m][n] = face1->GetVertex(m)->GetVertex()[n];
	t2[m][n] = P[m][n];
      }


  int result = tri_tri_intersection_test_3d(t1[0], t1[1], t1[2],
  					    t2[0], t2[1], t2[2],
  					    &coplanar1, source, target);

  A[0] = source[0];
  A[1] = source[1];
  A[2] = source[2];

  B[0] = target[0];
  B[1] = target[1];
  B[2] = target[2];

  coplanar = (coplanar1 ==1);
  
  if (!result)
    return false;

  // Determine which of face1's edges A and B came from.
  // A hacky post-hoc way to do it; it could also be done properly by incorporpating it into the intersection code

  real min_distA = DBL_MAX;
  real min_distB = DBL_MAX;

  for(int f=0;f<2;f++)
    {
      for(int e=0;e<3;e++)
	{
	  Vec3r v1, v2;
	  
	  if (f == 0)
	    {
	      v1 = face1->GetVertex(e)->GetVertex();
	      v2 = face1->GetVertex((e+1)%3)->GetVertex();
	    }
	  else
	    {
	      v1 = P[e];
	      v2 = P[(e+1)%3];
	    }
	  
	  real distA = GeomUtils::distPointSegment<Vec3r>(A, v1, v2);
	  if (distA < min_distA)
	    {
	      min_distA = distA;
	      faceIndA = f;
	      edgeIndA = e;
	    }
	  real distB = GeomUtils::distPointSegment<Vec3r>(B, v1, v2);
	  if (distB < min_distB)
	    {
	      min_distB = distB;
	      faceIndB = f;
	      edgeIndB = e;
	    }
	}
    }

  return true;
}



struct POBoundaryEdge
{
  Vec3r A,B; // endpoints of the boundary segment

  WFace * myFace;  // face on which this edge lies
  WFace * sourceFace; // face on which the source "inconsistent" boundary lies

  typedef enum { MESH_SIL, SMOOTH_SIL, DEPTH_CLIPPING } SourceType;

  SourceType sourceType;

  bool isRealBoundary; // is this edge a boundary separating PO region from valid region?
  // the fake ones are only kept around for debugging/visualization
  
  // ---- the following variables are used only for PO surface generation ----
  //      only for MESH_SIL sources that are real boundaries
  //      could have two versions of this class to save memory

  int POregionIndex;   // index of the region that this edge came from

  Vec3r Asrc, Bsrc; // source locations (3D points on "sourceFace" that project to A and B)

  // topological information:
  // each of the endpoints A and B may "come from" exactly one of the following sources:
  //    (a) Vertex of "sourceFace"
  //    (b) edge on "myFace"
  //    (c) clipping
  //    (d) point on edge of "sourceFace"
  // The following variables store topological information about the sources of A and B

  WVertex * AsrcVert, * BsrcVert;  // source vertices, or NULL if none
  int srcFaceEdge; // index of edge (Asrc,Bsrc) on source face, or -1
  int Aedge, Bedge; // index of edges that A and B lie on in myFace, or -1;
  bool isAdegen,isBdegen;   // true if this is a mesh sil, and srcVert is a vertex on BOTH faces
  // degeneracy should only correspond to cusps, I think.
  // I don't think B can be degenerate, but not sure. (this comment might be old)
  bool isAclipPoint, isBclipPoint; // is this endpoint due to clipping?
  // don't know yet if I'll use this variable, but it's helpful for debugging, at the very least

  POBoundaryEdge(Vec3r a, Vec3r b, WFace * myf, WFace * src, SourceType st, bool rb,
		 WVertex * Asv, WVertex * Bsv, Vec3r viewpoint, int e, int Ae, int Be,
		 bool Aclip,bool Bclip);

  bool projectPoint(Vec3r srcPoint, Vec3r viewpoint, Vec3r & projection);

  void print();
};

struct FacePOData
{
  int inconsistentRegion; //if this face contains inconsistency, index of that region, or -1
  bool inconsistentVertex[3]; // which vertices are inconsistent
  bool visitedForInconsistent;  // has this face been traversed as an inconsistent face yet?

  int POvisitIndex;   // has this face been traversed for a punchout region with the given index?

  set<WFace*> sourceFaces;  // faces containing silhouettes that generated punchoutedges on this face
  set<WFace*> targetFaces;  // faces that contain punch-out boundary due to silhouettes in this face
  //  set<int> POregions; // PO regions overlapping this triangle. for visualization only.

  vector<POBoundaryEdge*> POboundary;  // ONLY for boundary pieces that are the projection of mesh silhouettes. this comment might be wrong...

  int debugAge; // number of steps in the traversal since the inconsistent region

  // these variables are added as precomputation just to make IsInconsistent more efficient
  bool hasSilhouette;
  Vec3r silPt1, silPt2;  // two points on the smooth silhouette

  // ----------------------- methods --------------------------

  FacePOData()
  { 
    //  debugLastCrossOverRegion = -1; 
    debugAge = 10000;
    inconsistentRegion = -1;
    visitedForInconsistent = false;
    POvisitIndex = -1;
    inconsistentVertex[0] = inconsistentVertex[1] = inconsistentVertex[2] = false;
    hasSilhouette = false;
    //    debugAge = -1;
    //    regionIndex[0] = regionIndex[1] = regionIndex[2] = -1;
  }; 

  bool hasSourceFace(WFace *src)  {  return sourceFaces.find(src) != sourceFaces.end(); };
  bool hasTargetFace(WFace *tgt)  {  return targetFaces.find(tgt) != targetFaces.end(); };
  ~FacePOData(); // delete po boundary edges
};



struct InconsistentTri
{
  Vec3r P[3];
  WXFace * sourceFace;

  // Each inconsistent tri is formed from a subset of a mesh face.  The following two variables
  // keep track of the each *vertex* of the inconsistent tri came from

  // srcEdge: for Incons vertices that came from smooth silhouettes, 
  //          index of the source edge the silhouette point lies on;
  //          -1 if the point came from a triangle vertex
  int srcEdge[3];  

  // srcVertex: for Incons vertices that came from mesh vertices,
  //            index of the source face vertex
  //            -1 if it came from a silhouette point
  int srcVertex[3];

  // srcMeshEdge: for each cone edge: index of the triangle edge this comes from, if any
  //  (it might be possible to figure this out from the above variables, but it would be complex)
  int srcMeshEdge[3]; 
  //  bool isMeshSilOrCrease[3];
  
  InconsistentTri(WXFace * src, Vec3r p, Vec3r q, Vec3r r,
		  int srcEdge0, int srcEdge1, int srcEdge2,
		  int srcVertex0, int srcVertex1, int srcVertex2,
		  int srcMeshEdge0, int srcMeshEdge1, int srcMeshEdge2)
  { 
    sourceFace = src; P[0] = p; P[1] = q; P[2] = r; 
    srcEdge[0] = srcEdge0; srcEdge[1] = srcEdge1; srcEdge[2] = srcEdge2;
    srcVertex[0] = srcVertex0; srcVertex[1] = srcVertex1; srcVertex[2] = srcVertex2;
    srcMeshEdge[0] = srcMeshEdge0; srcMeshEdge[1] = srcMeshEdge1; srcMeshEdge[2] = srcMeshEdge2;
  };
  InconsistentTri(WXFace * src)
  { 
    sourceFace = src; 
    P[0] = src->GetVertex(0)->GetVertex();
    P[1] = src->GetVertex(1)->GetVertex();     
    P[2] = src->GetVertex(2)->GetVertex(); 
    srcEdge[0] = srcEdge[1] = srcEdge[2] = -1;  
    srcVertex[0] = 0; srcVertex[1] = 1; srcVertex[2] = 2;
    srcMeshEdge[0] = 0; srcMeshEdge[1] = 1; srcMeshEdge[2] =2;
  };

  WVertex * getVertex(int i) { assert (i>=0 && i<3); 
    return srcVertex[i] == -1 ? NULL: sourceFace->GetVertex(srcVertex[i]); }

  bool pointInside(Vec3r pt, Vec3r apex, bool clipping)
  {
    // three sign tests
    bool faceTests = 
      sameSide(apex, P[0], P[1], pt, P[2]) && 
      sameSide(apex, P[0], P[2], pt, P[1]) &&
      sameSide(apex, P[1], P[2], pt, P[0]);

    bool clippingTest = !clipping || (sameSide(sourceFace, pt, apex) != sourceFace->front());
				      
    return faceTests && clippingTest;
  }

  bool potentialPOBoundary(int e)
  {
    // for a "face" of the inconsistent tri:
    // is the source edge of this face (a smooth silhouette, mesh silhouette, or boundary), and thus a potential
    // punch-out boundary?

    int vertA = e;
    int vertB = (e+1)%3;

    // check if this comes from a smooth silhouette
    if (srcEdge[vertA] != -1 && srcEdge[vertB] != -1)
      return true;

    // is this a mesh edge?  (other case is an internal edge created when tesselating a trapezoid)
    if (srcMeshEdge[e] == -1)
      return false;

    WXFace * oppFace = (WXFace*)sourceFace->GetBordingFace(srcMeshEdge[e]);

    // check if this is a mesh boundary
    if (oppFace == NULL)
      return true;

    // check if this is a mesh silhouette
    return (sourceFace->front() != oppFace->front());
  }
};


#endif
