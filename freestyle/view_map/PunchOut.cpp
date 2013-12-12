#include <assert.h>

#include <map>

#include "ViewMapBuilder.h"
#include "PunchOut.h"

#include "../scene_graph/NodeGroup.h"
#include "../scene_graph/NodeShape.h"
#include "../scene_graph/VertexRep.h"
#include "../scene_graph/LineRep.h"
#include "../scene_graph/TriangleRep.h"

#include "../winged_edge/WXEdgeBuilder.h"
#include "../winged_edge/WFillGrid.h"

using namespace std;






extern "C"
{
  int tri_tri_intersection_test_3d(real p1[3], real q1[3], real r1[3], 
				   real p2[3], real q2[3], real r2[3],
				   int * coplanar, 
				   real source[3],real target[3]);
}


NodeGroup * punchOutDebugNode = NULL;

NodeShape * cuspRegionGeom = NULL; //used for debugging throughout

int debugRegionToShow = -1;  //use -1 to show all

extern NodeGroup * visDebugNode;
//void addDebugPoint(Vec3r point, NodeShape * node);

bool intersectTriConeTriangle(InconsistentTri & cone, WFace * face, Vec3r C, bool intersectsEdge[3],
			      vector<POBoundaryEdge*> & edges, bool clipping, bool debugPrintouts);

bool foundBoundaryCusp;


WFace * debugInconsFace = NULL;
WFace * debugTargetFace = NULL;

void ViewMapBuilder::ComputePunchOutRegions(WingedEdge & we, Grid * iGrid)
{
  printf("Computing punchout regions\n");

  vector<WFace*> startFaces;

  // =========== mark all inconsistent vertices/faces ================================
  for(vector<WShape*>::iterator wit = we.getWShapes().begin(); wit != we.getWShapes().end(); wit ++)
    for(vector<WFace*>::iterator fit = (*wit)->GetFaceList().begin(); fit != (*wit)->GetFaceList().end(); ++fit)
      {
	FacePOData * fd = new FacePOData;
	_ViewMap->setFacePOData(*fit,fd);

	// find all faces containing inconsistent regions
	for(int i=0;i<3;i++)
	  {
	    bool frontFacingVertex = ((WXVertex*)(*fit)->GetVertex(i))->ndotv() > 0;
	    fd->inconsistentVertex[i] = frontFacingVertex != (((WXFace*)(*fit))->front());
	  }	      

	if (fd->inconsistentVertex[0] || fd->inconsistentVertex[1] || fd->inconsistentVertex[2])
	  startFaces.push_back(*fit);
      }

  // ============================== setup the visualization ================================

  punchOutDebugNode = new NodeGroup;

  NodeShape * inconsistentNodes = new NodeShape;
  NodeShape * punchOutNodes = new NodeShape;
  cuspRegionGeom = new NodeShape;

  punchOutDebugNode->AddChild(inconsistentNodes);
  punchOutDebugNode->AddChild(punchOutNodes);
  punchOutDebugNode->AddChild(cuspRegionGeom);

  Material imat;
  imat.SetDiffuse(1,0,0,1);
  imat.SetEmission(0,0,0,0);
  imat.SetAmbient(0,0,0,0);

  Material pomat;
  pomat.SetDiffuse(0,0,1,1);
  pomat.SetEmission(0,0,0,0);
  pomat.SetAmbient(0,0,0,0);

  Material crmat;
  crmat.SetDiffuse(0,0,1,1);
  crmat.SetEmission(0,0,0,0);
  crmat.SetAmbient(0,0,0,0);

  inconsistentNodes->SetMaterial(imat);
  punchOutNodes->SetMaterial(pomat);
  cuspRegionGeom->SetMaterial(crmat);

  // =============================== main loop of algorithm =================================
  //

  // iterate over each inconsistent region


  int regionIndex = 0;   // each inconsistent region and punch-out region pair has a unique integer ID
  for(vector<WFace*>::iterator fit = startFaces.begin(); fit != startFaces.end(); ++fit)
    {
      if (_ViewMap->facePOData(*fit)->visitedForInconsistent)
	continue;
      
      vector<InconsistentTri> inconsistentCones; // faces overlapping this inconsistent region
      deque<WFace*> punchOutFaces;    // queue of faces to be visited for punchout

      // flood fill the inconsistent region:
      // visit all faces inside, determine the boundary, and add inconsistent faces to punchOutFaces
      bool smoothSilhouette = 
	FloodFillInconsistent(*fit,inconsistentCones, punchOutFaces, inconsistentNodes, regionIndex);

      // flood fill the PunchOut side, to visit the rest of the punched-out faces
      FloodFillPunchOut(punchOutFaces, inconsistentCones, regionIndex, punchOutNodes);

      // make a visualization
      //      if (regionIndex == debugRegionToShow || debugRegionToShow == -1 )
      //	MakePunchOutRegionVisualization(inconsistentCones, inconsistentNodes, punchOutNodes, regionIndex);

      for(vector<InconsistentTri>::iterator trit = inconsistentCones.begin(); 
	  trit != inconsistentCones.end(); ++trit)
	_ViewMap->addInconsistentTri(new InconsistentTri(*trit), regionIndex);

      ++regionIndex;
    }

  //  MakePunchOutFullVisualization(inconsistentNodes, punchOutNodes);


  // ================ generate new punch-out geometry for cusp regions ======================

  WXShape * poGeom = GenerateCuspRegionGeometry();

  if (poGeom != NULL)
    {
      // add the new geometry to the 3D Grid
      WFillGrid fillGridRenderer(iGrid, NULL);
      fillGridRenderer.addShape(poGeom);
      
      // add the shape to the entire 3D scene. 
      we.addWShape(poGeom);
    }

  printf("\n  ========== Done computing punchout regions: %d total   ==========\n", regionIndex);
}





bool ViewMapBuilder::FloodFillInconsistent(WFace * startFace,
					   vector<InconsistentTri> & inconsistentCones,
					   deque<WFace*> & punchOutFaces,
					   NodeShape * debugPunchOutNodes, int regionIndex)
{
  deque<WFace*> activeSet;

  activeSet.push_back(startFace);
  _ViewMap->facePOData(startFace)->visitedForInconsistent = true;

  bool smoothSilhouette = false; // does this inconsistent region connect somewhere to a smooth silhouette?

  while(!activeSet.empty())
    {
      WXFace * face = (WXFace*)activeSet.front();
      activeSet.pop_front();

      FacePOData * fd = _ViewMap->facePOData(face);
      
      //      if (fd->visitedForInconsistent)
      //	continue;

      assert(fd->visitedForInconsistent);
      fd->inconsistentRegion = regionIndex;
      
      Vec3r silPts[3];
      bool silEdge[3] = { false, false, false }; // which edges have some silhouette points

      assert(fd->inconsistentVertex[0] || fd->inconsistentVertex[1] || fd->inconsistentVertex[2]);

      //     bool faceNormalDotVPos = face2->normal() > 0;

      if (fd->inconsistentVertex[0] && fd->inconsistentVertex[1] && fd->inconsistentVertex[2])
	{
	  // entire face is inconsistent
	  
	  inconsistentCones.push_back(InconsistentTri((WXFace*)face));
	  fd->hasSilhouette = false;
	}
      else
	{
	  // need to find/tesselate the inconsistent subset of this face.

	  int numSil = 0;
	  // find the silhouette, if any
	  for(int e=0;e<3;e++)
	    {
	      // the two vertices of this edge
	      WXVertex * vA = (WXVertex*)(face->GetOEdge(e)->GetaVertex());
	      WXVertex * vB = (WXVertex*)(face->GetOEdge(e)->GetbVertex());
	      
	      // 3D locations of the face vertices
	      Vec3r pA = vA->GetVertex();
	      Vec3r pB = vB->GetVertex();
	      
	      // check if there's a silhouette point on this edge
	      real gA = vA->ndotv();
	      real gB = vB->ndotv();
	      bool isSilhouette = (gA < 0 && gB > 0) || (gA > 0 && gB < 0);
	      
	      // locate the silhouette point
	      Vec3r silPt;
	      if (isSilhouette)
		{
		  // a slightly more efficient formula than that given by (Hertzmann 1999)
		  real tA = gB / (gB - gA);
		  real tB = gA / (gA - gB);
		  assert(tA >= 0 && tA <=1 && tB>=0 && tB<=1);
		  silPt = tA * pA + tB * pB;
		  silPts[e] = silPt;
		  silEdge[e] = true;
		  numSil ++;
		}
	    }

	  assert(numSil == 2);

	  fd->hasSilhouette = true;
	  smoothSilhouette = true;

	  int incons[2] = {-1,1};  // list of inconsistent vertices
	  int numIncons = 0;

	  for(int i=0;i<3;i++)
	    if(fd->inconsistentVertex[i])
	      {
		incons[numIncons] = i;
		numIncons ++;
	      }

	  if (numIncons == 1)
	    {
	      int leftEdge = incons[0];
	      int rightEdge = (incons[0] +2)%3;
	      assert(silEdge[leftEdge] && silEdge[rightEdge]);

	      inconsistentCones.push_back(InconsistentTri((WXFace*)face, 
							  face->GetVertex(incons[0])->GetVertex(),
							  silPts[leftEdge], silPts[rightEdge],
							  -1, leftEdge, rightEdge,
							  incons[0],-1,-1,
							  leftEdge,-1,rightEdge));
	    }
	  else
	    {
	      assert(numIncons == 2);

	      int leftEdge = incons[0];
	      int rightEdge = (incons[0] +2)%3;
	      int oppEdge = (incons[0] +1)%3;
	      int nearEdge, meshEdge;
	      
	      if ( silEdge[leftEdge] )
		{
		  nearEdge = leftEdge;
		  meshEdge = rightEdge;
		}
	      else
		{
		  nearEdge = rightEdge;
		  meshEdge = leftEdge;
		}

	      assert(silEdge[nearEdge] && silEdge[oppEdge] && !silEdge[meshEdge]);
	      
	      // tesselate the trapezoid into two triangles
	      inconsistentCones.push_back(InconsistentTri((WXFace*)face, 
							  face->GetVertex(incons[0])->GetVertex(),
							  silPts[nearEdge], silPts[oppEdge],
							  -1, nearEdge, oppEdge,
							  incons[0], -1, -1,
							  nearEdge, -1, -1));
	      inconsistentCones.push_back(InconsistentTri((WXFace*)face,
							  face->GetVertex(incons[0])->GetVertex(),
							  face->GetVertex(incons[1])->GetVertex(),
							  silPts[oppEdge],
							  -1, -1, oppEdge,
							  incons[0], incons[1], -1,
							  meshEdge, oppEdge, -1));
	    }

	  // save the silhouette points too
	  if (silEdge[0] && silEdge[1])
	    {
	      fd->silPt1 = silPts[0];
	      fd->silPt2 = silPts[1];
		}
	  else if (silEdge[1] && silEdge[2])
	    {
	      fd->silPt1 = silPts[1];
	      fd->silPt2 = silPts[2];
	    }
	  else
	    {
	      assert(silEdge[0] && silEdge[2]);
	      fd->silPt1 = silPts[0];
	      fd->silPt2 = silPts[2];
	    }
	}

      // -------------------- add neighboring faces to the flood fill --------------------------

      for(int e=0;e<3;e++)
	{
	  WFace * oppFace = NULL;
	  
	  // get the neighboring face
	  if (face->GetOEdge(e)->GetaFace() == face)
	    oppFace = face->GetOEdge(e)->GetbFace();
	  else
	    {
	      assert(face->GetOEdge(e)->GetbFace() == face);
	      oppFace = face->GetOEdge(e)->GetaFace();
	    }

	  // mesh boundary
	  if (oppFace == NULL)
	    continue;
	      
	  // if the opposing face has INCONSISTENT vertices for the same vertices as the current one, add it to the active set
	  // transfer INCONSISTENT across mesh contours to mark PUNCHOUTS if necessary

	  // the two vertices of this edge
	  WXVertex * vA = (WXVertex*)(face->GetOEdge(e)->GetaVertex());
	  WXVertex * vB = (WXVertex*)(face->GetOEdge(e)->GetbVertex());
	  WXVertex * vC = (WXVertex*)(face->GetOEdge((e+1)%3)->GetbVertex()); // opposite vertex
	  
	  // 3D locations of the face vertices
	  Vec3r pA = vA->GetVertex();
	  Vec3r pB = vB->GetVertex();
	  Vec3r pC = vC->GetVertex();
	  
	  bool Ainconsistent = fd->inconsistentVertex[e];
	  bool Binconsistent = fd->inconsistentVertex[(e+1)%3];
	  int iAopp = oppFace->GetIndex(vA);
	  int iBopp = oppFace->GetIndex(vB);
	  bool Aopp_inconsistent = _ViewMap->facePOData(oppFace)->inconsistentVertex[iAopp];
	  bool Bopp_inconsistent = _ViewMap->facePOData(oppFace)->inconsistentVertex[iBopp];
	  
	  // if the opposing face has inconsistency in the same vertices as this one, 
	  // add it to the active set for this traversal
	  if ( ((Ainconsistent && Aopp_inconsistent) || (Binconsistent && Bopp_inconsistent)) &&
	       ! _ViewMap->facePOData(oppFace)->visitedForInconsistent)
	    {
	      activeSet.push_back(oppFace);
	      _ViewMap->facePOData(oppFace)->visitedForInconsistent = true;
	    }
	  
	  // check if the opposing face may have punch-outs, add it to the punch-out flood fill
	  if ( (Ainconsistent && !Aopp_inconsistent) || (Binconsistent && !Bopp_inconsistent) )
	    {
	      if (_ViewMap->facePOData(oppFace)->POvisitIndex != regionIndex)
		{		  
		  punchOutFaces.push_back(oppFace);
		  _ViewMap->facePOData(oppFace)->POvisitIndex = regionIndex;
		  _ViewMap->facePOData(oppFace)->debugAge = 0;
		}
	    }
	}
    }

  return smoothSilhouette;
}





void ViewMapBuilder::FloodFillPunchOut(deque<WFace*> & punchOutFaces, 
				       vector<InconsistentTri> & inconsistentCones,
				       int regionIndex, NodeShape * debugPunchOutNodes)
{
  int nFaces =0;
  while(!punchOutFaces.empty())
    {
      // pop the next face from the queue
      WFace * face = punchOutFaces.front();
      punchOutFaces.pop_front();

      FacePOData * fd = _ViewMap->facePOData(face);

      assert(fd->POvisitIndex == regionIndex);
      nFaces ++;


      int numIntersections = 0;
      Vec3r debugA, debugB;

      // for each triangle edge, does the PunchOut region intersect it (not counting intersections 
      // at vertices)?
      bool edgeIntersectsPO[3] = { false, false, false };
      
      // ------------------- find all intersections with the inconsistent cone ------------------------
      for(vector<InconsistentTri>::iterator it = inconsistentCones.begin(); it!=inconsistentCones.end(); 
	  ++it)
	{
	  InconsistentTri & cone = *it;
	  
	  WFace * sourceFace = cone.sourceFace;  //(WFace*)(*it)->userdata;
	  assert(sourceFace != NULL);

	  if (sourceFace == face)
	    continue;

	  // check if we've already added this source
	  //	  if (fd->hasSourceFace(sourceFace)) //sourceFaces.find(sourceFace) != fd->sourceFaces.end())
	  //	    continue;
	  
	  bool intersectsEdge[3] = { false, false, false };
	  vector<POBoundaryEdge*> edges;

	  bool debugMode = false;

	  if (sourceFace == debugInconsFace && face == debugTargetFace)
	    {
	      printf("************ intersecting debug case *********** \n");
	      debugMode = true;
	    }

	  // check if they intersect
	  bool result = intersectTriConeTriangle(cone, face, _viewpoint, intersectsEdge, edges, true,
						 debugMode);

#ifdef DEBUG_INCONSISTENT
	  if (!result)  // doing some debugging here
	    {
	      // check for the failure case
	      printf("checking\n");
	      fflush(stdout);

	      for(int e=0;e<3;e++)
		if (cone.srcMeshEdge[e] != -1 && cone.sourceFace != NULL
		    && face != NULL 
		    && cone.sourceFace->GetBordingFace(e) != face 
		    && (cone.sourceFace->GetBordingFace((e+1)%3) == face ||
			cone.sourceFace->GetBordingFace((e+2)%3) == face)
		    && (cone.srcMeshEdge[(e+1)%3] != -1 || cone.srcMeshEdge[(e+2)%3] != -1)
		    && (cone.srcEdge[e] != -1 || cone.srcEdge[(e+1)%3] != -1) 
		    && (cone.srcVertex[e] != -1 || cone.srcVertex[(e+1)%3] != -1)
		    && _ViewMap->facePOData(face)->inconsistentRegion == -1)
		  {
		    int v = cone.srcVertex[e] == -1 ? cone.srcVertex[(e+1)%3] : cone.srcVertex[e];
		    assert(v != -1);

		    printf("v = %d\n",v);
		    fflush(stdout);

		    WVertex * vert = cone.sourceFace->GetVertex(v);
		    assert(vert != NULL);

		    printf("v\n");
		    fflush(stdout);
		    if (vert == face->GetVertex(1) || vert == face->GetVertex(2) || 
			vert == face->GetVertex(0))
		      printf("    ----- MISSING INTERSECTION FOUND? area: %f ------\n", face->getArea());
		    printf("checked\n");
		    fflush(stdout);
		  }
	    }
#endif

	  if (!result)
	    continue;
	  
	  for(vector<POBoundaryEdge*>::iterator eit = edges.begin(); eit != edges.end(); ++eit)
	    {
	      //	      assert( ((*eit).A - (*eit).B).norm() > 0.0001);
	      (*eit)->POregionIndex = regionIndex;
	      fd->POboundary.push_back(*eit);
	    }

	  for(int e=0;e<3;e++)
	    if (intersectsEdge[e])
	      edgeIntersectsPO[e] = true;

	  // add the inconsistent triangle to the list of punch-out triangles, and vice versa
	  fd->sourceFaces.insert(sourceFace);
	  _ViewMap->facePOData(sourceFace)->targetFaces.insert(face);

	  //	  fd->POregions.insert(regionIndex);

	  // add the triangle to the list of triangles being punched-out by this source
	  
	  numIntersections ++;
	}

      // ------------- propagate labels across the mesh where possible -------------------------------

      // only visit neighbors if we have intersections; would be better to use intersectsEdge
      if (numIntersections == 0)
	continue;

      for(int e=0;e<3;e++)
	{
	  if (!edgeIntersectsPO[e])
	    continue;

	  // find neighboring face
	  WFace * oppFace = face->GetBordingFace(e);
	  
	  if (oppFace == NULL || _ViewMap->facePOData(oppFace)->POvisitIndex == regionIndex)
	    continue;

	  punchOutFaces.push_back(oppFace);
	  _ViewMap->facePOData(oppFace)->POvisitIndex = regionIndex;
	  int newAge = fd->debugAge + 1;
	  if (_ViewMap->facePOData(oppFace)->debugAge >= newAge)
	    _ViewMap->facePOData(oppFace)->debugAge = newAge;
	}
    }

  //  printf("PO Region %d: %d faces.\n", regionIndex, nFaces);
}

/*
void ViewMapBuilder::MakePunchOutFullVisualization(NodeShape * inconsistentNodes, NodeShape * punchOutNodes)
{
  for(map<WFace*, FacePOData*>::iterator it = _facePOData.begin(); it!= _facePOData.end(); ++it)
    {
      WFace * face = (*it).first;
      FacePOData * fd = (*it).second;


      // ----------------------- draw punch-out boundary curve -------------------
      
      for(vector<POBoundaryEdge*>::iterator pit = fd->POboundary.begin();pit != fd->POboundary.end(); ++pit)
	{
	  bool isReal = (*pit)->isRealBoundary;

	  LineRep * line =new LineRep((*pit).A, (*pit).B);
	  line->SetWidth(isReal ? 5 : 2);
	  line->ComputeBBox();
	  punchOutNodes->AddRep(line);
	}
    }  
}

void ViewMapBuilder::MakePunchOutRegionVisualization(vector<InconsistentTri> & inconsistentCones,
					       NodeShape * inconsistentNodes, NodeShape * punchOutNodes,
					       int regionIndex)
{

  // ----------------------- draw inconsistent region -------------------
  Vec3r red(1,0,0);
  for(vector<InconsistentTri>::iterator it = inconsistentCones.begin();
      it != inconsistentCones.end(); ++it)
    {
      TriangleRep * tri = new TriangleRep((*it).P[0],red, (*it).P[1],red, (*it).P[2],red);
      inconsistentNodes->AddRep(tri);
    }
}  
*/



////// Functions for use during ray-testing, once all inconsistent/punch-out data structures have been computed ////////////

bool ViewMapBuilder::IsInconsistentPoint(WFace * face, Vec3r point, Vec3r projectionDirection)
// assumes the point is inside the triangle
{
  FacePOData * fd = _ViewMap->facePOData(face);

  // quick-reject
  if (fd->inconsistentRegion == -1)
    return false;

  // no silhouette but inconsistent, so the entire face is inconsistent
  if (!fd->hasSilhouette)
    return true;

  /*
  // check if the point is inconsistent
  assert(dynamic_cast<WXFace*>(face) != NULL);
  WXFace * face2 = (WXFace*)face;

  // this is the straightforward test:
  real g = face2->interpolatedNdotV(point);
  
  if ( (face2->front() && g < 0) || (!face2->front() && g > 0))
    return true;

  return false;
  */

  // A test that might be more numerically robust.  avoids both projection and computing barycentric coordinates.

  // Given an arbitrary 3D point "point", we could determine inconsistency by orthogonal projection to the face, and then determining which side of the silhouette it's on.  This is equivalent to the following:
  // Construct a plane containing the silhouette points and the face normal, and determine which side of this plane "point" is on.  Which is equivalent to the following.

  // the "decision plane" constains three points: the two silhouette points, and a third arbitrarily-chosen point:
  // all of this could be precomputed in the data structure for efficiency

  Vec3r dir = (projectionDirection==Vec3r(0,0,0) ? face->GetNormal() : projectionDirection);

  real silDist = (fd->silPt2 - fd->silPt1).norm();
  Vec3r pt3 = fd->silPt1 + dir * silDist;  // third point on the decision plane
  
  // pick a vertex that is inconsistent
  int ivert = -1;
  for(int i=0;i<3;i++)
    if (fd->inconsistentVertex[i])
      {
	ivert = i; 
	break;
      }
  assert(ivert != -1);
  
  // now, check if the test point is on the same side of the decision plane

  return sameSide(fd->silPt1, fd->silPt2, pt3, face->GetVertex(ivert)->GetVertex(), point);
}

bool ViewMapBuilder::IsPunchOutPoint(WFace * face, Vec3r testPoint, set<int> * regionIndices)
// for use in ray-testing: test if a point on a given face is a punchout
{
  FacePOData * fd = _ViewMap->facePOData(face);
  
  // test the point against all the inconsistent triangles that project to this edge.
  for(set<WFace*>::iterator it = fd->sourceFaces.begin(); it != fd->sourceFaces.end(); ++it)
    //for(set<WFace*>::iterator it = fd->sourceFaces.begin(); it!=fd->sourceFaces.end(); ++it)
    {
      WFace * sourceFace = (*it);

      // project test point to the triangle
      real t, u, v;
      Vec3r dir = testPoint - _viewpoint;
      Vec3r v0 = sourceFace->GetVertex(0)->GetVertex();
      Vec3r v1 = sourceFace->GetVertex(1)->GetVertex();
      Vec3r v2 = sourceFace->GetVertex(2)->GetVertex();
      bool result = GeomUtils::intersectRayTriangle(_viewpoint, dir, v0,v1,v2, t, u, v);   // why is this call-by-reference?

      Vec3r sourcePoint = _viewpoint + t *dir;

      // this is a punchout if it projects to the test triangle, and the test triangle says it's inconsistent, and the projection point is on the correct side of the target point

      // the inconsistency test could probably be done on the original test point by constructing a decision plane containing the view vector.  this might be more robust
      if (result && IsInconsistentPoint(sourceFace, sourcePoint))
	{
	  bool sourceFurtherFromCamera = dir.norm() < (sourcePoint - _viewpoint).norm();
	  bool frontFace = ((WXFace*)sourceFace)->front();

	  if (sourceFurtherFromCamera != frontFace)
	    {
	      if (regionIndices == NULL)
		return true;
	      else
		regionIndices->insert(_ViewMap->facePOData(sourceFace)->inconsistentRegion);
	    }
	}
    }
  
  return (regionIndices != NULL && regionIndices->size() > 0);
}



bool ViewMapBuilder::IsPunchOutPointRay(WFace * face, Vec3r rayEnd, set<int> * regionIndices)
// for use in ray-testing: test if a point on a given face is a punchout, where the point is
// defined as the intersection of the face with the ray (rayEnd,viewpoint).
// an attempt at being a little more robust when checking for punchouts.
// nope, doesn't seem to help.
{
  FacePOData * fd = _ViewMap->facePOData(face);

  Vec3r dir = rayEnd - _viewpoint;
  real t, u, v;

  //  Vec3r v0 = face->GetVertex(0)->GetVertex();
  //  Vec3r v1 = face->GetVertex(1)->GetVertex();
  //  Vec3r v2 = face->GetVertex(2)->GetVertex();

  // intersect the ray with the face
  //  bool r = GeomUtils::intersectRayTriangle(_viewpoint, dir, v0,v1,v2, t,u,v);
  //  assert(r);
  
  GeomUtils::intersection_test result;


  // intersect the ray with the face, ignoring any bounds checks
  result = GeomUtils::intersectLinePlanePN(_viewpoint, dir, face->GetNormal(), 
					   face->GetVertex(0)->GetVertex(),t);
  assert(result == GeomUtils::DO_INTERSECT);

  Vec3r testPoint = _viewpoint + t * dir;

  Vec3r rayDir = rayEnd - _viewpoint;
  rayDir = rayDir / rayDir.norm();
  

  
  // test the point against all the inconsistent triangles that project to this edge.
  for(set<WFace*>::iterator it = fd->sourceFaces.begin(); it != fd->sourceFaces.end(); ++it)
    //for(set<WFace*>::iterator it = fd->sourceFaces.begin(); it!=fd->sourceFaces.end(); ++it)
    {
      WFace * sourceFace = (*it);

      // project test point to the triangle
      Vec3r v0 = sourceFace->GetVertex(0)->GetVertex();
      Vec3r v1 = sourceFace->GetVertex(1)->GetVertex();
      Vec3r v2 = sourceFace->GetVertex(2)->GetVertex();
      bool result = GeomUtils::intersectRayTriangle(_viewpoint, dir, v0,v1,v2, t, u, v);   // why is this call-by-reference?

      Vec3r sourcePoint = _viewpoint + t *dir;

      // this is a punchout if it projects to the test triangle, and the test triangle says it's inconsistent, and the projection point is on the correct side of the target point
      if (result && IsInconsistentPoint(sourceFace, rayEnd, rayDir))
	{
	  //	  bool sourceFurtherFromCamera = (testPoint-_viewpoint).norm() < (sourcePoint - _viewpoint).norm();
	  bool sourceFurtherFromCamera = sameSide(sourceFace,testPoint,_viewpoint);
	  bool frontFace = ((WXFace*)sourceFace)->front();

	  if (sourceFurtherFromCamera != frontFace)
	    {
	      if (regionIndices == NULL)
		return true;
	      else
		regionIndices->insert(_ViewMap->facePOData(sourceFace)->inconsistentRegion);
	    }
	}
    }
  
  return (regionIndices != NULL && regionIndices->size() > 0);
}








bool intersectPlanarConeTriangle(WFace * f, Vec3r P, Vec3r Q, Vec3r C, 
				 int Pedge, int Pvertex, 
				 Vec3r & A, Vec3r & B, int & Aedge, int & Bedge,
				 bool & AprojectsP, bool & AprojectsQ, bool & BprojectsP, bool &BprojectsQ,
				 bool & AonPlane)
// compute the intersection segment (AB) of a triangle (mesh face) and a planar cone
// The planar cone is defined by three points (C, P, Q) and is given by the parametric equation
//         f(a,b) = C + a (P-C) + b (Q-C)   for all a>=0, b>=0
// Special cases occur when point P topologically lies exactly on an edge or vertex of the triangle face.  
//    When this occurs, the args Pedge or Pvertex (not both) index the vertex/edge, or -1 otherwise
// Returns true if there is a non-trivial intersction.  The resulting edge is stored in (A,B).
// "Aedge" indicates which edge of the triangle face A lies on, or -1 if none or topologically on a vertex
// "Bedge" indicates which edge of the triangle face A lies on, or -1 if none or topologically on a vertex
// "AonPlane" indicate whether the resulting A point is topologically on the plane of the source face
// the "_projects_" variables indicate correspondence between variables
{
  // ---------- compute the intersection of the cone with the plane containing the face ----------
  // This will either be a line segment XY, or else a ray X+tV, determined by the value of "isRay"

  bool debug = false;

  //  Vec3r A,B,V;  
  Vec3r V;
  bool isRay = false;
  int Avertex = -1; // vertex that coincides with A, if any

  AonPlane = false;
  Aedge = -1;  // edge that A intersects, if any (not counting intersection at vertices)
  Bedge = -1;  // edge that B intersects, if any
  AprojectsP = false; AprojectsQ = false;
  BprojectsP = false; BprojectsQ = false;

  // compute direction vector of intersection line

  Vec3r PC = P-C;
  Vec3r QC = Q-C;

  V = (PC^QC)^f->GetNormal();

  //  assert(V.norm() > 1e-16);
  if (V.norm() == 0)
    {
      printf("WARNING: IGNORING DEGENERATE CONE FACE\n");
      return false;
    }
  V.normalize();

  if (debug)
    printf("V = [%f %f %f]\n", V[0], V[1], V[2]);

  // ------- project P to the triangle's plane to get X

  if (Pvertex != -1 || Pedge != -1)
    {
      A = P;
      isRay = false;
      AonPlane = true;
      AprojectsP = true;

      Aedge = Pedge;
      Avertex = Pvertex;
    }
  else
    {
      real t;
      GeomUtils::intersection_test result = GeomUtils::intersectRayPlanePN(C, PC, f->GetNormal(), f->GetVertex(0)->GetVertex(), t);//,1e-16);

      if (debug)
	printf("intersectP = %s\n", result == GeomUtils::DO_INTERSECT? "true":"false");

      if (result == GeomUtils::DO_INTERSECT)
	{
	  A = C + t * PC;
	  isRay = false;
	  AprojectsP = true;

	  if (debug)
	    printf("A = [%f %f %f]\n", A[0], A[1], A[2]);
	}
      else
	{
	  isRay = true;

	  if ((P - Q) * V < 0)
	    V = -1*V;
	}
    }

  // ------- project Q to the plane to get Y
  real t;
  GeomUtils::intersection_test result = GeomUtils::intersectRayPlanePN(C, QC, f->GetNormal(), f->GetVertex(0)->GetVertex(), t);//, 1e-16);

  if (debug)
    printf("intersectQ = %s\n", result == GeomUtils::DO_INTERSECT? "true":"false");

  if (result == GeomUtils::DO_INTERSECT)
    {
      if (isRay)
	{
	  A = C + t*QC;
	  AprojectsQ = true;
	  AonPlane = false;
	  Avertex = -1;
	}
      else
	{
	  B = C + t*QC;
	  BprojectsQ = true;
	}
    }
  else
    {
      // for there to be an intersection, at least one of the rays (PC,QC) must intersect the plane       
      if (isRay) 
	return false;

      isRay = true;

      if ((Q - P) * V < 0)
	V = -1*V;
    }


  //  printf("================================== PERFORMING CLIPPING\n");
  //  printf("p1 = [%f %f %f], p2 = [%f %f %f], p3 = [%f %f %f]\n",
  //	 f->GetVertex(0)->GetVertex()[0],	 f->GetVertex(0)->GetVertex()[1],	 f->GetVertex(0)->GetVertex()[2],
  //	 f->GetVertex(1)->GetVertex()[0],	 f->GetVertex(1)->GetVertex()[1],	 f->GetVertex(1)->GetVertex()[2],
  //	 f->GetVertex(2)->GetVertex()[0],	 f->GetVertex(2)->GetVertex()[1],	 f->GetVertex(2)->GetVertex()[2]);


  // ------------------------------- clip AB/AV to the triangle  -----------------------------

  for(int e=0;e<3;e++)
    {
      //     Vec2r oppVec2d = vertex2d[oppVertexInd] - X2D;
      //      Vec3r oppPt = f->GetVertex(oppVertexInd)->GetVertex();
      Vec3r edgeVec = f->GetVertex(e)->GetVertex() - f->GetVertex((e+1)%3)->GetVertex();
      edgeVec.normalize();
      
      // construct a signed representation for the half-space 
      Vec3r halfSpacePoint = (f->GetVertex(e)->GetVertex() + f->GetVertex((e+1)%3)->GetVertex())/2;
      //      Vec3r halfSpaceNormal = f->GetVertex((e+2)%3)->GetVertex() - halfSpacePoint;
      //halfSpaceNormal = halfSpaceNormal - edgeVec * (halfSpaceNormal * edgeVec);
      //    halfSpaceNormal.normalize();
      Vec3r halfSpaceNormal = edgeVec ^ f->GetNormal();
      halfSpaceNormal.normalize();
      if (halfSpaceNormal * (f->GetVertex((e+2)%3)->GetVertex() - halfSpacePoint) < 0)
	halfSpaceNormal = -1*halfSpaceNormal;
	//	f->GetNormal() ^ edgeVec;
	//		halfSpaceNormal = -halfSpaceNormal;

      // real-check my indexing
      assert(f->GetVertex((e+2)%3) != f->GetOEdge(e)->GetaVertex() && 
	     f->GetVertex((e+2)%3) != f->GetOEdge(e)->GetbVertex());

      if (debug)
	{
	  printf("face = [%f %f %f; %f %f %f; %f %f %f]\n",
		 f->GetVertex(0)->GetVertex()[0],f->GetVertex(0)->GetVertex()[1],f->GetVertex(0)->GetVertex()[2],
		 f->GetVertex(1)->GetVertex()[0],f->GetVertex(1)->GetVertex()[1],f->GetVertex(1)->GetVertex()[2],
		 f->GetVertex(2)->GetVertex()[0],f->GetVertex(2)->GetVertex()[1],f->GetVertex(2)->GetVertex()[2]);

	  printf("A = [%f %f %f], B = [%f %f %f]\n", A[0],A[1],A[2], B[0], B[1], B[2]);
	  printf("P = [%f %f %f], Q = [%f %f %f]\n", P[0],P[1],P[2], Q[0], Q[1], Q[2]);
	  printf("C = [%f %f %f]\n", C[0],C[1],C[2]);
	  printf("hspt = [%f %f %f], hsnm = [%f %f %f]\n", halfSpacePoint[0], halfSpacePoint[1], halfSpacePoint[2],
		 halfSpaceNormal[0], halfSpaceNormal[1], halfSpaceNormal[2]);
	  printf("Asign = %f, Bsign = %f\n", (A-halfSpacePoint)*halfSpaceNormal, (B-halfSpacePoint)*halfSpaceNormal);
	  printf("BprojectsQ = %s\n", BprojectsQ ? "true" : "false");
	}



      //      bool Ainside = ((A - halfSpacePoint) * halfSpaceNormal) >= 0;
      //      bool Binside = ((B-halfSpacePoint) * halfSpaceNormal) >= 0;

      Vec3r edgept1 = f->GetVertex(e)->GetVertex();
      Vec3r edgept2 = f->GetVertex((e+1)%3)->GetVertex();
      Vec3r edgevec = edgept2 - edgept1;
      Vec3r oppPt = f->GetVertex((e+2)%3)->GetVertex();
      Vec3r offsetPt = edgept1 + f->GetNormal() * edgevec.norm();
      Vec3r projN = (edgept1 - C)^(edgept2 - C);

      // maybe not stable to use C as the 3rd point?
      bool Ainside = sameSide(edgept1, edgept2, offsetPt, A, oppPt);
      bool Binside = sameSide(edgept1, edgept2, offsetPt, B, oppPt);
      
      // check if A is exactly on the edge
      if (Aedge == e || Avertex == e || Avertex == (e+1)%3)
	{
	  // is a ray: check if the ray points inside or outside the half-space
	  if (isRay && V * halfSpaceNormal <= 0)
	    {
	      if (debug)
		printf("AAA %d\n",e);
	      return false;
	    }
	  
	  // not a ray: check if B is inside or outside the half-space
	  if (!isRay && !Binside)
	    {
	      if (debug)
		printf("BBB %d\n",e);
	      return false;
	    }
	}
      else
	{
	  if (isRay)
	    {
	      // intersect the ray with the half-space
	      real t;
	      GeomUtils::intersection_test res = GeomUtils::intersectRayPlanePN(A,V,halfSpaceNormal,halfSpacePoint,t);



	      if (!Ainside && res != GeomUtils::DO_INTERSECT)
		{
		  if (debug)
		    printf("CCC %d\n",e);
		  return false;
		}

	      if (Ainside && res == GeomUtils::DO_INTERSECT)
		{
		  isRay = false;
		  B = A + t*V;
		  Bedge = e;
		  BprojectsP = BprojectsQ = false;
		}

	      if (!Ainside && res == GeomUtils::DO_INTERSECT)
		{
		  A = A + t*V;
		  Aedge = e;
		  AprojectsP = AprojectsQ = false;
		  AonPlane = false;
		  assert(Avertex == -1);
		}
	    }
	  else
	    {
	      // clip line seg to half-space
	      if (!Ainside && !Binside)
		{
		  if (debug)
		    printf("EEE %d\n",e);
		  return false;
		}
	      if (Ainside != Binside)   
		{
		  real t;
		  Vec3r BA = B-A;
		  real eps = 0.0001;  // test below acts weird with eps =1e-16????
		  //		  GeomUtils::intersection_test res = GeomUtils::intersectLinePlanePN(Ah,BAh,projN,C,t);
		  GeomUtils::intersection_test res = GeomUtils::intersectLinePlanePN(A,BA,halfSpaceNormal,halfSpacePoint,t);
		  if (res != GeomUtils::DO_INTERSECT || t > 1+eps || t<-eps)
		    {
		      Vec3r P = f->GetVertex(e)->GetVertex();
		      Vec3r Q = f->GetVertex((e+1)%3)->GetVertex();
		      Vec3r R = f->GetVertex((e+2)%3)->GetVertex();

		      printf("WARNING: impossible non-intersection: res = %d, t=%f\n", int(res),t);
		      printf("(DO_INTERSECT = %d)\n", int(GeomUtils::DO_INTERSECT));
		      printf("A = [%f %f %f], B = [%f %f %f]\n", A[0], A[1], A[2], B[0], B[1], B[2]);
		      printf("hsNormal = [%f %f %f], hsPoint =[%f %f %f]\n", halfSpaceNormal[0], halfSpaceNormal[1], halfSpaceNormal[2], halfSpacePoint[0], halfSpacePoint[1], halfSpacePoint[2]);
		      printf("P = [%f %f %f], Q = [%f %f %f], R = [%f %f %f]\n", 
			     P[0], P[1], P[2], Q[0], Q[1], Q[2], R[0], R[1], R[2]);
		      printf("Ainside = %s, Binside = %s\n", Ainside?"true":"false", Binside?"true":"false");
		      //		      return false;
		      //assert(0);
		    }
		  //		  assert(res == GeomUtils::DO_INTERSECT);
		  if (Ainside) 
		    {
		      B = A+ t*BA;
		      Bedge = e;
		      BprojectsP = BprojectsQ = false;
		    }
		  else
		    { 
		      A = A+t*BA;
		      Aedge = e;
		      AprojectsP = AprojectsQ = false;
		      AonPlane = false;
		      assert(Avertex == -1);
		    }

		}
	    }
	}
      //     printf("POSTCLIP:\n");
      //      printf("X = [%f %f %f], Y = [%f %f %f]\n", X[0],X[1],X[2], Y[0], Y[1], Y[2]);
      //      printf("hspt = [%f %f %f], hsnm = [%f %f %f]\n", halfSpacePoint[0], halfSpacePoint[1], halfSpacePoint[2],
      //	     halfSpaceNormal[0], halfSpaceNormal[1], halfSpaceNormal[2]);
      // printf("xsign = %f, ysign = %f\n", (X-halfSpacePoint)*halfSpaceNormal, (Y-halfSpacePoint)*halfSpaceNormal);
    }

  // various sanity checks:

  if (isRay)
    {
      printf("WARNING: INTERSECTION IS A RAY.\n");
      return false;
    }

  //  assert(!isRay);

  assert(Aedge != Bedge || Aedge == -1);

  assert(AprojectsP || AprojectsQ || Aedge != -1);
  assert(BprojectsP || BprojectsQ || Bedge != -1);

  assert(!(AprojectsP && BprojectsP));
  assert(!(AprojectsQ && BprojectsQ));

  return true;
}





bool intersectPlanarConeTriangleProj(WFace * f, Vec3r P, Vec3r Q, Vec3r C, 
				      int Pedge, int Pvertex, 
				      Vec3r & A, Vec3r & B, int & Aedge, int & Bedge,
				      bool & AprojectsP, bool & AprojectsQ, bool & BprojectsP, bool &BprojectsQ,
				      bool & AonPlane)
// compute the intersection segment (AB) of a triangle (mesh face) and a planar cone
// The planar cone is defined by three points (C, P, Q) and is given by the parametric equation
//         f(a,b) = C + a (P-C) + b (Q-C)   for all a>=0, b>=0
// Special cases occur when point P topologically lies exactly on an edge or vertex of the triangle face.  
//    When this occurs, the args Pedge or Pvertex (not both) index the vertex/edge, or -1 otherwise
// Returns true if there is a non-trivial intersction.  The resulting edge is stored in (A,B).
// "Aedge" indicates which edge of the triangle face A lies on, or -1 if none or topologically on a vertex
// "Bedge" indicates which edge of the triangle face A lies on, or -1 if none or topologically on a vertex
// "AonPlane" indicate whether the resulting A point is topologically on the plane of the source face
// the "_projects_" variables indicate correspondence between variables
{
  // ---------- compute the intersection of the cone with the plane containing the face ----------
  // This will either be a line segment XY, or else a ray X+tV, determined by the value of "isRay"

  bool debug = true;

  bool isRay = false;
  bool rayToP;
  int Avertex = -1; // vertex that coincides with A, if any

  AonPlane = false;
  Aedge = -1;  // edge that A intersects, if any (not counting intersection at vertices)
  Bedge = -1;  // edge that B intersects, if any
  AprojectsP = false; AprojectsQ = false;
  BprojectsP = false; BprojectsQ = false;

  // compute direction vector of intersection line

  Vec3r PC = P-C;
  Vec3r QC = Q-C;

  Vec3r Ah, Bh; // "projective coordinates" versions of A and B, in hopes of computing them more stably rather than via intermediate projections
  
  // ------- project P to the triangle's plane to get X

  if (Pvertex != -1 || Pedge != -1)
    {
      A = P;
      Ah = P;
      isRay = false;
      AonPlane = true;
      AprojectsP = true;

      Aedge = Pedge;
      Avertex = Pvertex;
    }
  else
    {
      real t;
      GeomUtils::intersection_test result = GeomUtils::intersectRayPlanePN(C, PC, f->GetNormal(), f->GetVertex(0)->GetVertex(), t,1e-16);

      if (debug)
	printf("intersectP = %s\n", result == GeomUtils::DO_INTERSECT? "true":"false");

      if (result == GeomUtils::DO_INTERSECT)
	{
	  Ah = P;
	  isRay = false;
	  AprojectsP = true;

	  if (debug)
	    printf("A = [%f %f %f]\n", A[0], A[1], A[2]);
	}
      else
	{
	  isRay = true;
	  rayToP = true;
	}
    }

  // ------- project Q to the plane to get Y
  real t;
  GeomUtils::intersection_test result = GeomUtils::intersectRayPlanePN(C, QC, f->GetNormal(), f->GetVertex(0)->GetVertex(), t, 1e-16);

  if (debug)
    printf("intersectQ = %s\n", result == GeomUtils::DO_INTERSECT? "true":"false");

  if (result == GeomUtils::DO_INTERSECT)
    {
      if (isRay)
	{
	  A = C + t*QC;
	  Ah = Q;
	  AprojectsQ = true;
	  AonPlane = false;
	  Avertex = -1;
	}
      else
	{
	  B = C + t*QC;
	  Bh = Q;
	  BprojectsQ = true;
	}
    }
  else
    {
      // for there to be an intersection, at least one of the rays (PC,QC) must intersect the plane       
      if (isRay) 
	return false;

      isRay = true;
      rayToP = false;

    }


  //  printf("================================== PERFORMING CLIPPING\n");
  //  printf("p1 = [%f %f %f], p2 = [%f %f %f], p3 = [%f %f %f]\n",
  //	 f->GetVertex(0)->GetVertex()[0],	 f->GetVertex(0)->GetVertex()[1],	 f->GetVertex(0)->GetVertex()[2],
  //	 f->GetVertex(1)->GetVertex()[0],	 f->GetVertex(1)->GetVertex()[1],	 f->GetVertex(1)->GetVertex()[2],
  //	 f->GetVertex(2)->GetVertex()[0],	 f->GetVertex(2)->GetVertex()[1],	 f->GetVertex(2)->GetVertex()[2]);


  // ------------------------------- clip AB/AV to the triangle  -----------------------------

  for(int e=0;e<3;e++)
    {
      //     Vec2r oppVec2d = vertex2d[oppVertexInd] - X2D;
      //      Vec3r oppPt = f->GetVertex(oppVertexInd)->GetVertex();
      Vec3r edgeVec = f->GetVertex(e)->GetVertex() - f->GetVertex((e+1)%3)->GetVertex();
      edgeVec.normalize();
      
      // construct a signed representation for the half-space 
      Vec3r halfSpacePoint = (f->GetVertex(e)->GetVertex() + f->GetVertex((e+1)%3)->GetVertex())/2;
      //      Vec3r halfSpaceNormal = f->GetVertex((e+2)%3)->GetVertex() - halfSpacePoint;
      //halfSpaceNormal = halfSpaceNormal - edgeVec * (halfSpaceNormal * edgeVec);
      //    halfSpaceNormal.normalize();
      Vec3r halfSpaceNormal = edgeVec ^ f->GetNormal();
      halfSpaceNormal.normalize();
      if (halfSpaceNormal * (f->GetVertex((e+2)%3)->GetVertex() - halfSpacePoint) < 0)
	halfSpaceNormal = -1*halfSpaceNormal;
	//	f->GetNormal() ^ edgeVec;
	//		halfSpaceNormal = -halfSpaceNormal;

      // real-check my indexing
      assert(f->GetVertex((e+2)%3) != f->GetOEdge(e)->GetaVertex() && 
	     f->GetVertex((e+2)%3) != f->GetOEdge(e)->GetbVertex());

      if (debug)
	{
	  printf("face = [%f %f %f; %f %f %f; %f %f %f]\n",
		 f->GetVertex(0)->GetVertex()[0],f->GetVertex(0)->GetVertex()[1],f->GetVertex(0)->GetVertex()[2],
		 f->GetVertex(1)->GetVertex()[0],f->GetVertex(1)->GetVertex()[1],f->GetVertex(1)->GetVertex()[2],
		 f->GetVertex(2)->GetVertex()[0],f->GetVertex(2)->GetVertex()[1],f->GetVertex(2)->GetVertex()[2]);

	  printf("A = [%f %f %f], B = [%f %f %f]\n", A[0],A[1],A[2], B[0], B[1], B[2]);
	  printf("P = [%f %f %f], Q = [%f %f %f]\n", P[0],P[1],P[2], Q[0], Q[1], Q[2]);
	  printf("C = [%f %f %f]\n", C[0],C[1],C[2]);
	  printf("hspt = [%f %f %f], hsnm = [%f %f %f]\n", halfSpacePoint[0], halfSpacePoint[1], halfSpacePoint[2],
		 halfSpaceNormal[0], halfSpaceNormal[1], halfSpaceNormal[2]);
	  printf("Asign = %f, Bsign = %f\n", (A-halfSpacePoint)*halfSpaceNormal, (B-halfSpacePoint)*halfSpaceNormal);
	  printf("BprojectsQ = %s\n", BprojectsQ ? "true" : "false");
	}



      //      bool Ainside = ((A - halfSpacePoint) * halfSpaceNormal) >= 0;
      //      bool Binside = ((B-halfSpacePoint) * halfSpaceNormal) >= 0;

      Vec3r edgept1 = f->GetVertex(e)->GetVertex();
      Vec3r edgept2 = f->GetVertex((e+1)%3)->GetVertex();
      Vec3r edgevec = edgept2 - edgept1;
      Vec3r oppPt = f->GetVertex((e+2)%3)->GetVertex();
      Vec3r offsetPt = edgept1 + f->GetNormal() * edgevec.norm();
      Vec3r projN = (edgept1 - C)^(edgept2 - C);

      bool Ainside = sameSide(edgept1, edgept2, C, Ah, oppPt);
      bool Binside = sameSide(edgept1, edgept2, C, Bh, oppPt);
      //      bool Ainside = sameSide(edgept1, edgept2, offsetPt, A, oppPt);
      //      bool Binside = sameSide(edgept1, edgept2, offsetPt, B, oppPt);
      
      // check if A is exactly on the edge
      if (Aedge == e || Avertex == e || Avertex == (e+1)%3)
	{
	  // is a ray: check if the ray points inside or outside the half-space
	  if (isRay && ((rayToP && sameSide(edgept1,edgept2,C,P,oppPt)) ||
			(!rayToP && sameSide(edgept1,edgept2,C,Q,oppPt))))
	    {
	      if (debug)
		printf("AAA %d\n",e);
	      return false;
	    }
	  
	  // not a ray: check if B is inside or outside the half-space
	  if (!isRay && !Binside)
	    {
	      if (debug)
		printf("BBB %d\n",e);
	      return false;
	    }
	}
      else
	{
	  if (isRay)
	    {
	      // intersect the ray with the half-space
	      real t;
	      //	      GeomUtils::intersection_test res = GeomUtils::intersectRayPlanePN(A,V,halfSpaceNormal,halfSpacePoint,t);
	      Vec3r vecdir = (rayToP ? P : Q) - Ah;
	      GeomUtils::intersection_test res = GeomUtils::intersectRayPlanePN(Ah,vecdir,projN,C,t);



	      if (!Ainside && res != GeomUtils::DO_INTERSECT)
		{
		  if (debug)
		    printf("CCC %d\n",e);
		  return false;
		}

	      if (Ainside && res == GeomUtils::DO_INTERSECT)
		{
		  isRay = false;
		  Bh = Ah + t*vecdir;
		  Bedge = e;
		  BprojectsP = BprojectsQ = false;
		}

	      if (!Ainside && res == GeomUtils::DO_INTERSECT)
		{
		  Ah = Ah + t*vecdir;
		  Aedge = e;
		  AprojectsP = AprojectsQ = false;
		  AonPlane = false;
		  assert(Avertex == -1);
		}
	    }
	  else
	    {
	      // clip line seg to half-space
	      if (!Ainside && !Binside)
		{
		  if (debug)
		    printf("EEE %d\n",e);
		  return false;
		}
	      if (Ainside != Binside)   
		{
		  real t;
		  Vec3r BAh = Bh - Ah;
		  real eps = 1e-16;
		  GeomUtils::intersection_test res = GeomUtils::intersectLinePlanePN(Ah,BAh,projN,C,t);
		  //		  GeomUtils::intersection_test res = GeomUtils::intersectLinePlanePN(A,BA,halfSpaceNormal,halfSpacePoint,t);
		  if (res != GeomUtils::DO_INTERSECT || t > 1+eps || t<-eps)
		    {
		      Vec3r P = f->GetVertex(e)->GetVertex();
		      Vec3r Q = f->GetVertex((e+1)%3)->GetVertex();
		      Vec3r R = f->GetVertex((e+2)%3)->GetVertex();

		      printf("WARNING: impossible non-intersection: res = %d, t=%f\n", int(res),t);
		      printf("(DO_INTERSECT = %d\n)\n", int(GeomUtils::DO_INTERSECT));
		      printf("Ah = [%f %f %f], Bh = [%f %f %f]\n", Ah[0], Ah[1], Ah[2], Bh[0], Bh[1], Bh[2]);
		      printf("hsNormal = [%f %f %f], hsPoint =[%f %f %f]\n", halfSpaceNormal[0], halfSpaceNormal[1], halfSpaceNormal[2], halfSpacePoint[0], halfSpacePoint[1], halfSpacePoint[2]);
		      printf("P = [%f %f %f], Q = [%f %f %f], R = [%f %f %f]\n", 
			     P[0], P[1], P[2], Q[0], Q[1], Q[2], R[0], R[1], R[2]);
		      printf("Ainside = %s, Binside = %s\n", Ainside?"true":"false", Binside?"true":"false");
		      //		      return false;
		      //		      assert(0);
		    }
		  //		  assert(res == GeomUtils::DO_INTERSECT);
		  if (Ainside) 
		    {
		      Bh = Ah+ t*BAh;
		      Bedge = e;
		      BprojectsP = BprojectsQ = false;
		    }
		  else
		    { 
		      Ah = Ah+t*BAh;
		      Aedge = e;
		      AprojectsP = AprojectsQ = false;
		      AonPlane = false;
		      assert(Avertex == -1);
		    }

		}
	    }
	}
      //     printf("POSTCLIP:\n");
      //      printf("X = [%f %f %f], Y = [%f %f %f]\n", X[0],X[1],X[2], Y[0], Y[1], Y[2]);
      //      printf("hspt = [%f %f %f], hsnm = [%f %f %f]\n", halfSpacePoint[0], halfSpacePoint[1], halfSpacePoint[2],
      //	     halfSpaceNormal[0], halfSpaceNormal[1], halfSpaceNormal[2]);
      // printf("xsign = %f, ysign = %f\n", (X-halfSpacePoint)*halfSpaceNormal, (Y-halfSpacePoint)*halfSpaceNormal);
    }

  // convert from projective to projected coordinates

  Vec3r Adir = Ah - C;
  GeomUtils::intersection_test Ares = GeomUtils::intersectLinePlanePN(C,Adir,f->GetNormal(),
								      f->GetVertex(0)->GetVertex(),t);
  assert(Ares == GeomUtils::DO_INTERSECT && t > 0);
  A = C + t * Adir;

  Vec3r Bdir = Bh - C;
  GeomUtils::intersection_test Bres = GeomUtils::intersectLinePlanePN(C,Bdir,f->GetNormal(),
								      f->GetVertex(0)->GetVertex(),t);
  assert(Bres == GeomUtils::DO_INTERSECT && t > 0);
  B = C + t * Bdir;

  // various sanity checks:

  assert(!isRay);

  assert(Aedge != Bedge || Aedge == -1);

  assert(AprojectsP || AprojectsQ || Aedge != -1);
  assert(BprojectsP || BprojectsQ || Bedge != -1);

  assert(!(AprojectsP && BprojectsP));
  assert(!(AprojectsQ && BprojectsQ));

  return true;
}
  







bool intersectTriConeTriangle(InconsistentTri & cone, WFace * meshFace, Vec3r C, bool intersectsEdge[3],
			      vector<POBoundaryEdge*> & edges, bool clipping, bool debugPrintouts)
// compute the intersection segments (AB) of a triangle (mesh face) and a triangular cone
// The planar cone is defined by base point C and the vertices {(P_i)} and is given by the parametric equation
//         f(a_i) = C + a_0 (P_0 - C) + a_1 (P_1 - C) + a_2 (P_2 - C)   for all a_i>=0
// Special cases occur when point P topologically lies exactly on an edge or vertex of the triangle meshFace, and are treated specially
// Returns true if there is a non-trivial intersction.  
// "intersectsEdge" indicates whether the returned line segment intersects that edge of the meshFace, except for 
//     topological intersections (indicated by isPonEdge), in order to remove vertex intersections
//
// The intersection occurs iff 
//  (a) for any given test point in the triangle, it lies in the cone, or 
//  (b) a "face" of the cone intersect the triangle  (except for coincident edges, which are degenerate intersections)
//
// A list of the intersection edges is returned.  For each edge, a boolean indicates whether the intersection
// is due to a boundary curve that is the image of a mesh silhouette.
//
// If the argument "clipping" is true, then the results are clipped to the plane containing the original face, either omitting geometry in front of or behind the face, depending
// on whether the original sourceFace is front-facing or backfacing.
{
  intersectsEdge[0] = intersectsEdge[1] = intersectsEdge[2] = false;

  bool debugPrint = debugPrintouts; //meshFace->getArea() > 40;

  bool intersection = false;

  WXFace * sourceFace = (WXFace*)cone.sourceFace;

  int adjacentEdge = -1;  // which edge of the meshFace borders the cone's source face, if any.
  int adjacentVertex = -1;  // if the faces are connected by a single vertex, which one on the mesh face
  int coneAdjVertex = -1;
  int oldSize = edges.size();

  // check if the cone and the target mesh face are adjacent on the mesh
  for(int k=0;k<3;k++)
    if (meshFace->GetBordingFace(k) == cone.sourceFace)
      {
	adjacentEdge = k;
	break;
      }

  if (adjacentEdge == -1)
    for(int k=0;k<3;k++)
      if (cone.srcVertex[k] != -1)
	for(int j=0;j<3;j++)
	  if (sourceFace->GetVertex(cone.srcVertex[k]) == meshFace->GetVertex(j))
	    {
	      adjacentVertex = j;
	      coneAdjVertex = k;
	      break;
	    }

  if (debugPrint)
    printf("** IntersectTriConeTri.  adjacentEdge = %d.  adjacentVertex = %d\n", adjacentEdge, adjacentVertex);

  // If the faces share an edge, then the target face all lies on one side of the target face or the other.
  // Check if the entire target triangle is clipped.
  if (clipping && adjacentEdge != -1)
    {
      int oppVertex = (adjacentEdge+2)%3;
      if ( sameSide(sourceFace, meshFace->GetVertex(oppVertex)->GetVertex(), C) ==
	   sourceFace->front())
	return false;
    }

  // if the faces share one vertex, and the other two are all in the clipped region, then there is no intersection
  if (clipping && adjacentVertex != -1)
    {
      int oppVertex1 = (adjacentVertex+1)%3;
      int oppVertex2 = (adjacentVertex+2)%3;
      bool clipped1 = (sameSide(sourceFace, meshFace->GetVertex(oppVertex1)->GetVertex(), C) ==
		       sourceFace->front());
      bool clipped2 = (sameSide(sourceFace, meshFace->GetVertex(oppVertex2)->GetVertex(), C) ==
		       sourceFace->front());

      if (clipped1 && clipped2)
	return false;
    }

  // ------------------ test the three "faces" of the cone -------------------------------
  for(int e=0;e<3;e++)
    {
      // this "face" is a planar cone defined by two vertices P,Q, and the apex C.
      int indP = e;
      int indQ = (e+1)%3;

      Vec3r P = cone.P[indP];
      Vec3r Q = cone.P[indQ];

      // --------  do a lot of bookkeeping to see how/whether the faces connect topologically ---------
      // whether P or Q are incident with any edges of the mesh face, and, if so, which ones:
      int Pvertex = -1, Qvertex = -1; // which vertex of the tri P/Q is on, if any 
      int Pedge = -1, Qedge = -1; // which edge of the tri P/Q is on, if any (not including vertices)

      if (cone.srcVertex[indP] != -1)
	{
	  WVertex * v = sourceFace->GetVertex(cone.srcVertex[indP]);
	  for(int i=0;i<3;i++)
	    if (meshFace->GetVertex(i) == v)
	      {
		Pvertex = i;
		break;
	      }
	}
      
      if (cone.srcVertex[indQ] != -1)
	{
	  WVertex * v = sourceFace->GetVertex(cone.srcVertex[indQ]);
	  for(int i=0;i<3;i++)
	    if (meshFace->GetVertex(i) == v)
	      {
		Qvertex = i;
		break;
	      }
	}
      
      if (debugPrint)
	printf("Edge %d. srcMeshEdge = %d. Pvertex = %d, Qvertex = %d\n", e, cone.srcMeshEdge[e],
	       Pvertex, Qvertex);

      // check if both P and Q lie on the target triangle
      if (Pvertex != -1 && Qvertex != -1)
	continue;
      
      // check if the cone edge intersects an edge of the mesh face (except at vertices)
      if (adjacentEdge != -1)
	{
	  if (cone.srcEdge[indP] != -1 &&
	      cone.sourceFace->GetBordingFace(cone.srcEdge[indP]) == meshFace)
	    Pedge = adjacentEdge;
	    //	    isPonEdge[adjacentEdge] = true;

	  if (cone.srcEdge[indQ] != -1 &&
	      cone.sourceFace->GetBordingFace(cone.srcEdge[indQ]) == meshFace)
	    Qedge = adjacentEdge;
	    //	    isQonEdge[adjacentEdge] = true;
	}
      
      /*      // check if both P and Q lie on the target triangle
      if ( (isPonEdge[0] || isPonEdge[1] || isPonEdge[2]) &&
	   (isQonEdge[0] || isQonEdge[1] || isQonEdge[2]))
	continue;
      */

      if (debugPrint)
	printf("Pedge = %d, Qedge = %d\n", Pedge, Qedge);

      assert( !(Pvertex != -1 && Pedge != -1));
      assert( !(Qvertex != -1 && Qedge != -1));

      bool PisOnEdge = (Pvertex != -1) || (Pedge != -1);
      bool QisOnEdge = (Qvertex != -1) || (Qedge != -1);

      // is PQ is on the edge between the triangles?
      if (PisOnEdge && QisOnEdge)
	continue;
	

      // --------  compute the intersection ------------- 
      Vec3r A,B;
      //      bool intersectsEdgeT[3] = {-1,-1,-1};
      bool result;
      bool AonPlane;  // is A/B topologically on the plane of the source face?
      bool AprojectsP, AprojectsQ, BprojectsP, BprojectsQ;
      int Aedge, Bedge;
      bool Aclip = false, Bclip = false;

      if (PisOnEdge)
	result = intersectPlanarConeTriangle(meshFace, P, Q, C, Pedge, Pvertex,
					     A, B, Aedge, Bedge,
					     AprojectsP, AprojectsQ, BprojectsP, BprojectsQ,
					     AonPlane);
      else
	result = intersectPlanarConeTriangle(meshFace, Q, P, C, Qedge, Qvertex,
					     A, B, Aedge, Bedge,
					     AprojectsQ, AprojectsP, BprojectsQ, BprojectsP,
					     AonPlane);

      if (debugPrint)
	printf("result: %s.  PisOnEdge: %s\n", result? "true":"false", PisOnEdge? "true":"false");

      // each endpoint should either (a) come from a source vertex, or (b) lie on a target face edge
      
      if (!result)
	continue;

      assert(AprojectsP || AprojectsQ || Aedge != -1);
      assert(BprojectsP || BprojectsQ || Bedge != -1);
      
      if (clipping)
	{
	  // clip the resulting edge to the plane of the source face.
	  
	  // for each of A and B: there are three options:
	  //  1. the point is behind the plane (w.r.t. the camera)
	  //  2. the point is in front of the plane
	  //  3. the point is topologically on one of the sourceFace edges (A only)
	  //
	  //  If the source face is front-facing, then we want to clip to keep the volume
	  // on the *far* side of the plane.  If the source is back-facing, we want to keep the near part
	  bool ArightSide = sameSide(sourceFace, A, C) != sourceFace->front();
	  bool BrightSide = sameSide(sourceFace, B, C) != sourceFace->front();
	  
	  // check if both endpoints in the clipped region 
	  if ( (AonPlane && !BrightSide) || (!AonPlane && !ArightSide && !BrightSide) )
	    {
	      if (debugPrint)
		printf("CLIPPING\n");
	      
	      continue;
	    }
	  
	  if (!AonPlane && (!ArightSide || !BrightSide))
	    {
	      // we need to clip
	      Vec3r BA = B - A;
	      real t;
	      GeomUtils::intersection_test result = 
		GeomUtils::intersectRayPlanePN(A, BA, sourceFace->GetNormal(),
					       sourceFace->GetVertex(0)->GetVertex(),
					       t,1e-16);
	      if (result != GeomUtils::DO_INTERSECT)
		{
		  printf("WARNING: EXPECTED INTERSECTION MISSING.  NUMERICAL UNDERFLOW PROBABLY.\n");
		  continue;
		}

	      //	      assert(result == GeomUtils::DO_INTERSECT);
	      if (!ArightSide)
		{
		  A = A + t * BA;
		  AprojectsP = AprojectsQ = false;
		  Aedge = -1;
		  Aclip = true;
		}
	      else
		{
		  B = A + t * BA;
		  BprojectsP = BprojectsQ = false;
		  Bedge = -1;
		  Bclip = true;
		}
	    }
	}
      
      if (debugPrint)
	printf("survived clipping\n");

      // no reason to generate the ones that are not potentialPOboundaries except for visualization
      bool boundary = cone.potentialPOBoundary(e);
      
      if (!boundary)  // comment this out in order to keep these edges in for visualization
	continue;

      
      intersection = true;
      
      // keep track of information about what this PO boundary edge is the projection of
      bool isFromSmoothSil = cone.srcEdge[indP] != -1 && cone.srcEdge[indQ] != -1;
      
      POBoundaryEdge::SourceType st = 
	(isFromSmoothSil ? POBoundaryEdge::SMOOTH_SIL : POBoundaryEdge::MESH_SIL);
      
      WVertex * AsrcVert = NULL;
      WVertex * BsrcVert = NULL;
      
      if (AprojectsP)
	AsrcVert = cone.getVertex(indP);
      else
	if (AprojectsQ)
	  AsrcVert = cone.getVertex(indQ);
      
      if (BprojectsP)
	BsrcVert = cone.getVertex(indP);
      else
	if (BprojectsQ)
	  BsrcVert = cone.getVertex(indQ);
      
      if (st == POBoundaryEdge::MESH_SIL)
	{
	  // check: each mesh edge sil endpoint must exactly one of: (a) src vert, (b) a "myFace" edge, (c) from clipping,
	  // or (d) a silhouette point on the middle of the edge [not checked here]
	  int isAfromSrc =  AsrcVert != NULL ? 1 : 0;
	  int isAfromEdge = Aedge != -1 ? 1 : 0;
	  int isAfromClip = Aclip ? 1 : 0;
	  int isBfromSrc =  BsrcVert != NULL ? 1 : 0;
	  int isBfromEdge = Bedge != -1 ? 1 : 0;
	  int isBfromClip = Bclip ? 1 : 0;
	  
	  assert(isAfromSrc + isAfromEdge + isAfromClip <= 1);
	  assert(isBfromSrc + isBfromEdge + isBfromClip <= 1);
	}

      edges.push_back(new POBoundaryEdge(A, B, meshFace, cone.sourceFace, st, boundary,
				     AsrcVert,BsrcVert,C,cone.srcMeshEdge[e],Aedge,Bedge,Aclip,Bclip));

      
      //	  assert( (A-B).norm() > 0.0001);
      
      //	  for(int i=0;i<3;i++)
      //	    if (intersectsEdgeT[i])
      //	      intersectsEdge[i] = true;
      
      if (Aedge != -1)
	intersectsEdge[Aedge] = true;
      
      if (Bedge != -1)
	intersectsEdge[Bedge] = true;
    }
				       
  // ---------------- Clipping: Intersect the target edge with the plane of the source face ------

  // clipping face shares a vertex but not an edge with the target triangle.
  // I doubt this weird case matters, but it could theoretically happen.
  if (clipping && adjacentVertex != -1) 
    {
      // to compute the intersection between two triangles ABC and PQR that share a vertex (A==P):
      // let X = intersect(BC,PQR); if X exists, then add AX, else:
      // let Y = intersect(QR,ABC); if Y exists, then add PY

      assert(meshFace->GetVertex(adjacentVertex) == cone.getVertex(coneAdjVertex));

      Vec3r A = sourceFace->GetVertex(adjacentVertex)->GetVertex();
      Vec3r B = sourceFace->GetVertex((adjacentVertex+1)%3)->GetVertex();
      Vec3r C = sourceFace->GetVertex((adjacentVertex+2)%3)->GetVertex();
      Vec3r P = cone.P[coneAdjVertex];
      Vec3r Q = cone.P[(coneAdjVertex+1)%3];
      Vec3r R = cone.P[(coneAdjVertex+2)%3];

      // quick reject.  this isn't necessarily faster to run than doing the whole intersection, but it's faster to implement
      if (!sameSide(A,B,C,Q,R) && !sameSide(P,Q,R,B,C))
	{
	  real t,u,v;
	  Vec3r BC = C - B;
	  bool result = GeomUtils::intersectRayTriangle(B,BC,P,Q,R,t,u,v);
	  if (result && t <= 1)
	    {
	      Vec3r X = B + t*BC;
	      
	      if ((A-X).norm() < 0.00001)
		printf("WARNING: infinitesimal clipping PO edge  (AX)\n");
	      //	  assert( (A-B).norm() > 0.00001);
	      else
		edges.push_back(new POBoundaryEdge(A, X, meshFace, cone.sourceFace, 
					       POBoundaryEdge::DEPTH_CLIPPING, 
					       true, NULL,NULL, C, -1, -1,-1,false,false));
	      // putting in bogus values for the last few arguments because they probably don't matter
	    }
	  
	  Vec3r QR = R - Q;
	  result = GeomUtils::intersectRayTriangle(Q,QR,A,B,C,t,u,v);
	  if (result && t <= 1)
	    {
	      Vec3r Y = Q + t*QR;
	      
	      if ((P-Y).norm() < 0.00001)
		printf("WARNING: infinitesimal clipping PO edge  (PY)\n");
	      //	  assert( (A-B).norm() > 0.00001);
	      else
		edges.push_back(new POBoundaryEdge(P, Y, meshFace, cone.sourceFace, 
					       POBoundaryEdge::DEPTH_CLIPPING, 
					       true, NULL,NULL, C, -1, -1,-1,false,false));
	      // putting in bogus values for the last few arguments because they probably don't matter
	    }
	}
    }
  

  if (clipping && adjacentEdge == -1 && adjacentVertex == -1)
    // Can ignore this test for faces that share an edge on the surface.
    {
      bool coplanar;
      Vec3r A,B;
      int faceIndA, faceIndB, edgeIndA, edgeIndB;
      bool result = intersectTriangleTriangle(meshFace, cone.P, coplanar, A, B, faceIndA, edgeIndA, faceIndB, edgeIndB);
      if (coplanar)
	printf("WARNING: COPLANAR TRIANGLES IN PO BOUNDARY CLIPPING\n");

      if (result && !coplanar)
	{
	  assert(edgeIndA >=0 && edgeIndA < 3 && edgeIndB >=0 && edgeIndB < 3);
	  //	  if (edgeIndA == edgeIndB)
	  //	    printf("Warning edgeIndA (%d) == edgeIndB (%d)\n", edgeIndA, edgeIndB);
	  intersection = true;

	  bool Aclip, Bclip, Aedge, Bedge;
	  
	  if (faceIndA == 0)
	    {
	      intersectsEdge[edgeIndA] = true;
	      Aedge = edgeIndA;
	    }
	  else
	    Aclip = true;

	  if (faceIndB == 0)
	    {
	      intersectsEdge[edgeIndB] = true;
	      Bedge = edgeIndB;
	    }
	  else
	    Bclip = true;
	  
	  if ((A-B).norm() < 0.00001)
	    printf("WARNING: infinitesimal clipping PO edge\n");
	  //	  assert( (A-B).norm() > 0.00001);
	  else
	    edges.push_back(new POBoundaryEdge(A, B, meshFace, cone.sourceFace, POBoundaryEdge::DEPTH_CLIPPING, 
					   true, NULL,NULL, C, -1, -1,-1,false,false));
	      // putting in bogus values for the last few arguments because they probably don't matter
	}
    }

  if (debugPrint)
    printf("*** Intersection done: num new edges: %d\n",edges.size() - oldSize);

  if (intersection)
    {
      if (debugPrint)
	printf("Intersection edges found\n");
      return true;
    }


  // none of the cone "faces" intersect the triangle.  
  // either the triangle is entirely inside or entirely outside the cone.
  // test an arbitrary point inside the triangle to resolve
  Vec3r centroid = (meshFace->GetVertex(0)->GetVertex() + 
		    meshFace->GetVertex(1)->GetVertex() + 
		    meshFace->GetVertex(2)->GetVertex())/3;
  
  if (cone.pointInside(centroid, C, clipping))
    {
      intersectsEdge[0] = intersectsEdge[1] = intersectsEdge[2] =  true;
      if (debugPrint)
	printf("Entire face contained (total intersection)\n");
      return true;
    }

  if (debugPrint)
    printf("No intersection found\n");
  return false;
}




real ViewMapBuilder::DistToPOFace(WFace * face, Vec3r testPoint)
// given a test point on a particular face, and, assuming that this point lies on a silhouette point
// find the distance to the furthest punch-out curve/point that is nearer to the camera
// return -1 if there is no corresponding point.
{
  real PODist = -1;


  Vec3r ray_dir = _viewpoint - testPoint;
  ray_dir.normalize();

  // return the distance to the furthest point being punched-out by this silhouette
  for(set<WFace*>::iterator it= _ViewMap->facePOData(face)->targetFaces.begin();
      it != _ViewMap->facePOData(face)->targetFaces.end(); ++it)
    {
      real t, u, v;

      // intersect a ray with the face
      if (GeomUtils::intersectRayTriangle(testPoint, ray_dir, 
					  (*it)->GetVertex(0)->GetVertex(),
					  (*it)->GetVertex(1)->GetVertex(), 
					  (*it)->GetVertex(2)->GetVertex(),
					  t, u,v))
	{
	  assert(t>=0);

	  if (PODist == -1 || t > PODist)
	    PODist = t; 
	}
    }

  return PODist;
}


bool POBoundaryEdge::projectPoint(Vec3r srcPoint, Vec3r viewpoint, Vec3r & projection)
{
  static NodeShape * debugNode = NULL;

  if (debugNode == NULL)
    {
      debugNode = new NodeShape;
      punchOutDebugNode->AddChild(debugNode);
    }


  Vec3r dirA = A - viewpoint;
  Vec3r dirB = B - viewpoint;

  /*
  // compute Aproj and Bproj, as the intersection of a view vector through A/B and the source triangle
  if (!projectionsComputed)
    {
      GeomUtils::intersection_test result;
      real t;

      result = intersectRayPlanePN(viewpoint, dirA, sourceFace->normal(),
				   sourceFace->GetVertex(0)->GetVertex(), t);
      
      assert(result == DO_INTERSECT);

      Aproj = viewpoint + t * dir;


      result = intersectRayPlanePN(viewpoint, dirB, sourceFace->normal(),
				   sourceFace->GetVertex(0)->GetVertex(), t);

      assert(result == DO_INTERSECT);

      Bproj = viewpoint + t * dir;



      projectionsComputed = true;
    }
  */

  // project "srcPoint" onto the planar quadrilateral (A,B,Aproj,Bproj)

  Vec3r normal = dirA ^ dirB;
  normal.normalize();

  real d = (A- srcPoint)*normal;
  
  projection = srcPoint + d * normal;

  
  // --------- check if the the projection lies within the quad ----------------
  

  // clip against source face
  if (!sameSide (sourceFace, projection, A))
    return false;

  // clip against target face
  if (!sameSide (myFace, projection, sourceFace->GetVertex(0)->GetVertex()))
    return false;


  
  Vec3r dirProj = projection - viewpoint;

  Vec3r AP = dirA ^ dirProj;
  Vec3r BP = dirB ^ dirProj;
  Vec3r AB = dirA ^ dirB;


  // clip against "A" edge
  if ( AP * AB < 0)
    return false;

  // clip against "B" edge
  if ( BP * AB > 0)  // note: AB = -BA
    return false;


  real t;
  GeomUtils::intersectRayPlanePN(viewpoint, dirProj, sourceFace->GetNormal(), myFace->GetVertex(0)->GetVertex(), t);
  Vec3r A1 = viewpoint + t * dirProj;
  GeomUtils::intersectRayPlanePN(viewpoint, dirProj, myFace->GetNormal(), myFace->GetVertex(0)->GetVertex(), t);
  Vec3r A2 = viewpoint + t * dirProj;

  LineRep * line =new LineRep(A1, A2);
  line->SetWidth(2);
  line->ComputeBBox();
  debugNode->AddRep(line);
  
  return true;
}




WFace * makeFace(WXShape * poGeom, WXVertex * p1, WXVertex * p2, WXVertex * p3, Vec3r normal, 
		 POBoundaryEdge * bnd)
{
  vector<WVertex*> verts;
  vector<Vec3r> normals;
  vector<Vec2r> texCords;

  verts.push_back(p1);
  verts.push_back(p2);
  verts.push_back(p3);

  //  cout << "Verts: " << p1 << ", " << p2 << ", " << p3 << endl;
  //  cout << "Locs : " << p1->GetVertex() << ", " << p2->GetVertex() << ", " << p3->GetVertex() << endl;

  normals.push_back(normal);
  normals.push_back(normal);
  normals.push_back(normal);

  texCords.push_back(Vec2r(-1,-1));
  texCords.push_back(Vec2r(-1,-1));
  texCords.push_back(Vec2r(-1,-1));

  WFace * f = poGeom->MakeFace(verts,normals,texCords,0);

  assert(dynamic_cast<WXFace*>(f) != NULL);

  WXFace * f2 = (WXFace*)f;

  f2->SetSourcePOB(bnd);

  return f;
  /*
  if (debugRegionToShow == -1 || bnd->POregionIndex == debugRegionToShow)
    {
      TriangleRep * tr = new TriangleRep(p1->GetVertex(),color,
					 p2->GetVertex(),color,
					 p3->GetVertex(),color);
      tr->ComputeBBox();
      tr->SetStyle(TriangleRep::FILL);
      cuspRegionGeom->AddRep(tr);
    }
  */
}


pair<POBoundaryEdge*,bool>
advance(pair<POBoundaryEdge*,bool> edgePair,
	multimap<pair<WFace*,WFace*>,POBoundaryEdge*> &faceMap,
	multimap<pair<WVertex*,WFace*>,POBoundaryEdge*> &vertexMap,
	int region, bool forward)
// given a particular POBoundary edge, find the next edge that connects to it.
//     connect at either B, depending on whether "flip" == "forward."
//     that is, "forward" means advancing from B if the edge is not flipped
{
  POBoundaryEdge * bnd = edgePair.first;
  bool flipped = edgePair.second;

  // what to return if the chain ends
  pair<POBoundaryEdge*,bool> none(NULL,true);
  
  //  // FOR DEBUGGING:
  //return none;

  // gather the source variables from the appropriate edge

  WVertex * srcVert;
  int myFaceEdge;
  bool isDegen, isClip;
  Vec3r startPos; // for debugging

  if (flipped != forward)  // flipped = false and forward = true, then we advance from B
    {
      srcVert = bnd->BsrcVert;
      myFaceEdge = bnd->Bedge;
      isDegen = bnd->isBdegen;
      isClip = bnd->isBclipPoint;
      startPos = bnd->B;
    }
  else
    {
      srcVert = bnd->AsrcVert;
      myFaceEdge = bnd->Aedge;
      isDegen = bnd->isAdegen;
      isClip = bnd->isBclipPoint;
      startPos = bnd->A;
    }

  // don't chain through degeneracies and clip points

  if (isDegen || isClip)
    return none;

  // don't chain through silhouette points
  // couldn't hurt to add another variable that confirms that this is a silhouette point
  if (srcVert == NULL && myFaceEdge == -1)
    return none;

  // the endpoint is either on an edge in myFace or a vertex on sourceFace.  
  // to advance, we cross the corresponding edge
  
  // do we lie on a vertex of sourceFace
  if (srcVert != NULL)
   {
      assert(myFaceEdge == -1);

      POBoundaryEdge * result = NULL;

      int numFound =  0;

      //      vector<POBoundaryEdge*> b2;

      pair<WVertex*,WFace*> query(srcVert, bnd->myFace);
      for(multimap<pair<WVertex*,WFace*>,POBoundaryEdge*>::iterator it = vertexMap.lower_bound(query);
	  it != vertexMap.upper_bound(query); ++it)
	{
	  POBoundaryEdge * bnd2 = (*it).second;

	  assert(bnd2->isRealBoundary);

	  if (bnd2 == bnd || bnd2->POregionIndex != region)
	    continue;

	  result = bnd2;
	  numFound++;
	  //	  b2.push_back(bnd2);
	}
      if (numFound >= 2)
	{
	  printf("WARNING: CHAINING CUSP-REGIONS FOUND A JUNCTION\n");

	  /*	  if (debugRegionToShow == -1 || region == debugRegionToShow)
	    {
	      VertexRep * vert = new VertexRep(startPos.x(), startPos.y(), startPos.z());
	      vert->SetPointSize(30);
	      vert->ComputeBBox();
	      cuspRegionGeom->AddRep(vert);	    

	      VertexRep * vert2 = new VertexRep(srcVert->GetVertex().x(), 
						srcVert->GetVertex().y(), srcVert->GetVertex().z());
	      vert2->SetPointSize(30);
	      vert2->ComputeBBox();
	      cuspRegionGeom->AddRep(vert2);	    
	      
	      for(vector<POBoundaryEdge*>::iterator it = b2.begin(); it != b2.end(); ++it)
		{
		  LineRep * lr = new LineRep((*it)->A, (*it)->B);
		  lr->SetWidth(10);
		  lr->ComputeBBox();
		  cuspRegionGeom->AddRep(lr);	    

		  LineRep * lr2 = new LineRep((*it)->Asrc, (*it)->Bsrc);
		  lr2->SetWidth(20);
		  lr2->ComputeBBox();
		  cuspRegionGeom->AddRep(lr2);	    
		}
	    }
	  */
	  return none;
	}
      
      if (result == NULL)
	return none;

      
      // figure out the flip.
      // if A matches srcVert, then do not flip when going forward
      assert((srcVert == result->AsrcVert) != (srcVert == result->BsrcVert));
      bool flipAB = (srcVert == result->BsrcVert) == forward;

      assert(result != bnd);
      /*
      printf("%s on srcVert (bnd edge vertex. flip = %s)\n",forward?"forward":"backward",
	     flipAB?"true":"false");

      printf("--------- bnd: -----------\n");
      bnd->print();

      printf("--------- result: ---------\n");
      result->print();
      */
      assert(! (result->isAdegen && (flipAB != forward)));
      assert(! (result->isBdegen && (flipAB == forward)));
      assert(! (result->isAclipPoint && (flipAB != forward)));
      assert(! (result->isBclipPoint && (flipAB == forward)));


      // output shouldn't match input
      //      pair<POBoundaryEdge*,bool> r(result,flipAB);
      //      assert(r != edgePair);

      return pair<POBoundaryEdge*,bool>(result,flipAB);
    }

  assert(srcVert == NULL);
  assert(myFaceEdge != -1);

  // we're crossing over a myFace edge; get the corresponding face
  WFace * newMyFace = bnd->myFace->GetBordingFace(myFaceEdge);

  if (newMyFace == NULL) // we're at a mesh boundary
    return none;

  // look up candidate edges that match this pairing
  POBoundaryEdge * nextEdge = NULL;
  bool nextFlipped;

  pair<WFace*,WFace*> query(bnd->sourceFace, newMyFace);

  int numFound = 0;
  for(multimap<pair<WFace*,WFace*>,POBoundaryEdge*>::iterator it = faceMap.lower_bound(query);
      it != faceMap.upper_bound(query); ++it)
    {
      assert( (*it).first.first == bnd->sourceFace );
      assert( (*it).first.second == newMyFace );

      POBoundaryEdge * cand = (*it).second;

      if (cand->POregionIndex != region)
	continue;

      // check if this edge is adjacent

      bool foundA = (cand->Aedge != -1 && cand->myFace->GetBordingFace(cand->Aedge) == bnd->myFace);
      bool foundB = (cand->Bedge != -1 && cand->myFace->GetBordingFace(cand->Bedge) == bnd->myFace);

      assert(!(foundA && foundB));
      
      if (foundA || foundB)
	{
	  numFound++;
	  nextEdge = cand;
	  nextFlipped = (foundB == forward);

	  //	  assert(! (foundA && cand->isAdegen));
	  //	  assert(! (foundB && cand->isBdegen));
	}
    }

  if (numFound == 0)
    return none;

  if (numFound >= 2)
    {
      foundBoundaryCusp = true;
      return none;
    }

  //  assert(numFound < 2);

  assert(nextEdge != bnd);

  bool nextDegen = forward != nextFlipped ? nextEdge->isAdegen : nextEdge->isBdegen;

  assert(!nextDegen);

  //  printf("%s on edge (intersection edge)\n",forward?"forward":"backward");

  // output shouldn't be the same as input
  //  pair<POBoundaryEdge*,bool> r(nextEdge,nextFlipped);
  //  assert(r != edgePair);
  
  return pair<POBoundaryEdge*,bool>(nextEdge,nextFlipped);
}

bool border(WFace *f1, WFace *f2)
{
  //  printf("f1: %08X, f2: %08X\n", f1,f2);
  //  printf("f1 bordingfaces: %08X,%08X,%08X\n", f1->GetBordingFace(0),f1->GetBordingFace(1),f1->GetBordingFace(2));

  return f1->GetBordingFace(0) == f2 || f1->GetBordingFace(1) == f2 || f1->GetBordingFace(2) == f2;
}  

WXShape * 
ViewMapBuilder::GenerateCuspRegionGeometry()
// for each PO surface in a CUSP region --- that is, generated by Mesh Silhouette edges
//   create new polygons for these faces, and add them to the wshape and the grid
//   they will be totally separate geometry; we do not attempt topological surgery here
{
  printf("Generating PO-Cusp geometry\n");

  //  WXEdgeBuilder builder;
  //  builder.setCurrentWShape(poGeom);

  // collect all boundary edges 
  multimap<pair<WFace*,WFace*>,POBoundaryEdge*> faceMap; 
  multimap<pair<WVertex*,WFace*>,POBoundaryEdge*> vertexMap;

  // every boundary edge arises from one "sourceFace" and one "myFace"

  typedef pair<WFace*,WFace*> FFPair;
  typedef pair<WVertex*,WFace*> VFPair;

  for(map<WFace*,FacePOData*>::iterator fit = _ViewMap->facePOData().begin(); fit != 
	_ViewMap->facePOData().end(); ++fit)
    for(vector<POBoundaryEdge*>::iterator bit = (*fit).second->POboundary.begin(); 
	bit != (*fit).second->POboundary.end(); ++bit)
      if ( (*bit)->sourceType == POBoundaryEdge::MESH_SIL)   // keep track of mesh-source edges only
	{
	  if (! (*bit)->isRealBoundary )
	    continue;
	  
	  POBoundaryEdge * bnd = *bit;
	  // note: these are currently never being deallocated, will be a memory leak...

	  faceMap.insert(pair<FFPair,POBoundaryEdge*>(FFPair(bnd->sourceFace,bnd->myFace),bnd));

	  if (bnd->AsrcVert != NULL)
	    vertexMap.insert(pair<VFPair,POBoundaryEdge*>(VFPair(bnd->AsrcVert,bnd->myFace),bnd));
	  if (bnd->BsrcVert != NULL)
	    vertexMap.insert(pair<VFPair,POBoundaryEdge*>(VFPair(bnd->BsrcVert,bnd->myFace),bnd));
	}

  int numEdges = faceMap.size();

  if (numEdges == 0)
    return NULL;

  // Make a new shape
  WXShape * poGeom = new WXShape();

  foundBoundaryCusp = false;
  
  // chain the edges.
  set<POBoundaryEdge*> processed;
  typedef pair<POBoundaryEdge*,bool> EBpair;
  list<EBpair> chain;   // the bool indicates whether (A,B) are flipped
  
  multimap<FFPair,POBoundaryEdge*>::iterator mmit, mmit2;

  for(mmit = faceMap.begin(); mmit != faceMap.end(); ++mmit)
    {
      //      printf("NEW CHAIN -----------------\n");

      bool closed = false;
     
      POBoundaryEdge * startEdge = (*mmit).second;
      if (processed.find(startEdge) != processed.end())
	continue;

      // find the rest of the chain starting at this edge
      chain.clear();
      //      chain.push_back(pair<POBoundaryEdge*,bool>(startEdge,false));
      //      processed.insert(startEdge);
      int region = startEdge->POregionIndex;

      EBpair estart(startEdge, startEdge->isBdegen);

      // bidirectional chaining: first forward, then backward
      for(EBpair e = estart; e.first != NULL; e = advance(e,faceMap,vertexMap,region,true))
	{
	  // can this test ever pass?
	  if (e.first == startEdge && chain.size() > 0)
	    {
	      assert(chain.size() > 1);
	      closed = true;
	      break;
	    }

	  //	  assert(processed.find(e.first) == processed.end());
	  if (processed.find(e.first) != processed.end())
	    break;

	  //	  if (chain.size() > 0)
	  //	    printf("FORWARD!\n");

	  assert(chain.size() == 0 ||
		 !(e.first->isAdegen && !e.second) && !(e.first->isBdegen && e.second));

	  chain.push_back(e);
	  processed.insert(e.first);
	}

      if (!closed)
	for(EBpair e = advance(estart,faceMap,vertexMap,region,false); 
	    e.first != NULL; e = advance(e,faceMap,vertexMap,region,false))
	  {
	  // can this test ever pass?
	    if (processed.find(e.first) != processed.end())
	      break;

	    //	    printf("BACKWARD!\n");

	    assert(processed.find(e.first) == processed.end());
	    chain.push_front(e);
	    processed.insert(e.first);
	  }

      assert(!closed || chain.size() > 1);

      if (closed)
	printf("WARNING: CLOSED PO BOUNDARY CHAIN.  size = %d.  IS THIS POSSIBLE?\n",chain.size());
      //      cout << chain.front().first << ", " << chain.back().first << endl;

      //      assert(!closed); // I don't think a chain can ever be closed.
      
      
      // now we have a chain with labeled flips.
      
      // each po-edge generates a quad. at cusps, the quad degenerates into a triangle
      // generate these quads and connect them topologically
      
      //      int numNonCusps = chain.size() - numCuspFaces;
      //      int numNewWFaces = 2*numNonCusps + numCuspFaces; // each quad generates two faces
      //      int numWVertices = 2+ numNewWFaces; // triangle strip
      
      WXVertex * lastSrcVert = NULL;
      WXVertex * lastMyVert = NULL; 
      
      // note: normals being used here are very approximate, since they shouldn't matter later...

      // ----------------------- generate first face(s) ----------------------------

      //      printf("====================== PROCESSING NEW CHAIN (length: %d) ======================\n", chain.size());

      list<EBpair>::iterator it = chain.begin();  
      
      POBoundaryEdge * bnd = (*it).first;
      bool flipped = (*it).second;
      Vec3r normal = (bnd->A-_viewpoint) ^ (bnd->B-_viewpoint);
      bool isADegen;
      
      Vec3r AF, BF, AsrcF, BsrcF;

      int numVerts = 0;

      assert(normal.norm() > 0);
    
      Vec3r color(drand48(), drand48(), drand48());

      //      cout << "chain length: " << chain.size() << ", color: " << color << endl;
      
      // NOTE: it appears that order matters when creating faces.  No documentation in Freestyle, but
      // my working hypothesis is that each edge that connects two triangles should be created with
      // both "orientations."  Suppose we have two triangles ABC and BCD that share an edge BC/CB.
      // We'll get the right results if we create ABC and CBD, but not ABC and BCD.  Does this make sense?
      // It doesn't seem to be possible to assign a single set of windings to a given mesh (e.g., consider
      // an extraordinary vertex with an odd number of adjacent trianges.)
      // Will work with this assumption and see if we get the right results...

      WFace * lastFace = NULL;




      if (!flipped)
	{
	  AF = bnd->A;
	  BF = bnd->B;
	  AsrcF = bnd->Asrc;
	  BsrcF = bnd->Bsrc;
	  isADegen = bnd->isAdegen;
	  assert(!bnd->isBdegen); // can this ever happen?
	}
      else
	{
	  AF = bnd->B;
	  BF = bnd->A;
	  AsrcF = bnd->Bsrc;
	  BsrcF = bnd->Asrc;
	  isADegen = bnd->isBdegen;
	  assert(!bnd->isAdegen); // can this ever happen?
	}

      //      printf("first edge\n");
      
      if (!isADegen)
	{
	  lastSrcVert = new WXVertex(AsrcF);
	  lastMyVert = new WXVertex(AF);
	  WXVertex * srcVert = new WXVertex(BsrcF);
	  WXVertex * myVert = new WXVertex(BF); 
	  lastSrcVert->SetId(numVerts++);
	  lastMyVert->SetId(numVerts++);
	  myVert->SetId(numVerts++);
	  srcVert->SetId(numVerts++);
	  poGeom->AddVertex(srcVert);
	  poGeom->AddVertex(lastSrcVert);
	  poGeom->AddVertex(lastMyVert);
	  poGeom->AddVertex(myVert);
	  WFace * f1 = makeFace(poGeom, srcVert, lastSrcVert, lastMyVert, normal, bnd);
	  WFace * f2 = makeFace(poGeom, myVert, srcVert, lastMyVert, normal, bnd);
	  _ViewMap->addCuspFace(f1, color);
	  _ViewMap->addCuspFace(f2, color);
	  lastSrcVert = srcVert;
	  lastMyVert = myVert;

	  assert(border(f1,f2));
	  assert(border(f2,f1));

	  lastFace = f2;
	}
      else
	{
	  lastSrcVert = new WXVertex(BsrcF);
	  lastMyVert = new WXVertex(BF);
	  WXVertex * srcVert = new WXVertex(AsrcF);  // same as AF for degenerate points
	  lastSrcVert->SetId(numVerts++);
	  lastMyVert->SetId(numVerts++);
	  srcVert->SetId(numVerts++);
	  poGeom->AddVertex(srcVert);
	  poGeom->AddVertex(lastSrcVert);
	  poGeom->AddVertex(lastMyVert);
	  WFace * f = makeFace(poGeom, srcVert, lastMyVert, lastSrcVert, normal, bnd);
	  _ViewMap->addCuspFace(f, color);

	  lastFace = f;

	  /*	  cout << "Makin' face:\n";
	  if (flipped)
	    printf("flipped\n");
	  else
	    printf("not flipped\n");
	  cout << srcVert << "; " << lastSrcVert << "; " << lastMyVert << endl;
	  cout << AsrcF << "; " << AF << "; " << BsrcF << endl;
	  */
	}

      int j =0;
      
      for(++it; it != chain.end(); ++it, ++j)
	{
	  //	  printf("Interior Edge %d\n", j);

	  bnd = (*it).first;
	  flipped = (*it).second;
	  // it should only be possible for the last face that B (or, flipped, A) is degenerate

	  bool isBdegen;
	  if (!flipped)
	    {
	      AF = bnd->A;
	      BF = bnd->B;
	      AsrcF = bnd->Asrc;
	      BsrcF = bnd->Bsrc;
	      isBdegen = bnd->isBdegen;
	      assert(!bnd->isAdegen); // can this ever happen?
	    }
	  else
	    {
	      AF = bnd->B;
	      BF = bnd->A;
	      AsrcF = bnd->Bsrc;
	      BsrcF = bnd->Asrc;
	      isBdegen = bnd->isAdegen;
	      assert(!bnd->isBdegen); // can this ever happen?
	    }
	  
	  // can't have a degenerate edge in the middle of a chain
	  assert(!bnd->isBdegen || j+1== chain.size());

	  normal = (bnd->A-_viewpoint) ^ (bnd->B-_viewpoint);

	  if (!isBdegen)
	    {
	      WXVertex * srcVert = new WXVertex(BsrcF);
	      WXVertex * myVert = new WXVertex(BF);
	      myVert->SetId(numVerts++);
	      srcVert->SetId(numVerts++);
	      poGeom->AddVertex(srcVert);
	      poGeom->AddVertex(myVert);
	      WFace * f1 = makeFace(poGeom, srcVert, lastSrcVert, lastMyVert, normal, bnd);
	      WFace * f2 = makeFace(poGeom, myVert, srcVert, lastMyVert, normal, bnd);
	      _ViewMap->addCuspFace(f1, color);
	      _ViewMap->addCuspFace(f2, color);
	      lastSrcVert = srcVert;
	      lastMyVert = myVert;

	      assert(border(lastFace,f1));
	      assert(border(f1,lastFace));
	      assert(border(f1,f2));
	      assert(border(f2,f1));
	      lastFace = f2;
	    }
	  else
	    {
	      // if B is degenerate, this must be the last face

	      WXVertex * srcVert = new WXVertex(BsrcF);  // same as AF for degenerate points
	      srcVert->SetId(numVerts++);
	      poGeom->AddVertex(srcVert);
	      WFace * f = makeFace(poGeom, srcVert, lastSrcVert, lastMyVert, normal, bnd);
	      _ViewMap->addCuspFace(f, color);

	      assert(border(lastFace,f));
	      assert(border(f,lastFace));
	      lastFace = f;
	    }
	}
    }

  //  printf("done adding\n");


  // real-check topology

  poGeom->ComputeBBox();
  // compute mean edge size:
  poGeom->ComputeMeanEdgeSize();

  for(vector<WFace*>::iterator it = poGeom->GetFaceList().begin(); it != poGeom->GetFaceList().end(); ++it)
    _ViewMap->setFacePOData(*it,new FacePOData());

  if (foundBoundaryCusp)
    printf("WARNING: PO BOUNDARY CUSPS FOUND.  IS THAT POSSIBLE?\n");

  return poGeom;
} 


void POBoundaryEdge::print()
{
  printf("myFace = %08X, sourceFace = %08X\n", myFace, sourceFace);
  printf("AsrcVert = %08X, BsrcVert = %08X\n", AsrcVert, BsrcVert);
  printf("srcFaceEdge = %d, region = %d\n", srcFaceEdge, POregionIndex);
  printf("Aedge = %d, Bedge = %d\n", Aedge, Bedge);
  printf("isAdegen = %s, isBdegen = %s\n", isAdegen ? "true" : "false", isBdegen ? "true" : "false");
  printf("isAclipPoint = %s, isBclipPoint = %s\n", isAclipPoint ? "true" : "false", isBclipPoint ? "true" : "false");
}


POBoundaryEdge::POBoundaryEdge(Vec3r a, Vec3r b, WFace * myf, WFace * src, SourceType st, bool rb,
			       WVertex * Asv, WVertex * Bsv, Vec3r viewpoint, int e, int Ae, int Be,
			       bool Aclip,bool Bclip) 
{ A = a; B = b; myFace = myf; sourceFace = src; 
    sourceType = st; isRealBoundary = rb;
    //    AisOnSourceTri = AsrcTri; 
    AsrcVert = Asv; BsrcVert = Bsv;
    POregionIndex = -1;
    srcFaceEdge = e;
    Aedge = Ae; Bedge = Be;
    isAclipPoint = Aclip;
    isBclipPoint = Bclip;

    if (st != MESH_SIL)
      return;
    
    // check: each mesh edge sil endpoint must come at most one of: (a) src vert, (b) a "myFace" edge, or (c) from clipping,
    // or (d) a silhouette point on the middle of the edge [not checked here]
    int isAfromSrc =  AsrcVert != NULL ? 1 : 0;
    int isAfromEdge = Aedge != -1 ? 1 : 0;
    int isAfromClip = Aclip ? 1 : 0;
    int isBfromSrc =  BsrcVert != NULL ? 1 : 0;
    int isBfromEdge = Bedge != -1 ? 1 : 0;
    int isBfromClip = Bclip ? 1 : 0;

    assert(isAfromSrc + isAfromEdge + isAfromClip <= 1);
    assert(isBfromSrc + isBfromEdge + isBfromClip <= 1);


    isAdegen = AsrcVert != NULL &&
      (myFace->GetVertex(0) == AsrcVert || 
       myFace->GetVertex(1) == AsrcVert || 
       myFace->GetVertex(2) == AsrcVert);

    isBdegen = BsrcVert != NULL &&
      (myFace->GetVertex(0) == BsrcVert || 
       myFace->GetVertex(1) == BsrcVert || 
       myFace->GetVertex(2) == BsrcVert);    

    assert(! (isAdegen && isBdegen));

    real t;
    GeomUtils::intersection_test res;

    if (AsrcVert != NULL)
      Asrc = AsrcVert->GetVertex();
    else
      {
	Vec3r dir = A - viewpoint;
	res = GeomUtils::intersectRayPlanePN(viewpoint, dir, sourceFace->GetNormal(),
					     sourceFace->GetVertex(0)->GetVertex(),t);
	assert(res == GeomUtils::DO_INTERSECT);
	Asrc = viewpoint + t*dir;
      }

    if (BsrcVert != NULL)
      Bsrc = BsrcVert->GetVertex();
    else
      {
	Vec3r dir = B-viewpoint;
	res = GeomUtils::intersectRayPlanePN(viewpoint, dir, sourceFace->GetNormal(),
					     sourceFace->GetVertex(0)->GetVertex(),t);
	assert(res == GeomUtils::DO_INTERSECT);
	Bsrc = viewpoint + t*dir;
      }

    //could theoretically happen, but more likely indicates a bug
    //    assert(!(A[0] == 0 && A[1] == 0 && A[2] == 0));
    //    assert(!(B[0] == 0 && B[1] == 0 && B[2] == 0));
    //    assert(!(Asrc[0] == 0 && Asrc[1] == 0 && Asrc[2] == 0));
    //    assert(!(Bsrc[0] == 0 && Bsrc[1] == 0 && Bsrc[2] == 0));
  }



struct less_vv : public binary_function<NonTVertex*,NonTVertex*,bool>
{
  Vec3r start;
  less_vv(Vec3r s) : start(s) { };
  bool operator()(NonTVertex* v1, NonTVertex* v2)
  {
    real d1 = (v1->getPoint3D() - start).norm();
    real d2 = (v2->getPoint3D() - start).norm();
    return (d1 > d2);
  }
};


NonTVertex * findMatchingPOEndpoint(ViewMap * ioViewMap, Vec3r pt, WFace * inconsistentFace)
{
  // for a viewvertex ntv that lies on the po boundary mesh edge (where inconsistentFace is on the PO side),
  // find the corresponding viewvertex for a PO_SURFACE_INTERSECTION curve.  It will be one of the
  // endpoints of a PO CURVE, returned in ve, with Aside indicating the A or B end.


  // The viewvertex/fedge must:
  // 1. be in a po_surface_intersection
  // 2. have source-face be the same as the inconsistent face here
  // 3. have the endpoint lie at the same 3D location as the viewvertex above
  
  ViewEdge * ve2 = NULL;

  real bestDist = .1; // maximum distance threshold
  
  bool Aside;
  int faceNum; // used for anything?
  
  for(vector<ViewEdge*>::iterator veit = ioViewMap->viewedges_begin();
      veit != ioViewMap->viewedges_end(); ++veit)
    {
      ViewEdge * ve = *veit;
      if (!(ve->getNature() & Nature::PO_SURFACE_INTERSECTION))
	continue;

      /*
      assert(ve->fedgeA()->getFace1() != NULL);
      assert(ve->fedgeA()->getFace2() != NULL);
      assert(dynamic_cast<WXFace*>(ve->fedgeA()->getFace1()) != NULL);
      assert(dynamic_cast<WXFace*>(ve->fedgeA()->getFace2()) != NULL);
      assert(ve->fedgeB()->getFace1() != NULL);
      assert(ve->fedgeB()->getFace2() != NULL);
      */

      if (((WXFace*)(ve->fedgeA()->getFace1()))->sourcePOB() != NULL &&
	  ((WXFace*)(ve->fedgeA()->getFace1()))->sourcePOB()->sourceFace == inconsistentFace &&
	  ve->A() != NULL &&
	  (ve->A()->getPoint3D() - pt).norm() < bestDist)
	{
	  ve2 = ve;
	  Aside = true;
	  faceNum = 1;
	  bestDist = (ve->A()->getPoint3D() - pt).norm();
	}
      
      if (((WXFace*)(ve->fedgeA()->getFace2()))->sourcePOB() != NULL &&
	  ((WXFace*)(ve->fedgeA()->getFace2()))->sourcePOB()->sourceFace == inconsistentFace &&
	  ve->A() != NULL &&
	  (ve->A()->getPoint3D() - pt).norm() < bestDist)
	{
	  ve2 = ve;
	  Aside = true;
	  faceNum = 2;
	  bestDist = (ve->A()->getPoint3D() - pt).norm();
	}
      
      if (((WXFace*)(ve->fedgeB()->getFace1()))->sourcePOB() != NULL &&
	  ((WXFace*)(ve->fedgeB()->getFace1()))->sourcePOB()->sourceFace == inconsistentFace &&
	  ve->B() != NULL &&
	  (ve->B()->getPoint3D() - pt).norm() < bestDist)
	{
	  ve2 = ve;
	  Aside = false;
	  faceNum = 1;
	  bestDist = (ve->B()->getPoint3D() - pt).norm();
	}
      
      if (((WXFace*)(ve->fedgeB()->getFace2()))->sourcePOB() != NULL &&
	  ((WXFace*)(ve->fedgeB()->getFace2()))->sourcePOB()->sourceFace == inconsistentFace &&
	  ve->B() != NULL &&
	  (ve->B()->getPoint3D() - pt).norm() < bestDist)
	{
	  ve2 = ve;
	  Aside = false;
	  faceNum = 2;
	  bestDist = (ve->B()->getPoint3D() - pt).norm();
	}
    }

  //  assert(ve2 != NULL);

  if (ve2 == NULL)
    return NULL;

  ViewVertex * result = Aside ? ve2->A() : ve2->B();

  assert(dynamic_cast<NonTVertex*>(result) != NULL);

  return (NonTVertex*)result;
}


void ViewMapBuilder::ComputePunchOutIntersections(ViewMap * ioViewMap)
{
  // TODO: Break this up into a bunch of separate functions, one for each loop.

  /*
  NodeShape * poNode1 = new NodeShape();
  visDebugNode->AddChild(poNode1);
  Material intmat1;
  intmat1.SetDiffuse(0,1,1,1);
  intmat1.SetEmission(0,0,0,0);
  intmat1.SetAmbient(0,0,0,0);

  poNode1->SetMaterial(intmat1);


  NodeShape * poNode2 = new NodeShape();
  visDebugNode->AddChild(poNode2);
  Material intmat2;
  intmat2.SetDiffuse(1,.5,0,1);
  intmat2.SetEmission(0,0,0,0);
  intmat2.SetAmbient(0,0,0,0);

  poNode2->SetMaterial(intmat2);

  NodeShape * poNode3 = new NodeShape();
  visDebugNode->AddChild(poNode3);
  Material intmat3;
  intmat3.SetDiffuse(.5,.5,.5,1);
  intmat3.SetEmission(0,0,0,0);
  intmat3.SetAmbient(0,0,0,0);

  poNode3->SetMaterial(intmat3);




  NodeShape * poNode4 = new NodeShape();
  visDebugNode->AddChild(poNode4);
  Material intmat4;
  intmat4.SetDiffuse(1,0,0,1);
  intmat4.SetEmission(0,0,0,0);
  intmat4.SetAmbient(0,0,0,0);

  poNode4->SetMaterial(intmat4);

  */

  printf("Computing punch-out intersection points\n");

  vector<FEdge*> newFedges;
  vector<ViewEdge*> newVedges;

  // ============= COMPUTE INTERSECTIONS BETWEEN SIs AND PO BOUNDARIES INSIDE FACES ============
  
  for(vector<FEdge*>::iterator feit = ioViewMap->FEdges().begin();
      feit != ioViewMap->FEdges().end(); ++feit)
    {
      if ( !((*feit)->getNature() & Nature::SURFACE_INTERSECTION))
	continue;


      // ---------------- collect all intersections for this edge ----------------------

      vector<NonTVertex*> newVVertices;
      
      WFace * faces[2] = { (*feit)->getFace1(), (*feit)->getFace2() };

      for(int i=0;i<2;i++)
	if (faces[i] != NULL)
	  {
	    Vec3r pA = (*feit)->vertexA()->getPoint3D();
	    Vec3r pB = (*feit)->vertexB()->getPoint3D();

	    for(vector<POBoundaryEdge*>::iterator pobit = _ViewMap->facePOData(faces[i])->POboundary.begin(); 
		pobit != _ViewMap->facePOData(faces[i])->POboundary.end(); ++pobit)
	      {
		if ( (*pobit)->sourceType == POBoundaryEdge::SMOOTH_SIL)
		  continue;

		/*
		// a silhouette point can't break its own image.  detect this case.
		// occurs when
		//   (a) the target curve is a silhouette
                //   (b) the source of the PO boundary curve is a silhouette, and
		//   (c) the PO boundary curve has a self-crossing (as defined in PunchOut.h)
		// These conditions, together with the knowledge that each face can only
		//   have one smooth silhouette edge, are sufficient to detect the desired case:
		//   if the curves do intersect, then they intersect at a self-crossing.
		//
		// but can a silhouette ever be broken on the surface by a PO edge?

		if ( ((*feit)->getNature() & Nature::SILHOUETTE) &&
		     (*pobit).isFromSmoothSil && (*pobit).hasSelfCrossing)
		  continue;
		*/


		Vec3r qA = (*pobit)->A;
		Vec3r qB = (*pobit)->B;
		       
		Vec3r inter3d;
		
		// probably not the most robust way to compute this intersection.
		if (GeomUtils::intersect3dLines(pA, pB, qA, qB, inter3d) == GeomUtils::DO_INTERSECT)
		{
		  // clip to the line segment
		  real p_t = GeomUtils::segmentParam(pA, pB, inter3d);
		  real q_t = GeomUtils::segmentParam(qA, qB, inter3d);

		  if (p_t >=0 && p_t <= 1 && q_t >=0 && q_t <= 1)
		    {
		      Vec3r inter2d = SilhouetteGeomEngine::WorldToImage(inter3d);
		      
		      ViewEdge * ve;
		      NonTVertex * vv = ioViewMap->CreateNonTVertex(inter3d, inter2d, (*feit));
		      vv->setNature(vv->getNature() | Nature::PO_BOUNDARY);
		      
		      newVVertices.push_back(vv);
		      
		      ioViewMap->addDebugPoint(DebugPoint::PO_CROSSING_INSIDE_TRI, inter3d, 
					       false, vv->svertex());
		      //		      addDebugPoint(inter3d, poNode1);
		    }
		}
	      }
	  }


      // ---------------- sort the intersections by distance from A -----------------

      sort(newVVertices.begin(), newVVertices.end(), less_vv( (*feit)->vertexA()->getPoint3D() ) );

      // ---------------- split the edge at its intersections -----------------------
      
      ViewShape *shape = (*feit)->viewedge()->viewShape();

      shape->SplitEdge(*feit, newVVertices, newFedges, newVedges); 

      // if this assertion fails, then we need to remove it and transfer the tag to new vertices
      assert(! (*feit)->vertexA()->POProjected() && !(*feit)->vertexB()->POProjected());
    }

  for(vector<FEdge*>::iterator fit = newFedges.begin(); fit != newFedges.end(); ++fit)
    ioViewMap->AddFEdge(*fit);

  for(vector<ViewEdge*>::iterator vit = newVedges.begin(); vit != newVedges.end(); ++vit)
    ioViewMap->AddViewEdge(*vit);



  ioViewMap->checkPointers("ComputePunchOutIntersections 1");

  // ==== compute intersections between Surface Intersection curves and Valid-Inconsistent boundaries on the mesh
  
  for(vector<SVertex*>::iterator sit = ioViewMap->SVertices().begin();
      sit != ioViewMap->SVertices().end(); ++sit)
    {
      SVertex * sv = *sit;

      if (sv->fedges().size() != 2)
	continue;

      FEdge * fe1 = sv->fedges()[0];
      FEdge * fe2 = sv->fedges()[1];

      if ( !(fe1->getNature() & Nature::ALL_INTERSECTION) ||
	   !(fe2->getNature() & Nature::ALL_INTERSECTION))
	continue;

      // surface intersections come from two different meshes, handle each
      for(int f=0;f<2;f++)
	{
	  WFace * face1 = f == 0 ? fe1->getFace1() : fe1->getFace2();
	  WFace * face2 = NULL;

	  assert(face1 != NULL);

	  int e;
	  
	  // fe2 has two faces.  find the one that's connected to face1 on the mesh
	  if (fe2->getFace1() == face1 ||
	      fe2->getFace1() == face1->GetBordingFace(0) ||
	      fe2->getFace1() == face1->GetBordingFace(1) ||
	      fe2->getFace1() == face1->GetBordingFace(2))
	    face2 = fe2->getFace1();
	  else
	    {
	      face2 = fe2->getFace2();

	      assert(face2 == face1 || 
		     face2 == face1->GetBordingFace(0) || face2 == face1->GetBordingFace(1) ||
		     face2 == face1->GetBordingFace(2));
	    }

	  if (face1 == face2)
	    continue;
	  
	  assert(face2 != NULL);

	  // IsPunchOut doesn't handle reliably the particular type of intersection point we're looking at here

	  POType pt1 = IsInconsistentPoint(face1, sv->getPoint3D()) ? INCONSISTENT : UNKNOWN;
	  POType pt2 = IsInconsistentPoint(face2, sv->getPoint3D()) ? INCONSISTENT : UNKNOWN;

	  if (pt1 == UNKNOWN)
	    {
	      if ( pt2 == INCONSISTENT && _ViewMap->facePOData(face1)->hasSourceFace(face2))
		pt1 = PUNCH_OUT;
	      else
		pt1 = VALID;
	    }

	  if (pt2 == UNKNOWN)
	    {
	      if (pt1 == INCONSISTENT &&_ViewMap->facePOData(face2)->hasSourceFace(face1))
		pt2 = PUNCH_OUT;
	      else
		pt2 = VALID;
	    }



	  if ( (pt1 == INCONSISTENT && pt2 == VALID) ||
	       (pt2 == INCONSISTENT && pt1 == VALID) )
	    {
	      // Split the ViewEdge into two at this point, and update all data structures
	      // should be safe to do this within the iteration over SVertices, since we're not adding/deleting any svertices here

	      ViewShape * shape = fe1->viewedge()->viewShape();

	      NonTVertex * ntv = new NonTVertex(sv);
	      ioViewMap->AddViewVertex(ntv);
	      shape->AddVertex(ntv);
	      ntv->setNature( ntv->getNature() | Nature::PO_BOUNDARY );


	      // Split the View Edge (analogue of ViewMap::SplitEdge)
	      ViewEdge * vEdge = fe1->viewedge();
	      ViewEdge * newViewEdge = NULL;

	      shape->SplitEdge(ntv, vEdge, newViewEdge,false);

	      if (newViewEdge != vEdge)
		ioViewMap->AddViewEdge(newViewEdge);

	      ioViewMap->addDebugPoint(DebugPoint::PO_CROSSING_ON_MESH_EDGE,sv->getPoint3D(),false,sv);
	      //	      addDebugPoint(sv->getPoint3D(), poNode2);


	      // There should already be a vertex here that's the endpoint of a PO_SURFACE_INTERSECTION.
	      // Find it.
	      NonTVertex * ntv2 = findMatchingPOEndpoint(ioViewMap, sv->getPoint3D(),
							 pt1 == INCONSISTENT ? face1 : face2);
	      
	      //	      assert(ntv2 != NULL); // not sure it's not possible

	      if (ntv2 != NULL)
		{
		  //printf("MERGING. Nature1: %X, Nature2: %X\n", ntv->getNature(), ntv2->getNature());
		  //		  ntv->setNature(ntv2->getNature());
		  ioViewMap->MergeNonTVertices(ntv,ntv2);
		}

	    }
	  
	}
    }


  // ====== CONNECT THE END OF A CUSP-REGION INTERSECTION TO THE CORRESPONDING SILHOUETTE POINT ======

  vector<NonTVertex*> deletedVertices;
  vector<TVertex*> newVertices;

  map<pair<WFace*,WFace*>,SVertex*> svertexMap; // map from pairs of faces to any SVertex* that lies on the edge between them

  for(vector<SVertex*>::iterator sit = ioViewMap->SVertices().begin();
      sit != ioViewMap->SVertices().end(); ++sit)
    {
      const vector<FEdge*> & fes = (*sit)->fedges();

      if ((fes.size() == 2) && 
	  (fes[0]->getNature() & Nature::SILHOUETTE) &&
	  (fes[1]->getNature() & Nature::SILHOUETTE) &&
	  (fes[0]->getFace1() != fes[1]->getFace1()))
	svertexMap[pair<WFace*,WFace*>(fes[0]->getFace1(),fes[1]->getFace1())] = *sit;
    }
  

  

  //  for(vector<ViewVertex*>::iterator vit1 = ioViewMap->ViewVertices().begin();
  //      vit1 != ioViewMap->ViewVertices().end(); ++vit1)
  //    printf("%08X\n", *vit1);

  vector<ViewVertex*> vvlist2 = ioViewMap->ViewVertices(); // copy the list of vvs

  ioViewMap->checkPointers("ComputePunchOutIntersections 2");

  map<SVertex*, NonTVertex*> toBeMerged;

  for(vector<ViewVertex*>::iterator vit = vvlist2.begin(); vit != vvlist2.end(); ++vit)
    {
      NonTVertex * vv= dynamic_cast<NonTVertex*>(*vit);

      if (vv == NULL)
	continue;

      SVertex * posv = vv->svertex();

      if (posv->fedges().size() != 1)
	continue;
      
      FEdge * fe = posv->fedges()[0];
      
      if ( !(fe->getNature() & Nature::PO_SURFACE_INTERSECTION))
	continue;

      // ----- Find which, if any, of the PO boundaries this is the endpoint of ---------
      real minDist = DBL_MAX; // distance threshold
      //      real minDist = 0.1; // distance threshold
      POBoundaryEdge * poSrc = NULL;
      bool Asrc;
      int srcEdge = -1;

      for(int f =0 ;f<2;f++)
	{
	  WXFace * face = (WXFace*)(f == 1 ? fe->getFace1() : fe->getFace2());
	  if (face != NULL && face->sourcePOB() != NULL)
	    {
	      POBoundaryEdge * po = face->sourcePOB();
	      assert(po->sourceType == POBoundaryEdge::MESH_SIL && po->srcFaceEdge != -1);

	      if (po->AsrcVert == NULL && !po->isAdegen) // po->Aedge == -1 && && !po->isAclipPoint)
		{
		  real dist = GeomUtils::distPointSegment<Vec3r>(posv->getPoint3D(), po->A, po->Asrc);
		  if (dist < minDist)
		    {
		      minDist = dist;
		      poSrc = po;
		      Asrc = true;
		    }
		}
	      if (po->BsrcVert == NULL && !po->isBdegen) // po->Bedge == -1 && && !po->isAclipPoint)
		{
		  real dist = GeomUtils::distPointSegment<Vec3r>(posv->getPoint3D(), po->B, po->Bsrc);
		  if (dist < minDist)
		    {
		      minDist = dist;
		      poSrc = po;
		      Asrc = false;
		    }
		}
	    }
	}

      if (poSrc == NULL)
	{
	  printf("WARNING: CAN'T FIND MATCHING POSRC\n");
	  ioViewMap->addDebugPoint(DebugPoint::ERROR,vv->getPoint3D(),false, vv->svertex());
	  //	  addDebugPoint(vv->getPoint3D(),poNode4);
	  continue;
	}

      // ----- Find the corresponding SVertex to connect to: we are looking for an SVertex
      //       in a silhouette curve, such that the svertex lies on "srcEdge" of "srcFace"
      

      WFace * srcFace = poSrc->sourceFace;
      assert(srcFace != NULL);

      WFace * oppFace = poSrc->sourceFace->GetBordingFace(poSrc->srcFaceEdge);

      if (oppFace == NULL)
	{
	  continue;
	}

      assert(srcFace != oppFace);
      
      SVertex * silsv = NULL;

      if (svertexMap.find(pair<WFace*,WFace*>(oppFace,srcFace)) != svertexMap.end())
	silsv = svertexMap[pair<WFace*,WFace*>(oppFace,srcFace)];
      else
	if (svertexMap.find(pair<WFace*,WFace*>(srcFace,oppFace)) != svertexMap.end())
	  silsv = svertexMap[pair<WFace*,WFace*>(srcFace,oppFace)];

      

      if (silsv == NULL)
	{
	  printf("ERROR: COULDN'T FIND SVERTEX IN PO-NODE CONNECTION\n");
	  ioViewMap->addDebugPoint(DebugPoint::ERROR,vv->getPoint3D(),false,vv->svertex());
	  //	  addDebugPoint(vv->getPoint3D(),poNode4);
	  continue;
	}

      //      assert(silsv->viewvertex() == NULL);

      real silD = (silsv->getPoint3D() - SilhouetteGeomEngine::GetViewpoint()).norm();
      real posvD = (posv->getPoint3D() - SilhouetteGeomEngine::GetViewpoint()).norm();

      // we only merge if the silhouette is behind the intersection point, and the intersection point is in front of any other intersection points.  otherwise, the intersection curve won't be visible anyway.

      if (silD > posvD)
      	{
	  real otherDist = DBL_MAX;
	  
	  if (toBeMerged.find(silsv) != toBeMerged.end())
	    otherDist = ( toBeMerged[silsv]->getPoint3D() - SilhouetteGeomEngine::GetViewpoint()).norm();

	  // add these vertices to the list of pairs to be merged (possibly overwriting previous pair for vv)
	  if (otherDist > posvD)
	    toBeMerged[silsv] = vv;
	}
    }
  

  // ---- Create all needed T-vertices -----

  for(map<SVertex*,NonTVertex*>::iterator it = toBeMerged.begin(); it != toBeMerged.end(); ++it)
    {
      // ----- Create a new TVertex for the junction ----------
	  
      //      printf("Creating T-vertex\n");

      SVertex * silsv = (*it).first;
      NonTVertex * vv = (*it).second;
	  
      Vec3r vv3d = vv->getPoint3D();

      TVertex * newv = _ViewMap->MergeSNonTVertices(silsv,vv);
      
      if (newv == NULL)
	{
	  continue;
	}
      
      ioViewMap->addDebugPoint(DebugPoint::SI_SIL_CONNECTION, silsv->getPoint3D(), vv3d,
			       false, silsv, false, vv->svertex());
      //      ioViewMap->addDebugPoint(DebugPoint::SI_SIL_CONNECTION, silsv->getPoint3D(),false,silsv);
      //      ioViewMap->addDebugPoint(DebugPoint::SI_SIL_CONNECTION, vv3d, false, vv->svertex());
    }

  ioViewMap->checkPointers("ComputePunchOutIntersections 3");

  printf("Done computing punch-out intersection points\n");
}



FacePOData::~FacePOData()
{
  for(vector<POBoundaryEdge*>::iterator it = POboundary.begin(); it!=POboundary.end(); ++it)
    delete *it;
}
