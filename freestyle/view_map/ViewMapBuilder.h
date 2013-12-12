//
//  Filename         : ViewMapBuilder.h
//  Author(s)        : Stephane Grabli
//  Purpose          : Class to build silhouette edges from a
//                     Winged-Edge structure
//  Date of creation : 25/03/2002
//
///////////////////////////////////////////////////////////////////////////////


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

#ifndef  VIEWMAPBUILDER_H
# define VIEWMAPBUILDER_H

# include <vector>
# include <deque>
# include "../system/FreestyleConfig.h"
# include "../geometry/Geom.h"
# include "../scene_graph/NodeGroup.h"
# include "../winged_edge/WXEdge.h"
# include "Silhouette.h"
# include "../geometry/GeomUtils.h"
# include "../geometry/Grid.h"
# include "../system/ProgressBar.h"
# include "../geometry/SweepLine.h"
# include "ViewMap.h"
# include "SilhouetteGeomEngine.h"
# include "../scene_graph/TriangleRep.h"
# include "../winged_edge/WEdge.h"
# include "ViewEdgeXBuilder.h"
# include "grid2d.h"
# include "PunchOut.h"
#include "../geometry/FastGrid.h"

using namespace Geometry;


typedef enum { VALID, INCONSISTENT, PUNCH_OUT, UNKNOWN } POType;


class LIB_VIEW_MAP_EXPORT ViewMapBuilder
{
private:

    ViewMap * _ViewMap; // result
    //SilhouetteGeomEngine _GeomEngine;
    ProgressBar *_pProgressBar;
    Vec3r _viewpoint;
    Grid* _Grid;
    ViewEdgeXBuilder *_pViewEdgeBuilder;
    bool _EnableQI;
    real _epsilon;

    // tmp values:
    int _currentId;
    int _currentFId;
    int _currentSVertexId;

    bool _useConsistency;
    real _cuspTrimThreshold;
    real _graftThreshold;

public:

    typedef enum {
        sweep_line
    } intersection_algo;

    typedef enum {
        ray_casting,
        ray_casting_fast,
        ray_casting_very_fast,
        region_based,
        punch_out
    } visibility_algo;

    inline ViewMapBuilder()
    {
        _pProgressBar = 0;
        _Grid = 0;
        _currentId = 1;
        _currentFId = 0;
        _currentSVertexId = 0;
        _pViewEdgeBuilder = new ViewEdgeXBuilder;
        _EnableQI = true;
        _useConsistency = false;
        _cuspTrimThreshold = 0;
    }

    inline ~ViewMapBuilder()
    {
        if(_pViewEdgeBuilder){
            delete _pViewEdgeBuilder;
            _pViewEdgeBuilder = 0;
        }
    }

    /*! Compute Shapes from a WingedEdge containing a list of WShapes */
    void computeInitialViewEdges(WingedEdge&);

    void computeSelfIntersections(WingedEdge&);

    /*! Compute Cusps */
    void computeCusps(ViewMap *ioViewMap);
    /*! Detects cusps (for a single ViewEdge) among SVertices and builds a ViewVertex on top of
   *  each cusp SVertex
   *  We use a hysteresis approach to avoid noise.
   */
    void DetectCusps(ViewEdge *ioEdge);


    /*! Sets the current viewpoint */
    inline void SetViewpoint(const Vec3r& ivp) {_viewpoint = ivp; SilhouetteGeomEngine::SetViewpoint(ivp);}

    /*! Sets the current transformation
   *    iModelViewMatrix
   *      The 4x4 model view matrix, in column major order (openGL like).
   *    iProjection matrix
   *      The 4x4 projection matrix, in column major order (openGL like).
   *    iViewport
   *      The viewport. 4 real array: origin.x, origin.y, width, length
   */
    inline void SetTransform(const real iModelViewMatrix[4][4],
    const real iProjectionMatrix[4][4],
    const int iViewport[4],
    real iFocalLength,
    real iAspect,
    real iFovy) {
        SilhouetteGeomEngine::SetTransform(iModelViewMatrix, iProjectionMatrix, iViewport, iFocalLength);
    }

    inline void SetFrustum(real iZnear, real iZfar) {
        SilhouetteGeomEngine::SetFrustum(iZnear, iZfar);
    }

    /*! Builds the scene view map
   *  returns the list the view map
   *  it is up to the caller to delete this ViewMap
   *    iWRoot
   *      The root group node containing the WEdge structured scene
   */

    ViewMap* BuildViewMap(WingedEdge& we, visibility_algo iAlgo = ray_casting, real epsilon=1e-06) ;
    /*! computes the intersection between all 2D
   *  feature edges of the scene.
   *    ioViewMap
   *      The view map. It is modified by the method.
   *      The list of all features edges of the scene.
   *      Each time an intersection is found, the 2 intersecting
   *      edges are splitted (creating 2 new vertices)
   *      At the end, this list is updated with the adding
   *      of all new created edges (resulting from splitting).
   *    iAlgo
   *      The algo to use for computing the intersections
   */
    void ComputeCurveIntersections(ViewMap *ioViewMap, visibility_algo iAlgo, real epsilon=1e-06);

    /*! Computes the 2D scene silhouette edges visibility
   *    iGrid
   *      For the Ray Casting algorithm.
   */
    void ComputeEdgesVisibility(ViewMap *ioViewMap, WingedEdge & we,
                                visibility_algo iAlgo= ray_casting, Grid* iGrid = 0, real epsilon=1e-6);

    void PropagateVisibilty(ViewMap *ioViewMap);


    void SetGrid(Grid *iGrid) {_Grid = iGrid;}

    /*! accessors */

    /*! Modifiers */
    inline void SetProgressBar(ProgressBar *iProgressBar) {_pProgressBar = iProgressBar;}
    inline void SetEnableQI(bool iBool) {_EnableQI = iBool;}
    void SetUseConsistency(bool uc) { _useConsistency = uc; }

    static void ResetGroupingData(WingedEdge& we);

    void SetCuspTrimThreshold(real threshold) { _cuspTrimThreshold = threshold; }
    void SetGraftThreshold(real threshold) { _graftThreshold = threshold; }

    bool HideSmallBits(ViewMap * ioViewMap);
    bool HideDeadEnds(ViewMap * vm);
    bool HideSmallLoopsAndDeadEnds(ViewMap * vm);
    void GraftDeadEnds(ViewMap * vm);

protected:

    /*! Computes the 2D scene silhouette edges visibility
   *  using a ray casting. On each edge, a ray is cast
   *  to check its quantitative invisibility. The list
   *  of occluders are each time stored in the tested edge.
   *    ioViewMap
   *      The view map.
   *      The 2D scene silhouette edges as FEdges.
   *      These edges have already been splitted at their intersections points.
   *      Thus, these edges do not intersect anymore.
   *      The visibility corresponding to each edge of ioScene is set is this
   *      edge.
   */
    void ComputeRayCastingVisibility(ViewMap *ioViewMap, Grid *iGrid, real epsilon, visibility_algo iAlgo);
    void ComputeFastRayCastingVisibility(ViewMap *ioViewMap, Grid *iGrid, real epsilon=1e-6);
    void ComputeVeryFastRayCastingVisibility(ViewMap *ioViewMap, Grid *iGrid, real epsilon=1e-6);

    /*! Compute the visibility for the FEdge fe.
   *  The occluders are added to fe occluders list.
   *    fe
   *      The FEdge
   *    iGrid
   *      The grid used to compute the ray casting visibility
   *    epsilon
   *      The epsilon used for computation
   *    oShapeId
   *      fe is the border (in 2D) between 2 2D spaces.
   *      if fe is a silhouette,
   *      One of these 2D spaces is occupied by the shape
   *      to which fe belongs (on its left) and the other one is either occupied
   *      by another shape or empty or occupied by the same shape.
   *      We use this ray csating operation to determine which shape
   *      lies on fe's right.
   *      The result is the shape id stored in oShapeId
   */
    int ComputeRayCastingVisibility(ViewMap *ioViewMap, FEdge *fe, Grid* iGrid, real epsilon, set<ViewShape*>& oOccluders,
                                    Polygon3r** oaPolygon, unsigned timestamp);
    int ComputeRayCastingVisibilityPunchOut(FEdge *fe, Grid* iGrid, real epsilon, set<ViewShape*>& oOccluders,
                                            Polygon3r** oaPolygon, unsigned timestamp);

    //  int ComputeRayCastingVisibilityPunchOut(FEdge *fe, Grid* iGrid, real epsilon, set<ViewShape*>& oOccluders,
    //					  Polygon3r** oaPolygon, unsigned timestamp);

    // FIXME
    void FindOccludee(FEdge *fe, Grid* iGrid, real epsilon, Polygon3r** oaPolygon, unsigned timestamp);
    void FindOccludee(FEdge *fe, Grid* iGrid, real epsilon, Polygon3r** oaPolygon, unsigned timestamp,
                      Vec3r& u, Vec3r& A, Vec3r& origin, Vec3r& edge, vector<WVertex*>& faceVertices);


    // our region-based visibility algorithm
    void ComputeRegionBasedVisibility(ViewMap * ioViewMap, WingedEdge& we, Grid * iGrid, real epsilon);

    // helpers for the new visibility algorithm
    Grid2D * Make2DGrid(ViewMap * ioViewMap, WingedEdge & we);
    void BreakEdges2D(Grid2D * grid, WingedEdge& we);
    void BuildRegions(WingedEdge& we,ViewMap * ioViewMap);
    void ComputePerRegionVisibility(WingedEdge & we, Grid * iGrid, real epsilon,
                                    Grid2D * grid, multimap<WFace*, FEdge*> & edgemap);
    void ComputeChainVisibility(ViewMap * ioViewMap, WingedEdge & we, Grid2D * grid,
                                multimap<WFace*, FEdge*> & edgemap);
    WXVertex * getAssociatedMeshVertex(FEdge * fe,Grid2D * grid,multimap<WFace*, FEdge*> & edgemap);


    // our punch-out visibility algorithm
    void ComputePunchOutVisibility(ViewMap * ioViewMap, WingedEdge & we, Grid * iGrid, real epsilon);
    bool FloodFillInconsistent(WFace * startFace,
                               vector<InconsistentTri> & inconsistentCones,
                               std::deque<WFace*> & punchOutFaces,
                               NodeShape * debugPunchOutNodes, int regionIndex);
    void FloodFillPunchOut(std::deque<WFace*> & punchOutFaces,
                           vector<InconsistentTri> & inconsistentCones,
                           int POindex, NodeShape *debug);
    void ComputePunchOutRegions(WingedEdge & we, Grid * iGrid);
    void ComputePunchOutIntersections(ViewMap *ioViewMap);
    bool IsInconsistentPoint(WFace * face, Vec3r point, Vec3r projectionDirection = Vec3r(0,0,0));
    bool IsPunchOutPoint(WFace * face, Vec3r point, set<int> * regionIndices = NULL);
    bool IsPunchOutPointRay(WFace * face, Vec3r rayEnd, set<int> * regionIndices = NULL);
    real DistToPOFace(WFace * face, Vec3r point);
    POType pointType(WFace * face, Vec3r point)
    {
        if (IsInconsistentPoint(face,point)) return INCONSISTENT;
        if (IsPunchOutPoint(face,point)) return PUNCH_OUT;
        return VALID;
    }
    WXShape* GenerateCuspRegionGeometry();

    void ComputeIntersectionPOProjections(ViewMap * ioViewMap);
    void ComputeCuspRegionIntersections(ViewMap * ioViewMap);
};



#endif // VIEWMAPBUILDER_H
