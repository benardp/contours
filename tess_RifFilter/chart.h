#ifndef __CHART_H__
#define __CHART_H__

#include "paramPoint.h"

// A chart is a bijective mapping between R^2 and a local neighborhood on a base mesh.  Different kinds of charts may be defined for different kinds of neighborhoods, and need not be isometric w.r.t. each other

// A vertex-centered chart defines a mapping for all faces in the one-ring of a given vertex.  Different cases depending on triangle vs. quads and extraordinary vs. regular, boundary vs. interior
// An edge-centered chart defines a mapping for a pair of faces shared by an edge

// Coordinates in a single face are called (u,v) coordinates.  Coordinates in the shared R^2 space are called (a,b).


// bijective UV<->AB mapping for a single quad face
template<class T>
class FaceMapQuad
{
private:
    ivec2 _originUV;
    ivec2 _v0UV, _v1UV, _v0AB, _v1AB;
    HbrVertex<T> * _origin;
    HbrFace<T> * _face;

public:
    FaceMapQuad() { _origin = NULL; _face = NULL; } // for STL initalizers
    FaceMapQuad(HbrVertex<T> * origin, HbrFace<T> * face);
    bool ParamToAB(const ParamPoint<T> & pt, real & a, real & b) const;
    ParamPoint<T> ABtoParam(real a, real b) const;
    bool PointInside(const ParamPoint<T> & pt) const;
};

// general vertex-centered chart data-structure
template<class T>
class Chart
{
private:
    HbrVertex<T> * _origin; // vertex at the center of the chart

    std::map<HbrFace<T>*, FaceMapQuad<T> > _faces; // list of adjacent faces and their UV<->AB mappings
    std::map<HbrVertex<T>*, std::pair<real, real> > _verts; // list of vertices in the one-ring and their (A,B) coordinates

public:
    Chart(HbrVertex<T> * origin);
    bool ParamToAB(const ParamPoint<T> & pt, real & a, real & b) const;
    ParamPoint<T> ABtoParam(real a, real b) const;
    bool PointInside(const ParamPoint<T> & pt) const;
};



#include "chartFunctions.h"


#endif
