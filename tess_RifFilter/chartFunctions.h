#include "meshFunctions.h"


// ============================ (A,B) PARAMETERIZATION FUNCTIONS ==============================
// ----- (a,b) parameterization provides a single parameterization for the four quads in the 
// one-ring of a regular vertex

// v0 and v1 are vertices shared by the face and the one-ring
// This procedure computes (U,V) and (A,B) coordinates for the origin and two basis vectors v0 and v1
// (The (A,B) coordinates for the origin are always (0,0))
// The basis vectors are guaranteed to be orthonormal, with all entries \in {-1,0,1}

template<class T>
bool MakeFaceBasis(HbrFace<T> * face, HbrVertex<T> * origin,
                   ivec2 & originUV,
                   ivec2 & v0UV, ivec2 & v1UV,
                   ivec2 & v0AB, ivec2 & v1AB)
{
    const int ABedgeCoord[4][2] = { { 1,0 }, { 0, 1}, {-1, 0}, {0, -1} };

    assert(face->GetNumVertices() == 4);

    int originIndex;
    bool result = FaceHasVertex<T>(face, origin, &originIndex);

    if (!result)
        return false;


    real originU, originV;

    GetVertexUV<T>(face, origin, originU, originV);

    originUV[0] = int(originU);
    originUV[1] = int(originV);


    // find the two other vertices shared by the face and the origin's one-ring

    int v0ind = (originIndex+1)%4;
    int v1ind = (originIndex+3)%4;

    // get their UV and AB coordinates

    real v0U, v0V, v1U, v1V;

    GetVertexUV<T>(v0ind, v0U, v0V);
    GetVertexUV<T>(v1ind, v1U, v1V);

    v0UV[0] = int(v0U - originU);
    v0UV[1] = int(v0V - originV);
    v1UV[0] = int(v1U - originU);
    v1UV[1] = int(v1V - originV);

    int i0 = GetEdgeIndex<T>(origin, face->GetVertex(v0ind));
    int i1 = GetEdgeIndex<T>(origin, face->GetVertex(v1ind));

    assert(i0 != -1 && i1 != -1);

    v0AB[0] = ABedgeCoord[i0][0];
    v0AB[1] = ABedgeCoord[i0][1];
    v1AB[0] = ABedgeCoord[i1][0];
    v1AB[1] = ABedgeCoord[i1][1];

    // double-check orthonormality
    assert(v0UV * v0UV == 1 && v1UV * v1UV == 1 && v1UV * v0UV == 0);
    assert(v0AB * v0AB == 1 && v1AB * v1AB == 1 && v1AB * v0AB == 0);


    return true;
}

template<class T>
FaceMapQuad<T>::FaceMapQuad(HbrVertex<T> * origin, HbrFace<T> * face)
{
    _origin = origin;
    _face = face;
    bool result = MakeFaceBasis<T>(_face, _origin, _originUV, _v0UV, _v1UV, _v0AB, _v1AB);

    assert(result);
}

template<class T>
bool FaceMapQuad<T>::PointInside(const ParamPoint<T> & pt) const
{
    real u,v;
    return GetFaceUV<T>(_face,pt,u,v);
}

template<class T>
bool FaceMapQuad<T>::ParamToAB(const ParamPoint<T> & pt, real & a, real & b) const
{
    real u,v;
    if (!GetFaceUV<T>(_face,pt,u,v))
        return false;

    real e0 = (u - _originUV[0]) * _v0UV[0] + (v - _originUV[1]) * _v0UV[1];
    real e1 = (u - _originUV[0]) * _v1UV[0] + (v - _originUV[1]) * _v1UV[1];

    // convert to AB basis
    a = e0*_v0AB[0] + e1*_v1AB[0];
    b = e0*_v0AB[1] + e1*_v1AB[1];

    assert( a>=-1 && a<=1 && b>=-1 && b<=1);
    return true;
}

template<class T>
ParamPoint<T> FaceMapQuad<T>::ABtoParam(real a, real b) const
{
    real e0 = a * _v0AB[0] + b*_v0AB[1];   // (a,b) * v0
    real e1 = a * _v1AB[0] + b*_v1AB[1];   // (a,b) * v1

    real u = _originUV[0] + e0 * _v0UV[0] + e1 * _v1UV[0];
    real v = _originUV[1] + e0 * _v0UV[1] + e1 * _v1UV[1];

    if (u >=0 && u <=1 && v >= 0 && v <= 1)
        return ParamPoint<T>(_face, u, v);
    else
        return ParamPoint<T>(); // not inside
}


template<class T>
Chart<T>::Chart(HbrVertex<T>* origin)
{
    assert(origin->GetValence() == 4 && !origin->OnBoundary());

    _origin = origin;

    // collect the faces in the one-ring
    std::list<HbrHalfedge<T>*> edges;
    origin->GetSurroundingEdges(std::back_inserter(edges));

    for(typename std::list<HbrHalfedge<T>*>::iterator it = edges.begin(); it != edges.end(); ++it)
    {
        if ( (*it)->GetLeftFace() != NULL && _faces.find( (*it)->GetLeftFace()) == _faces.end())
            _faces[(*it)->GetLeftFace()] = FaceMapQuad<T>(origin, (*it)->GetLeftFace() );

        if ( (*it)->GetRightFace() != NULL && _faces.find( (*it)->GetRightFace()) == _faces.end())
            _faces[(*it)->GetRightFace()] = FaceMapQuad<T>(origin, (*it)->GetRightFace() );
    }

    // Get the AB coordinates for each vertex in the one-ring

    for(typename std::map<HbrFace<T>*, FaceMapQuad<T> >::const_iterator it = _faces.begin(); it != _faces.end(); ++it)
    {
        HbrFace<T> * face = (*it).first;
        for(int i=0;i<face->GetNumVertices();i++)
        {
            HbrVertex<T> * vert = face->GetVertex(i);
            if (_verts.find(vert) == _verts.end())
            {
                real a, b;
                bool result = (*it).second.ParamToAB(ParamPoint<T>(vert), a, b);
                assert(result);
                _verts[vert] = std::pair<real,real>(a,b);
            }
        }
    }
}

template<class T>
bool Chart<T>::ParamToAB(const ParamPoint<T> & pt, real & a, real & b) const
{
    if (pt.SourceVertex() != NULL)
    {
        typename std::map<HbrVertex<T>*,std::pair<real,real> >::const_iterator vit = _verts.find(pt.SourceVertex());
        if (vit == _verts.end())
            return false;
        a = (*vit).second.first;
        b = (*vit).second.second;
        assert(a>=-1 && a<=1 && b>=-1 && b<=1);
        return true;
    }

    if (pt.SourceFace() != NULL)
    {
        typename std::map<HbrFace<T>*, FaceMapQuad<T> >::const_iterator fit = _faces.find(pt.SourceFace());

        if (fit == _faces.end())
            return false;

        bool result = (*fit).second.ParamToAB(pt, a, b);
        assert(result);
        return true;
    }

    HbrHalfedge<T> * edge = pt.SourceEdge();
    assert(edge != NULL);

    if (_verts.find(edge->GetOrgVertex()) == _verts.end() || _verts.find(edge->GetDestVertex()) == _verts.end())
        return false;

    std::pair<real,real> AB0 = (*_verts.find(edge->GetOrgVertex())).second;
    std::pair<real,real> AB1 = (*_verts.find(edge->GetDestVertex())).second;

    real t = pt.EdgeT();
    a = (1-t)*AB0.first  + t*AB1.first;
    b = (1-t)*AB0.second + t*AB1.second;

    assert(a>=-1 && a<=1 && b>=-1 && b<=1);
    return true;
}

template<class T>
ParamPoint<T> Chart<T>::ABtoParam(real a, real b) const
{
    // this case is now basically redundant
    if (a == 0 && b == 0)
        return ParamPoint<T>(_origin);

    // need special treatment to check if we're on an edge?

    for(typename std::map<HbrFace<T>*, FaceMapQuad<T> >::const_iterator it = _faces.begin(); it != _faces.end(); ++it)
    {
        ParamPoint<T> pt = (*it).second.ABtoParam(a,b);

        if (!pt.IsNull())
            return pt;
    }

    return ParamPoint<T>();
}

template<class T>
bool Chart<T>::PointInside(const ParamPoint<T> & pt) const
{
    for(typename std::map<HbrFace<T>*, FaceMapQuad<T> >::iterator it = _faces.begin(); it != _faces.end(); ++it)
        if ((*it).second.PointInside(pt))
            return true;

    return true;
}
