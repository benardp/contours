#ifndef __MESHFUNCTIONS_H__
#define __MESHFUNCTIONS_H__

#include "paramPoint.h"

#include <list>

template<class T>
HbrVertex<T> * GetOtherVertex(const HbrFace<T> * face, const HbrVertex<T> * v1, const HbrVertex<T> * v2)
{
    for(int i=0;i<3;i++)
        if (face->GetVertex(i) != v1 && face->GetVertex(i) != v2)
            return face->GetVertex(i);

    return NULL;
}

template<class T>
int GetOtherVertexIndex(const HbrFace<T> * face, const HbrVertex<T> * v1, const HbrVertex<T> * v2)
{
    for(int i=0;i<3;i++)
        if (face->GetVertex(i) != v1 && face->GetVertex(i) != v2)
            return i;

    return -1;
}


// not all of these are needed
inline
vec3 GetNormal(const vec3 & p0, const vec3 & p1, const vec3 & p2)
{
    return (p2-p0)^(p1-p0);
}

template<class T>
vec3 GetNormal(const HbrVertex<T> * v0, const HbrVertex<T> * v1, const HbrVertex<T> * v2)
{
    return GetNormal(v0->GetData().pos, v1->GetData().pos, v2->GetData().pos);
}

template<class T>
vec3 GetNormal(const HbrFace<T> * face)
{
    return GetNormal(face->GetVertex(0), face->GetVertex(1), face->GetVertex(2));
}

template<class T>
real GetArea(const HbrVertex<T> * v0, const HbrVertex<T> * v1, const HbrVertex<T> * v2)
{
    vec3 normal = GetNormal<T>(v0,v1,v2);
    return  normal*normal/2.;
}

inline
real GetArea(const vec3 & p0, const vec3 & p1, const vec3 p2)
{
    vec3 normal = GetNormal(p0,p1,p2);
    return  normal*normal/2.;
}

template<class T>
real GetArea(HbrVertex<T> * const * v)
{
    return GetArea<T>(v[0],v[1],v[2]);
}

template<class T>
real GetArea(const HbrFace<T> * face)
{
    return GetArea<T>(face->GetVertex(0), face->GetVertex(1), face->GetVertex(2));
}

inline
int NearestVertex(real u, real v, real & uvert, real & vvert)
{
    assert(u>=0 && u <=1 && v>=0 && v<=1);
    int vind;
    if (u < .5 && v < .5)
    {
        vind = 0;
        uvert = 0;
        vvert = 0;
    }
    else
        if (u >= .5 && v < .5)
        {
            vind = 1;
            uvert = 1;
            vvert = 0;
        }
        else
            if (u >= .5 && v >= .5)
            {
                vind = 2;
                uvert = 1;
                vvert = 1;
            }
            else
            {
                vind = 3;
                uvert = 0;
                vvert = 1;
            }
    return vind;
}



template<class T>
HbrFace<T> * NewFace(HbrMesh<T> * outputMesh, HbrVertex<T> * v1, HbrVertex<T> * v2,HbrVertex<T> * v3)
{
    int vtx[3] = { v1->GetID(), v2->GetID(), v3->GetID() };

    HbrFace<T> * result =  outputMesh->NewFace(3,vtx,0);
    assert(result != NULL);
    return result;
}

template<class T>
HbrFace<T> * NewFace(HbrMesh<T> * outputMesh, HbrVertex<T> * v1, HbrVertex<T> * v2,HbrVertex<T> * v3,HbrVertex<T> * v4)
{
    int vtx[4] = { v1->GetID(), v2->GetID(), v3->GetID(), v4->GetID() };

    HbrFace<T>* result =  outputMesh->NewFace(4,vtx,0);
    assert(result != NULL);
    return result;
}



template<class T>
real GetEdgeT(const HbrHalfedge<T> * edge, const ParamPoint<T> & pt)
// return the "t" value this point corresponds to on the edge, or -1 if it doesn't coincide
{
    if (pt.SourceVertex() != NULL)
    {
        if (edge->GetOrgVertex() == pt.SourceVertex())
            return 0;

        if (edge->GetDestVertex() == pt.SourceVertex())
            return 1;

        return -1;
    }

    if (pt.SourceEdge() != NULL)
    {
        if (pt.SourceEdge() == edge)
            return pt.EdgeT();

        if (pt.SourceEdge() == edge->GetOpposite())
            return 1-pt.EdgeT();

        return -1;
    }

    return -1;
}

template<class T>
void GetVertexUV(int index, real & u, real & v)
{
    switch(index)
    {
    case 0: u = 0; v = 0; break;
    case 1: u = 1; v = 0; break;
    case 2: u = 1; v = 1; break;
    case 3: u = 0; v = 1; break;
    default: assert(0);
    }
}

template<class T>
void GetVertexUV(const HbrFace<T> * face, const HbrVertex<T> * vert, real & u, real &v)
{
    assert(face->GetNumVertices() == 4);

    for(int i=0;i<4;i++)
        if (face->GetVertex(i) == vert)
        {
            GetVertexUV<T>(i,u,v);
            return;
        }

    u = -1;
    v = -1;
}

template<class T>
bool GetFaceUV(const HbrFace<T> * face, const ParamPoint<T> & pt, real & u, real & v)
// get (u,v) coordinates for some point on a face
{
    if (pt.SourceFace() != NULL)
    {
        assert(face == pt.SourceFace());

        u = pt.FaceU();
        v = pt.FaceV();

        if (u >=0 && u <=1 && v>=0 && v<=1)
            return true;
        else
        {
            u = -1; v = -1;
            return false;
        }
    }

    if (pt.SourceVertex() != NULL)
    {
        GetVertexUV<T>(face,pt.SourceVertex(),u,v);
        if (u >=0 && u <=1 && v>=0 && v<=1)
            return true;
        else
        {
            u = -1; v = -1;
            return false;
        }
    }

    real u_org=-1, v_org=-1, u_dest=-1, v_dest=-1;
    GetVertexUV<T>(face,pt.SourceEdge()->GetOrgVertex(),u_org, v_org);
    GetVertexUV<T>(face,pt.SourceEdge()->GetDestVertex(),u_dest, v_dest);

    if (u_org == -1 || v_org == -1)
    {
        u = -1;
        v = -1;
        return false;
    }
    else
    {
        u = u_org + pt.EdgeT()*(u_dest - u_org);
        v = v_org + pt.EdgeT()*(v_dest - v_org);
        return true;
    }
}

template<class T>
bool FaceHasVertex(const HbrFace<T> * face, const HbrVertex<T> * vert,int * index = NULL)
{
    if (index != NULL)
        *index = -1;

    if (face == NULL || vert == NULL)
        return false;

    for(int i=0;i<face->GetNumVertices();i++)
        if (face->GetVertex(i) == vert)
        {
            if (index != NULL)
                *index = i;
            return true;
        }

    return false;
}

template<class T>
bool FaceHasEdge(const HbrFace<T> * face, const HbrHalfedge<T> * edge)
{
    if (face == NULL || edge == NULL)
        return false;

    return edge->GetLeftFace() == face || edge->GetRightFace() == face;
}

template<class T>
HbrFace<T> * GetSharedFace(const ParamPoint<T> & p0, const ParamPoint<T> & p1)
// assuming they share exactly one face
{
    if (p0.SourceVertex() != NULL && p1.SourceVertex() != NULL)
    {
        // the result is only unique when the two points are opposite faces on a quad (or higher-order poly)
        assert(p0.SourceVertex()->GetEdge(p1.SourceVertex()) == NULL);

        std::list<HbrHalfedge<T>*> edges;

        p0.SourceVertex()->GetSurroundingEdges(std::back_inserter(edges));

        for(typename std::list<HbrHalfedge<T>*>::iterator it = edges.begin(); it != edges.end(); ++ it)
        {
            if ( FaceHasVertex<T>( (*it)->GetLeftFace(), p1.SourceVertex()))
                return (*it)->GetLeftFace();

            if ( FaceHasVertex<T>( (*it)->GetRightFace(), p1.SourceVertex()))
                return (*it)->GetRightFace();
        }

        return NULL;
    }

    if (p0.SourceFace() != NULL)
    {
        if (p1.SourceFace() != NULL && p0.SourceFace() == p1.SourceFace())
            return p0.SourceFace();

        if (FaceHasVertex<T>(p0.SourceFace(),p1.SourceVertex()))
            return p0.SourceFace();

        if (FaceHasEdge(p0.SourceFace(),p1.SourceEdge()))
            return p0.SourceFace();

        return NULL;
    }

    if (p1.SourceFace() != NULL)
    {
        if (FaceHasVertex<T>(p1.SourceFace(), p0.SourceVertex()))
            return p1.SourceFace();

        if (FaceHasEdge(p1.SourceFace(), p0.SourceEdge()))
            return p1.SourceFace();

        return NULL;
    }

    if (p0.SourceEdge() != NULL)
    {
        HbrFace<T>  * leftFace = p0.SourceEdge()->GetLeftFace();
        HbrFace<T>  * rightFace = p0.SourceEdge()->GetRightFace();

        if (FaceHasEdge(leftFace, p1.SourceEdge()))
            return leftFace;

        if (FaceHasEdge(rightFace, p1.SourceEdge()))
            return rightFace;

        if (FaceHasVertex<T>(leftFace, p1.SourceVertex()))
            return leftFace;

        if (FaceHasVertex<T>(rightFace, p1.SourceVertex()))
            return rightFace;

        return NULL;
    }

    if (p0.SourceVertex() != NULL && p1.SourceEdge() != NULL)
    {
        HbrFace<T> * leftFace = p1.SourceEdge()->GetLeftFace();
        HbrFace<T> * rightFace = p1.SourceEdge()->GetRightFace();

        if (FaceHasVertex<T>(leftFace, p0.SourceVertex()))
            return leftFace;

        if (FaceHasVertex<T>(rightFace, p0.SourceVertex()))
            return rightFace;

        return NULL;
    }

    return NULL;
}

template<class T>
int GetEdgeIndex(HbrVertex<T> * source, HbrVertex<T> * target)
{
    std::list<HbrHalfedge<T>*> edgeList;
    source->GetSurroundingEdges(std::back_inserter(edgeList));

    int i = 0;

    for(typename std::list<HbrHalfedge<T>*>::iterator it = edgeList.begin(); it != edgeList.end(); ++it, ++i)
        if ( (*it)->GetOrgVertex() == target || (*it)->GetDestVertex() == target)
            return i;

    return -1;
}


template<class T>
HbrFace<T> * GetOppFace(const HbrFace<T> * face, int oppVertex, int & eOpp)
{
    HbrHalfedge<T> * adjEdge = face->GetEdge( (oppVertex+1)%3);
    assert(adjEdge != NULL);  // why doesn't this crash at boundaries?

    // check my assumptions
    assert(adjEdge->GetOrgVertex() != face->GetVertex(oppVertex) &&
            adjEdge->GetDestVertex() != face->GetVertex(oppVertex));

    HbrFace<T> * oppFace = adjEdge->GetLeftFace() == face ? adjEdge->GetRightFace() : adjEdge->GetLeftFace();

    if (oppFace != NULL)
    {
        for(int i=0;i<3;i++)
        {
            HbrHalfedge<T> * edge = oppFace->GetEdge((i+1)%3);
            if (edge->GetLeftFace() == face || edge->GetRightFace() == face)
            {
                eOpp = i;
                return oppFace;
            }
        }
        // assert(0);
        eOpp = -1;
    }
    else
        eOpp = -1;

    return oppFace;
}

template<class T>
HbrFace<T> * GetOppFace(const HbrFace<T> * face, int oppVertex)
{
    HbrHalfedge<T> * edge = face->GetEdge( (oppVertex+1)%3);
    HbrFace<T> * oppFace = edge->GetLeftFace() == face ? edge->GetRightFace() : edge->GetLeftFace();
    return oppFace;
}

template<class T>
void GetOneRing(HbrVertex<T> * vertex, std::set<HbrFace<T>*> & faces)
{
    std::list<HbrHalfedge<T>*> edges;
    vertex->GetSurroundingEdges(std::back_inserter(edges));

    for(typename std::list<HbrHalfedge<T>*>::iterator it = edges.begin(); it != edges.end(); ++it)
    {
        if ( (*it)->GetLeftFace() != NULL)
            faces.insert( (*it)->GetLeftFace() );
        if ( (*it)->GetRightFace() != NULL)
            faces.insert( (*it)->GetRightFace() );
    }
}

template<class T>
void GetOneRingVertices(HbrVertex<T> * vertex, std::set<HbrVertex<T>*> & oneRingVertices)
{
    std::set<HbrFace<T>*> oneRing;
    GetOneRing<T>(vertex,oneRing);
    for(typename std::set<HbrFace<T>*>::iterator fit = oneRing.begin(); fit != oneRing.end(); ++fit)
        for(int i=0;i<3;i++)
            oneRingVertices.insert((*fit)->GetVertex(i));
    oneRingVertices.erase(oneRingVertices.find(vertex));
}


template<class T>
int GetVertexIndex(const HbrFace<T> * face, const HbrVertex<T> * v)
{
    assert(face->GetNumVertices() == 3 || face->GetNumVertices() == 4);

    for(int i=0;i<face->GetNumVertices();i++)
        if (face->GetVertex(i) == v)
            return i;

    return -1;
}

template<class T>
vec3 Laplacian(HbrVertex<T> * v_center, std::set<HbrVertex<T>*> & oneRingVertices)
{
    vec3 laplacian(0,0,0);
    real totalWeight = 0;

    const vec3 centerPos = v_center->GetData().pos;

    for(typename std::set<HbrVertex<T>*>::iterator it=oneRingVertices.begin(); it!=oneRingVertices.end();++it)
    {
        HbrHalfedge<T> * edge = v_center->GetEdge( *it) != NULL ? v_center->GetEdge(*it) : (*it)->GetEdge(v_center);

        HbrFace<T> * faces[2] = { edge->GetLeftFace() , edge->GetRightFace() };

        real weight = 0;
        for(int j=0;j<2;j++)
            if (faces[j] != NULL)
            {
                HbrFace<T> * face = faces[j];
                int i = GetVertexIndex<T>(face,v_center);
                vec3 e0 = (face->GetVertex( (i+1)%3 )->GetData().pos - centerPos );
                vec3 e1 = (face->GetVertex( (i+2)%3 )->GetData().pos - centerPos );

                if (length(e0) < 1e-10 || length(e1) < 1e-10)
                    continue;

                e0.normalize();
                e1.normalize();

                weight += tanl( acosl(e0*e1) /2 );

                assert(weight == weight); // check for nan
            }

        vec3 vec = (*it)->GetData().pos - v_center->GetData().pos;
        if (length(vec) < 1e-10)
            continue;

        weight = weight / length(vec);
        laplacian += weight * vec;
        totalWeight += weight;

        assert(laplacian[0] == laplacian[0]);  // check for nan
    }

    if (totalWeight < 1e-10)
        return vec3(0,0,0);

    laplacian = laplacian / totalWeight;

    assert(laplacian[0] == laplacian[0]);  // check for nan

    return laplacian;
}


template<class T>
vec3 Laplacian(HbrVertex<T> * v_center)
{
    std::set<HbrVertex<T>*> oneRingVertices;
    GetOneRingVertices<T>(v_center, oneRingVertices);
    return Laplacian(v_center, oneRingVertices);
}


template <class T>
vec3 FaceAveragedVertexNormal(HbrVertex<T> * vertex)
{
    // what is the best weighting to use?

    std::set<HbrFace<T>*> faces;

    GetOneRing<T>(vertex, faces);

    assert(faces.size() > 0);

    vec3 normal(0,0,0);

    const vec3 centerPos = vec3(vertex->GetData().GetPos());

    // weights from this paper: https://computing.llnl.gov/vis/images/pdf/max_jgt99.pdf
    // equation 3  (might be inaccurate at boundaries?)
    for(typename std::set<HbrFace<T>*>::iterator it = faces.begin(); it != faces.end(); ++it)
    {
        HbrFace<T> * face = *it;
        int i = GetVertexIndex<T>(face,vertex);
        vec3 v0 = vec3(face->GetVertex( (i+1)%3 )->GetData().GetPos()) - centerPos;
        vec3 v1 = vec3(face->GetVertex( (i+2)%3 )->GetData().GetPos()) - centerPos ;

        if (v0*v0 < 1e-10 || v1*v1 < 1e-10)
            continue;

        normal += (v1 ^ v0) / ((v0 * v0) * (v1 * v1));
    }

    return normal.normalize();
}


#endif
