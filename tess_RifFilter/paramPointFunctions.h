#ifndef __PARAMPOINTFUNCTIONS_H__
#define __PARAMPOINTFUNCTIONS_H__

#include <float.h>

#include "paramPoint.h"
#include "meshFunctions.h"
#include "chart.h"

// ================== PARAMETERIZATION HELPER FUNCTIONS ============================

template<class T>
ParamPoint<T>::ParamPoint(HbrVertex<T>  * v, real offset)
{
    clear();
    _sourceVertex = v;
    _normalOffset = offset;
}

template<class T>
ParamPoint<T>::ParamPoint(HbrHalfedge<T>  * e, real tin,real offset)
{
    clear();

    if (tin == 0)
        _sourceVertex = e->GetOrgVertex();
    else
        if (tin == 1)
            _sourceVertex = e->GetDestVertex();
        else
        {
            _sourceEdge = e;
            _t = tin;
        }

    _normalOffset = offset;
}


template<class T>
ParamPoint<T>::ParamPoint(HbrFace<T>  * f, real uin, real vin,real offset)
{
    clear();

    _normalOffset = offset;

    if ( (uin == 0 || uin == 1) && (vin == 0 || vin == 1))
    {
        real uvert,vvert;
        int vind = NearestVertex(uin,vin,uvert,vvert);

        assert(vind >= 0 && vind <= 3);

        _sourceVertex = f->GetVertex(vind);

        return;
    }

    if (uin == 0)
    {
        _sourceEdge = f->GetEdge(3);
        _t = 1-vin;
        return;
    }

    if (uin == 1)
    {
        _sourceEdge = f->GetEdge(1);
        _t = vin;
        return;
    }

    if (vin == 0)
    {
        _sourceEdge = f->GetEdge(0);
        _t = uin;
        return;
    }

    if (vin == 1)
    {
        _sourceEdge = f->GetEdge(2);
        _t = 1-uin;
        return;
    }

    _sourceFace = f;
    _u = uin;
    _v = vin;
}


template<class T>
void ParamPoint<T>::PrintString(char * str) const
{
    if (_sourceVertex != NULL)
    {
        sprintf(str,"Source: vertex (%p).  Boundary: %s. Valence: %d. NormalOffset = %f",
                _sourceVertex, _sourceVertex->OnBoundary() ? "true" : "false",
                _sourceVertex->GetValence(), double(_normalOffset));
    }
    else
    {
        if (_sourceEdge != NULL)
            sprintf(str,"Source: edge (%p; opp:%p). t = %f. NormalOffset = %f",
                    _sourceEdge, _sourceEdge->GetOpposite(),double(_t),double(_normalOffset));
        else
            sprintf(str,"Source: face (%p). (u,v) = (%.20f,%.20f). NormalOffset = %f",
                    _sourceFace, double(_u),double(_v),double(_normalOffset));
    }
}


template<class T>
void ParamPoint<T>::Print() const
{
    char str[200];
    PrintString(str);
    printf("%s",str);
    printf("\n");
}


template<class T>
Chart<T> * ParamPoint<T>::FindChart(const std::vector<ParamPoint<T> > & pts, std::vector<vec2> & abs)
{
    // collect all adjacent vertices
    std::set<HbrVertex<T>*> verts1;

    // probably would be sufficient just to find the neighborhood for a single vertex.

    for(typename std::vector<ParamPoint<T> >::const_iterator pit = pts.begin(); pit != pts.end(); ++pit)
    {
        ParamPoint<T>  pt = *pit;

        assert(!pt.IsNull());

        if (pt.SourceVertex() != NULL)
            verts1.insert(pt.SourceVertex() );
        else
            if (pt.SourceEdge() != NULL)
            {
                verts1.insert(pt.SourceEdge()->GetOrgVertex());
                verts1.insert(pt.SourceEdge()->GetDestVertex());
            }
            else
            {
                for(int i=0;i<4;i++)
                    verts1.insert(pt.SourceFace()->GetVertex(i));
            }
    }

    std::set<HbrFace<T> *> faces;  // all faces to draw candidate vertices from

    for(typename std::set<HbrVertex<T>*>::iterator vit = verts1.begin(); vit != verts1.end(); ++vit)
    {
        // insert the one-ring
        std::list<HbrHalfedge<T> *> edges;
        (*vit)->GetSurroundingEdges(std::back_inserter(edges));
        for(typename std::list<HbrHalfedge<T> *>::iterator it = edges.begin(); it != edges.end(); ++it)
        {
            if ( (*it)->GetLeftFace() != NULL)
                faces.insert( (*it)->GetLeftFace());
            if ( (*it)->GetRightFace() != NULL)
                faces.insert( (*it)->GetRightFace());
        }
    }


    // collect all vertices from all neighboring faces
    std::set<HbrVertex<T>*> verts2;
    for(typename std::set<HbrFace<T> *>::iterator it=faces.begin(); it!=faces.end();++it)
    {
        assert( (*it)->GetNumVertices() == 4);
        for(int i=0;i<4;i++)
        {
            HbrVertex<T> * v = (*it)->GetVertex(i);
            if (v->GetValence() == 4 && !v->OnBoundary())
                verts2.insert(v);
        }
    }

    // try all vertices as possible origins

    //  HbrVertex<T> * origin = NULL;

    Chart<T> * chart = NULL;

    for(typename std::set<HbrVertex<T>*>::iterator it=verts2.begin();it != verts2.end();++it)
    {
        if ( (*it)->GetValence() != 4 || (*it)->OnBoundary())
            continue;

        chart = new Chart<T>(*it);
        abs.clear();

        for(typename std::vector<ParamPoint<T> >::const_iterator pit = pts.begin();pit != pts.end();++pit)
        {
            real a,b;
            bool result = chart->ParamToAB(*pit, a, b);
            if (!result)
                break;
            abs.push_back(vec2(a,b));
        }
        if (abs.size() == pts.size())
            return chart;

        delete chart;
    }

    abs.clear();
    return NULL;
}


template<class T>
Chart<T> * ParamPoint<T>::FindChart(const ParamPoint<T>  & p0, const ParamPoint<T>  & p1, real & a0, real & b0, real & a1, real & b1)
{
    assert(!p0.IsNull() && !p1.IsNull());

    std::vector<ParamPoint<T> > pts;
    pts.push_back(p0);
    pts.push_back(p1);


    std::vector<vec2> abs;

    Chart<T> * chart = FindChart(pts, abs);

    if (chart == NULL)
        return NULL;

    assert(abs.size() == 2);

    a0 = abs[0][0];
    b0 = abs[0][1];
    a1 = abs[1][0];
    b1 = abs[1][1];

    return chart;
}


template<class T>
bool ParamPoint<T>::HasCommonChart(const ParamPoint<T>  & p1, const ParamPoint<T>  & p2)
{
    real a0,b0,a1,b1;

    // this could be made a lot more efficient
    Chart<T> * chart = FindChart(p1,p2,a0,b0,a1,b1);

    if (chart == NULL)
        return false;

    delete chart;
    return true;
}


// is the quadrilateral [p0,p1,p2,p3] guaranteeably convex in AB space?
template<class T>
bool ParamPoint<T>::ConvexInChart(const ParamPoint<T> & p0, const ParamPoint<T> & p1,const ParamPoint<T> & p2, const ParamPoint<T> & p3)
{
    std::vector<ParamPoint<T> > ps;
    ps.push_back(p0);
    ps.push_back(p1);
    ps.push_back(p2);
    ps.push_back(p3);

    std::vector<vec2> abs;

    Chart<T> * chart = FindChart(ps,abs);

    if (chart == NULL){ // can't tell
        //printf("\nNULL CHART\n");
        return false;
    }

    // don't current have vec2's or Shewchuck's orient linked, easier to do this for now.

    assert(abs.size() == 4);

    vec3 cs[4];
    for(int i=0;i<4;i++)
    {
        cs[i][0] = abs[i][0];
        cs[i][1] = abs[i][1];
        cs[i][2] = 0;
        //      printf("%d: (a,b) = (%f,%f)\n",i, double(cs[i][0]),double(cs[i][1]));
    }

    // p0p2 on same side of p1p3?

    real eps = 1e-10;

    real sign1 = ((cs[0] - cs[1]) ^ (cs[3] - cs[1]))[2];
    real sign2 = ((cs[2] - cs[1]) ^ (cs[3] - cs[1]))[2];

    //  printf("sign1,sign2 = %f,%f\n", double(sign1),double(sign2));

    // too close to zero to be safe
    if ( (sign1 < eps && sign1 > -eps) || (sign2 < eps && sign2 > -eps))
        return false;

    if ( (sign1 > 0) == (sign2 > 0))
        return false;


    sign1 = ((cs[1] - cs[0]) ^ (cs[2] - cs[0]))[2];
    sign2 = ((cs[3] - cs[0]) ^ (cs[2] - cs[0]))[2];

    //  printf("sign1,sign2 = %f,%f\n", double(sign1),double(sign2));

    // too close to zero to be safe
    if ( (sign1 < eps && sign1 > -eps) || (sign2 < eps && sign2 > -eps))
        return false;

    if ( (sign1 > 0) == (sign2 > 0))
        return false;



    return true;
}



// =================================== MAIN INTERPOLATION FUNCTIONS ==================================

template<class T>
ParamPoint<T>
ParamPoint<T>::Interpolate(const ParamPoint<T>  & p0, const ParamPoint<T>  & p1, real t)
// given t\in[0..1], find the source point that (1-t)p0 + tp1, assuming that p0 and p1 lie on the same face,
// the points are distinct, and 0 <= t <= 1.
// if the resulting point is invalid for the desired refinement level, then return false and give the
// "nearest" evaluable point instead.
{
    if (t == 0)
        return p0;

    if (t == 1)
        return p1;

    assert(t > 0 && t < 1);
    //  assert(p0 != p1);

    real newOffset = (1-t)*p0._normalOffset + t * p1._normalOffset;

    // ------------- do the two points share an edge? -------------------

    if (p0._sourceFace == NULL && p1._sourceFace == NULL)
    {
        HbrHalfedge<T> * edge = NULL;

        if (p0._sourceVertex != NULL && p1.SourceVertex() != NULL)
        {
            edge = p0._sourceVertex->GetEdge(p1._sourceVertex);
            if (edge == NULL)
                edge = p1._sourceVertex->GetEdge(p0._sourceVertex);
        }
        else
            edge = p0._sourceEdge != NULL ? p0._sourceEdge : p1._sourceEdge;

        if (edge != NULL)
        {
            real t0 = GetEdgeT(edge, p0);
            real t1 = GetEdgeT(edge, p1);

            if (t0 != -1 && t1 != -1)
                return ParamPoint<T> (edge, (1-t)*t0 + t*t1, newOffset);
        }
    }

    // ------------ Interpolate within the face of the quad -------

    HbrFace<T> * face = GetSharedFace(p0,p1);

    if (face != NULL)
    {
        // determine (u,v)s for the two points
        real u0,v0, u1, v1;

        GetFaceUV<T>(face, p0, u0, v0);
        GetFaceUV<T>(face, p1, u1, v1);

        assert(u0 != -1 && u1 != -1);

        return ParamPoint<T>(face, (1-t)*u0 + t*u1, (1-t)*v0 + t*v1, newOffset);
    }

    // ------------ They're not on the same face, due to shifting vertices. Use (a,b) parameterization -------

    real a0,b0,a1,b1;

    //  HbrVertex<T> * origin = FindABOrigin(p0,p1,a0,b0,a1,b1);
    Chart<T> * chart = FindChart(p0,p1,a0,b0,a1,b1);

    if( chart == NULL)
        return ParamPoint<T>();
    //  assert( origin != NULL);

    real a = (1-t)*a0+t*a1;
    real b = (1-t)*b0+t*b1;

    assert(a >= -1 && a <=1 && b>= -1 && b<=1);

    ParamPoint<T> result = chart->ABtoParam(a,b);

    result._normalOffset = newOffset;

    delete chart;

    return result;
}

template<class T>
ParamPoint<T>
ParamPoint<T>::MakeExtraordinarySeparator(const ParamPoint<T> & extPoint, const ParamPoint<T>  & p1)
// given an edge between an extraordinary vertex and some other point on an adjacent face,
// generate a point between them that is just outside the "invalid" region
{
    assert(extPoint._sourceVertex != NULL);

    const real t_threshold = pow(.5,REF_LEVEL) + EXTRAORDINARY_REGION_OFFSET+1e-10; // add a little extra to guarantee that the separator's evaluable

    ParamPoint<T> result;

    real newOffset = (1-t_threshold) * extPoint._normalOffset + t_threshold * p1._normalOffset;

    // ------------- do the two points share an edge? -------------------

    if (p1._sourceFace == NULL)
    {
        HbrHalfedge<T> * edge = NULL;
        if (p1._sourceVertex != NULL)
        {
            edge = extPoint._sourceVertex->GetEdge(p1._sourceVertex);
            if (edge == NULL)
                edge = p1._sourceVertex->GetEdge(extPoint._sourceVertex);
        }
        else
            edge = p1._sourceEdge;

        if (edge != NULL)
        {
            real t0 = GetEdgeT(edge, extPoint);
            real t1 = GetEdgeT(edge, p1);

            //	  printf("t0,t1: %f %f\n", double(t0), double(t1));

            if (t0 != -1 && t1 != -1)
            {
                assert(t0 == 0 || t0 == 1);
                result._sourceEdge = edge;
                result._t = (t0 == 0 ? t_threshold : 1-t_threshold);
                result._normalOffset = newOffset;

                assert(result._t > 0 && result._t < 1);
                assert( (t0 == 0 && result._t < t1) || (t0 == 1 && result._t > t1));

                return result;
            }
        }

    }

    // ------------ Interpolate within the face of the triangle, if we share a face -------

    HbrFace<T> * face = GetSharedFace<T>(extPoint, p1);

    if (face != NULL)
    {
        // determine (u,v)s for the two points
        real u0,v0, u1, v1;

        GetFaceUV(face, extPoint, u0, v0);
        GetFaceUV(face, p1, u1, v1);

        assert(u0 == 0 || u0 == 1);
        assert(v0 == 0 || v0 == 1);
        assert(u1 != -1 && v1 != -1);

        assert(u0 != u1 && v0 != v1); // otherwise we'd be on an edge

        result._sourceFace = face;

        // intersect the line segment {(u0,v0),(u1,v1)} with the invalid region

        // clip to region "u <= t_threshold" or  "u >= 1-t_threshold"
        result._u = (u0 == 0 ? t_threshold : 1-t_threshold);
        result._v = v0 + ((v1-v0) * (result._u - u0) / (u1-u0));

        //      result.print();

        // clip to region "v <= t_threshold" or  "v >= 1-t_threshold"
        if ((v0 == 0 && result._v > t_threshold) || (v0 == 1 && result._v < 1-t_threshold))
        {
            result._v = (v0 == 0 ? t_threshold : 1-t_threshold);
            result._u = u0 + (u1 - u0)*(result._v - v0)/(v1-v0);
        }

        //      result._print();

        assert(result._u >=0 && result._u <= 1 && result._v >=0 && result._v <= 1);

        assert( (u0 == 0 && result._u <= u1) || (u0 == 1 && result._u >= u1));
        assert( (v0 == 0 && result._v <= v1) || (v0 == 1 && result._v >= v1));

        //  result.print();

        //      assert( fabsl(u0 - result._u) >= t_threshold-1e-10 || fabsl(v0 - result._v) >= t_threshold-1e-10);

        assert(result.IsEvaluable());

        result._normalOffset = newOffset;

        return result;
    }

    // ------------- We don't share a face, so use (A,B) parameterization -----------------

    real a0,b0,a1,b1;

    Chart<T> * chart = FindChart(extPoint,p1,a0,b0,a1,b1);
    //  HbrVertex<T> * origin = FindABOrigin(extPoint,p1,a0,b0,a1,b1);

    assert( chart != NULL);

    // intersect the line segment {(a0,b0),(a1,b1)} with the invalid region, which is a box around (a0,b0)

    real a,b;

    // clip to region "a <= t_threshold" or "a >= 1-t_threshold"
    a = (a1 < a0 ? a0 - t_threshold : a0 + t_threshold);
    b = b0 + (b1-b0)*(a-a0)/(a1-a0);

    // clip to region (b <= t_threshold" or "b >= 1-t_threshold"
    if (b < b0 - t_threshold || b > b0 + t_threshold)
    {
        b = (b < b0 - t_threshold ? b0 - t_threshold : b0 + t_threshold);
        a = a0 + (a1-a0)*(b-b0)/(b1-b0);
    }

    assert(a >= -1 && a <=1 && b>= -1 && b<=1);
    assert(fabsl(a0 - a) >= t_threshold-1e-10 || fabsl(b0 - b) >= t_threshold - 1e-10);

    result = chart->ABtoParam(a,b);
    result._normalOffset = newOffset;

    assert(result.IsEvaluable());

    return result;
}


// =================================== EXACT EVALUTAION ===============================

template<class T>
bool ParamPoint<T>::IsEvaluable() const
{
    return ALLOW_EXTRAORDINARY_INTERPOLATION || IsExactlyEvaluable();
}

// is this point not too close to an extraordinary point to be evaluable?
template<class T>
bool ParamPoint<T>::IsExactlyEvaluable() const
{
    assert(!IsNull());

    const real t_threshold = pow(.5,REF_LEVEL) + EXTRAORDINARY_REGION_OFFSET;

    if (_sourceVertex != NULL){
        //        if(_sourceVertex->GetValence() == 4 && !_sourceVertex->OnBoundary()) // all vertices are evaluable
        return true;
        //        return false;
    }

    if (_sourceFace != NULL)
    {
        if ( (_u == 0 || _u == 1) && (_v == 0 || _v == 1))
            return true;

        assert(_sourceFace->GetNumVertices() == 4);

        real uvert, vvert;
        int vind = NearestVertex(_u,_v,uvert,vvert);
        HbrVertex<T> * vert = _sourceFace->GetVertex(vind);

        if (vert->GetValence() == 4 && !vert->OnBoundary())
            return true;

        if (fabsl(uvert - _u) >= t_threshold || fabsl(vvert - _v) >= t_threshold)
            return true;

        return false;
    }

    if (_t == 0 || _t == 1 || _t == 0.5)
        return true;

    HbrVertex<T> * vert = _t < .5 ? _sourceEdge->GetOrgVertex() : _sourceEdge->GetDestVertex();

    if (vert->GetValence() == 4 && !vert->OnBoundary())
        return true;

    if ( ((_t < .5) && _t > t_threshold) || ((_t > 0.5) && (_t < 1-(t_threshold))))
        return true;

    return false;
}

// ================= EVALUATION =====================

template<class T>
void ParamPoint<T>::Evaluate(vec3 & limitPosition, vec3 & limitNormal, real * k1, real * k2, vec3 * d1, vec3 * d2) const
{
    HbrFace<T> * face = NULL;
    real u,v;
    vec3 tanU, tanV;

    // TODO: Use vertex-specific routines when possible?

    if (!ALLOW_EXTRAORDINARY_INTERPOLATION && !IsExactlyEvaluable())
    {
        const real t_threshold = pow(.5,REF_LEVEL) + EXTRAORDINARY_REGION_OFFSET;

        Print();
        printf("t_threshold = %f\n", double(t_threshold));
        fflush(stdout);

        printf("ERROR: TRIED TO EVALUTE AN NON-EVALUABLE LOCATION\n");
        assert(0);
    }

    vec2 pdir1 = vec2(1,0);
    vec2 pdir2 = vec2(0,1);

    if (IsExactlyEvaluable())
    {
        if (_sourceFace != NULL)
        {
            face = _sourceFace;
            u = _u;
            v = _v;
        }
        else
        {
            HbrHalfedge<T> * edge = _sourceEdge != NULL ? _sourceEdge : _sourceVertex->GetIncidentEdge();
            face = edge->GetLeftFace() == NULL ? edge->GetRightFace() : edge->GetLeftFace();
            GetFaceUV<T>(face,*this,u,v);
        }

        assert(face != NULL && u != -1 && v != -1);
        assert(u>=0 && u<=1 && v>=0 && v<=1);  // generic error-checking

        //      printf("Evaluating face %p at (%f,%f)\n",face, double(u),double(v));
        //      printf("\tface verts [%p, %p, %p, %p]\n",
        //	     face->GetVertex(0),	 face->GetVertex(1),	 face->GetVertex(2),	 face->GetVertex(3));

        if(k1 && k2){
            mat2 I, II, S;

            Subdiv::getInstance().Evaluate(OsdEvalCoords(face->GetID(),u,v),&limitPosition,&tanU,&tanV,&I,&II);

            real detI = determinant(I);
            if(fabsl(detI) < 1e-20){
                //printf("DET I NULL [%f,%f,%f,%f]\n",double(I[0][0]),double(I[0][1]),double(I[1][0]),double(I[1][1]));
                detI = 1e-20;
            }

            S[0][0] = (II[0][1]*I[0][1] - II[0][0]*I[1][1]) / detI;
            S[0][1] = (II[1][1]*I[0][1] - II[0][1]*I[1][1]) / detI;
            S[1][0] = (II[0][0]*I[0][1] - II[0][1]*I[0][0]) / detI;
            S[1][1] = (II[0][1]*I[0][1] - II[1][1]*I[0][0]) / detI;

//            printf("detI: %.16f / %.16f / %.16f / %.16f\n",double(detI),double(II[1][1]*I[0][1]), double(II[0][1]*I[1][1]), double((II[1][1]*I[0][1] - II[0][1]*I[1][1]) / detI));
//            printf(" I: %.16f %.16f %.16f %.16f\n",double(I[0][0]),double(I[0][1]),double(I[1][0]),double(I[1][1]));
//            printf("II: %.16f %.16f %.16f %.16f\n",double(II[0][0]),double(II[0][1]),double(II[1][0]),double(II[1][1]));
//            printf(" S: %.16f %.16f %.16f %.16f\n",double(S[0][0]),double(S[0][1]),double(S[1][0]),double(S[1][1]));

            real traceS = S[0][0] + S[1][1];
            real detS = determinant(S);
            real diff = traceS*traceS - 4.0 * detS;
            if(diff>=0){
                real sqrtDiff = sqrtl(diff);
                (*k1) = 0.5 * (traceS + sqrtDiff);
                (*k2) = 0.5 * (traceS - sqrtDiff);
                if(fabsl(*k1)<fabsl(*k2)){
                    real swap = (*k1);
                    (*k1) = (*k2);
                    (*k2) = swap;
                }
                if(fabsl(S[1][0])>1e-20){
                    pdir1 = vec2((*k1) - S[1][1], S[1][0]);
                    pdir2 = vec2((*k2) - S[1][1], S[1][0]);
                }else if (fabsl(S[0][1])>1e-20){
                    pdir1 = vec2(S[0][1], (*k1) - S[0][0]);
                    pdir2 = vec2(S[0][1], (*k2) - S[0][0]);
                }
                pdir1.normalize();
                pdir2.normalize();
                //printf("trace: %.16f detS: %.16f diff: %.16f k1*k2: %.16f / %.16f\n",double(traceS),double(detS),double(diff),double((*k1)*(*k2)),double((II[0][0]*II[1][1] - II[0][1]*II[0][1])/detI));
            }else{
                //#if LINK_FREESTYLE
                //    char str[200];
                //    sprintf(str, "DIFF NULL");
                //    addRIFDebugPoint(-1, double(limitPosition[0]), double(limitPosition[1]), double(limitPosition[2]), str, NULL);
                //#endif
                //                printf("DIFF NULL\n");
                (*k1) = 0.0;
                (*k2) = 0.0;
                (*d1) = tanU;
                (*d2) = tanV;
            }
        }else{
            Subdiv::getInstance().Evaluate(OsdEvalCoords(face->GetID(),u,v),&limitPosition,&tanU,&tanV,NULL,NULL);
            //_evaluator->Eval(face,u,v,REF_LEVEL,&limitPosition,&tanU,&tanV,NULL,NULL,NULL,0,NULL,0,NULL,0);
        }

        //        printf("tanV [%f, %f, %f]\n",(double)tanV[0],(double)tanV[1],(double)tanV[2]);
        //        printf("tanU [%f, %f, %f]\n",(double)tanU[0],(double)tanU[1],(double)tanU[2]);

        limitNormal = tanU ^ tanV;

        if (limitNormal * limitNormal < 1e-12) // a bug in Nsd, it appears
        {
            if (_sourceVertex != NULL)
            {
                limitNormal = FaceAveragedVertexNormal(_sourceVertex);
                printf("averaged vtx normal = %f %f %f\n", double(limitNormal[0]), double(limitNormal[1]), double(limitNormal[2]));
            }
        }

        // do some finite differences since the tangent evaluation doesn't work

        //  assert(u >=0 && u <=1 && u+hu >=0 && u+hu <=1);
        //  assert(v >=0 && v <=1 && v+hv >=0 && v+hv <=1);

        //  limitNormal = tan2 ^ tan1;
        //limitNormal.Normalize();

        //        if (limitNormal * limitNormal < 1e-12) // a bug in Nsd, it appears
        //        {
        //            // printf("VANISHING LIMIT NORMAL\n");
        //            //      print();
        //            #if LINK_FREESTYLE
        //                    char str[200];
        //                    sprintf(str, "VANISHING LIMIT NORMAL");
        //                    addRIFDebugPoint(-1, double(limitPosition[0]), double(limitPosition[1]), double(limitPosition[2]), str, NULL);
        //            #endif
        //            if (_sourceVertex != NULL)
        //            {
        //                limitNormal = FaceAveragedVertexNormal(_sourceVertex);
        //                //	  printf("averaged vtx normal = %f %f %f\n", double(limitNormal[0]), double(limitNormal[1]), double(limitNormal[2]));
        //            }
        //        }
        //        else
        limitNormal.normalize();

        if(d1 && d2){
            (*d1) = pdir1[0] * tanU + pdir1[1] * tanV;
            (*d1).normalize();
            (*d2) = (*d1) ^ limitNormal; //tanU * pdir2[0] + tanV * pdir2[1];
        }
    }
    else
        EvaluateByInterpolation(limitPosition, limitNormal, k1, k2, d1, d2);


    assert(limitPosition * limitPosition > 0);  // could theoretically happen
    assert(limitNormal * limitNormal > 0);

    if (_normalOffset != 0)
        limitPosition += _normalOffset * limitNormal;

#ifdef isnan
    assert(!isnan(limitPosition[0]));
#endif

    return;
}


template<class T>
void ParamPoint<T>::EvaluateByInterpolation(vec3 & limitPosition, vec3 & limitNormal, real* k1, real* k2, vec3 * d1, vec3 * d2) const
{
    // determine which four points to interpolate
    const real t_threshold = pow(.5,REF_LEVEL) + EXTRAORDINARY_REGION_OFFSET;

    assert(!IsExactlyEvaluable());

    limitPosition = vec3(0,0,0);
    limitNormal = vec3(0,0,0);
    if(k1 && k2){
        (*k1) = 0.0;
        (*k2) = 0.0;
        if(d1 && d2){
            (*d1) = vec3(0,0,0);
            (*d2) = vec3(0,0,0);
        }
    }

    HbrFace<T> * face;
    real u,v;

    //  printf("=========== Evaluate by interpolation ==== \n");
    //  print();

    if (_sourceFace != NULL)
    {
        face = _sourceFace;
        u = _u;
        v = _v;
    }
    else
    {
        face = _sourceEdge->GetLeftFace() != NULL ? _sourceEdge->GetLeftFace() : _sourceEdge->GetRightFace();
        assert(face != NULL);
        GetFaceUV<T>(face, *this, u,v);
    }

    assert(face->GetNumVertices() == 4);

    real uvert, vvert;
    int vind = NearestVertex(u,v,uvert,vvert);

    real u2 = (uvert == 0 ? t_threshold+1e-10 : 1-(t_threshold+1e-10));
    real v2 = (vvert == 0 ? t_threshold+1e-10 : 1-(t_threshold+1e-10));

    real alpha = (u - uvert)/(u2 - uvert);
    real beta =  (v - vvert)/(v2 - vvert);

    //  printf("alpha = %f, beta = %f, u = %f, uvert = %f, u2 = %f, v = %f, vvert = %f, v2 = %f\n",
    //	 double(alpha), double(beta), double(u), double(uvert), double(u2), double(v), double(vvert), double(v2));

    assert(alpha >=0 && alpha <=1 && beta >= 0 && beta <= 1);

    ParamPoint<T> corners[4] = { ParamPoint<T> (face->GetVertex(vind)),
                                 ParamPoint<T> (face, u2, vvert),
                                 ParamPoint<T> (face, uvert, v2),
                                 ParamPoint<T> (face, u2, v2) };
    real weights[4] = { (1-alpha) * (1-beta), alpha * (1-beta), (1-alpha)*beta, alpha * beta };

    assert(corners[0].SourceVertex() != NULL);

    for(int i=0;i<4;i++)
    {
        vec3 p, n;
        real k1c, k2c;
        vec3 d1c, d2c;
        //assert(corners[i].IsExactlyEvaluable());
        if(k1 && k2){
            corners[i].Evaluate(p,n,&k1c,&k2c,&d1c,&d2c);
            (*k1) += weights[i] * k1c;
            (*k2) += weights[i] * k2c;
            if(d1 && d2){
                (*d1) += weights[i] * d1c;
                (*d2) += weights[i] * d2c;
            }
        }else{
            corners[i].Evaluate(p,n);
        }
        limitPosition += weights[i] * p;
        limitNormal += weights[i] * n;
        //      if (tanU != NULL)
        //	tanU += weights[i] * tu;
        //      if (tanV != NULL)
        //	tanV += weights[i] * tv;
        //      printf("p: %f %f %f, n: %f %f %f\n", double(p[0]), double(p[1]), double(p[2]), double(n[0]), double(n[1]), double(n[2]));
        //      totalWeight += weights[i];
    }

    limitNormal.normalize();

    if(d1 && d2){
        (*d1).normalize();
        (*d2).normalize();
    }

    //  printf("Interpolated Limit pos: [%f,%f,%f], limit normal: [%f, %f, %f]\n",
    //  	 double(limitPosition[0]), double(limitPosition[1]), double(limitPosition[2]),
    //  	 double(limitNormal[0]), double(limitNormal[1]), double(limitNormal[2]));

    // total weight should always be 1.
    //  printf("Totalweight = %f\n",double(totalWeight));
}



// given a non-orthogonal 2D basis (v0,v1) in 3D, project vector w to this basis, fitting w = a0 v0 + a1 v1 in the least-squares sence
template<class T>
void ProjectVector(const vec3 & v0, const vec3 & v1, const vec3 w,
                   real & a0, real & a1)
{
    // Let A = [ v0 v1]; we want to solve A [a0 a1]' = w in least-squares.
    // The normal equations are: [a0 a1]' = (A' A)^-1 A' w


    // compute A'A
    real mtx[2][2] = { { v0 * v0, v0 * v1}, {v0 * v1, v1 * v1} };

    // compute det(A'A)     [ copied from gf/matrix2::Inverse() ]
    real det = mtx[0][0] * mtx[1][1] - mtx[0][1] * mtx[1][0];

    if (det == 0)
    {
        a0 = 0;
        a1 = 0;
        return;
    }

    real rcp = 1.0/det;
    real inverse[2][2];

    // compute (A'A)^{-1}
    inverse[0][0] = mtx[1][1]*rcp;
    inverse[0][1] = mtx[0][1]*-rcp;
    inverse[1][0] = mtx[1][0]*-rcp;
    inverse[1][1] = mtx[0][0]*rcp;

    // compute A' w
    real Atw0 = v0 * w;
    real Atw1 = v1 * w;

    // compute (A' A)^-1 A' w
    a0 = inverse[0][0] * Atw0 + inverse[0][1] * Atw1;
    a1 = inverse[1][0] * Atw0 + inverse[1][1] * Atw1;
}

template<class T>
ParamRay<T>
ParamPoint<T>::VectorToParamRay(const vec3 & worldVec) const
{
    vec3 limitPosition, tanU, tanV;

    HbrFace<T> * face = NULL;
    real u,v;

    if(!IsExactlyEvaluable())
        return ParamRay<T>();

    real du, dv;

    if (_sourceFace != NULL)
    {
        face = _sourceFace;
        u = _u;
        v = _v;

        Subdiv::getInstance().Evaluate(OsdEvalCoords(face->GetID(),u,v),&limitPosition,&tanU,&tanV,NULL,NULL);
        //_evaluator->Eval(face,u,v,REF_LEVEL,&limitPosition,&tanU,&tanV,NULL,NULL,NULL,0,NULL,0,NULL,0);

        ProjectVector<T>(tanU, tanV, worldVec, du, dv);
    }
    else
    {
        // gather all neighboring faces
        std::list<HbrHalfedge<T> *> edges;
        if (_sourceVertex != NULL)
            _sourceVertex->GetSurroundingEdges(std::back_inserter(edges));
        else
            edges.push_back(_sourceEdge);

        std::set<HbrFace<T> *> adjFaces;
        for(typename std::list<HbrHalfedge<T>*>::iterator it = edges.begin();it !=edges.end(); ++it)
        {
            if ((*it)->GetLeftFace() != NULL)
                adjFaces.insert((*it)->GetLeftFace());
            if ((*it)->GetRightFace() != NULL)
                adjFaces.insert((*it)->GetRightFace());
        }

        bool success = false;

        for(typename std::set<HbrFace<T> *>::iterator it = adjFaces.begin(); it != adjFaces.end(); ++it)
        {
            face = *it;
            GetFaceUV(face,*this,u,v);

            Subdiv::getInstance().Evaluate(OsdEvalCoords(face->GetID(),u,v),&limitPosition,&tanU,&tanV,NULL,NULL);
            //_evaluator->Eval(face,u,v,REF_LEVEL,&limitPosition,&tanU,&tanV,NULL,NULL,NULL,0,NULL,0,NULL,0);

            ProjectVector<T>(tanU, tanV, worldVec, du, dv);

            if (u == 0 && du < 0)
                continue;

            if (u == 1 && du > 0)
                continue;

            if (v == 0 && dv < 0)
                continue;

            if (v == 1 && dv > 0)
                continue;

            success = true;
            break;
        }

        if (!success){
            return ParamRay<T> ();
        }
    }

    assert(face != NULL && u != -1 && v != -1);
    assert(u>=0 && u<=1 && v>=0 && v<=1);  // generic error-checking
    assert(u != 0 || du >= 0);
    assert(u != 1 || du <= 0);
    assert(v != 0 || dv >= 0);
    assert(v != 1 || dv <= 0);


    //printf("Evaluating face %p at (%f,%f)\n",face, double(u),double(v));
    //printf("\tface verts [%p, %p, %p, %p]\n", face->GetVertex(0),	 face->GetVertex(1),	 face->GetVertex(2),	 face->GetVertex(3));

    ParamRay<T> ray(face,u,v,du,dv);

    return ray;
}

template<class T>
ParamRay<T>::ParamRay(HbrFace<T> * sourceFace, real u, real v, real du, real dv)
{
    _sourceFace = sourceFace;
    _u = u;
    _v = v;
    _du=du;
    _dv=dv;
}

template<class T>
ParamPoint<T>
ParamRay<T>::Advance() const
{
    if (IsNull())
        return ParamPoint<T>();

    assert(_u >=0 && _u <=1 && _v >= 0 && _v <=1);
    assert(_du != 0 || _dv != 0);
    assert(_sourceFace != NULL);

    assert((_u > 0 || _du >= 0) && (_u < 1 || _du <= 0) && (_v > 0 || _dv >= 0) && (_v < 1 || _dv <= 0));

    real u,v;

    if (_du == 0)
    {
        u = _u;
        v = _dv > 0 ? 1 : 0;

        return ParamPoint<T>(_sourceFace,u,v);
    }

    if (_dv == 0)
    {
        u = _du > 0 ? 1 : 0;
        v = _v;

        return ParamPoint<T>(_sourceFace,u,v);
    }

    if (_du > 0)  // clip to u=0
        u = 1;
    else
        if (_du < 0)  // clip to v=1
            u = 0;
        else
            u = _u;

    v = _dv* (u - _u)/_du + _v;

    // clip to V=0 or V = 1
    if (v > 1)
    {
        v = 1;
        u = _du*(v - _v)/_dv + _u;
    }
    else
        if (v < 0)
        {
            v = 0;
            u = _du*(v - _v)/_dv + _u;
        }

    assert(u>=0 && u<=1 && v>=0 && v<=1);
    assert(u==0 || u==1 || v==0 || v==1);

    //printf("Next point on face %p at (%f,%f)\n",_sourceFace, double(u),double(v));

    return ParamPoint<T>(_sourceFace,u,v);
}



// are P and Q on the same side of the line containing points (E0, E1)?
template<class T>
bool
ParamPoint<T>::SameSide(const ParamPoint<T>  & P, const ParamPoint<T>  & Q, const ParamPoint<T>  & E0, const ParamPoint<T>  & E1)
{
    std::vector<ParamPoint<T> > pts;
    pts.push_back(P);
    pts.push_back(Q);
    pts.push_back(E0);
    pts.push_back(E1);

    std::vector<vec2> abs;

    Chart<T> * chart = FindChart(pts,abs);
    //  HbrVertex<T> * origin = FindABOrigin(pts,abs);

    if (chart == NULL)
    {
        // perhaps a risky choice: if they don't share an chart, then they probably don't cross.
        // doing this in order to allow an extraordinary point to be shifted
        if (ALLOW_EXTRAORDINARY_INTERPOLATION)
            return true;
        else
            return false;
    }


    // compute sign(rot90(E_1 - E_0) dot (P - E_0)) == sign(rot90(E_1 - E_0) dot (Q - E_0))
    // would be easier if we created vec2
    // or could use Shewchuk's code here

    vec3 pAB(abs[0][0], abs[0][1],0);
    vec3 qAB(abs[1][0], abs[1][1],0);
    vec3 e0AB(abs[2][0], abs[2][1],0);
    vec3 e1AB(abs[3][0], abs[3][1],0);

    // rotate edge vector by 90 degrees
    vec3 normal(  abs[3][1] - abs[2][1], - (abs[3][0] - abs[2][0]), 0);

    real valP = (pAB - e0AB) * normal;
    real valQ = (qAB - e0AB) * normal;

    if (valP == 0 || valQ == 0)
        return false;

    return (  (valP < 0) == (valQ < 0));
}

template<class T>
real
ParamPoint<T>::IsophoteDistance(const CameraModel & camera, real isovalue, int maxDistance) const
{
    vec3 cameraCenter = camera.CameraCenter();

    if (!IsEvaluable())
        return -1;

    ParamPoint<T> pt0 = *this;

    vec3 pos0, normal0;
    pt0.Evaluate(pos0, normal0);
    vec3 viewVec0 = (cameraCenter - pos0);
    viewVec0.normalize();
    real rho0 = viewVec0 * normal0;

    vec3 startPos = pos0;

    if(rho0 > isovalue)
        return 0;

    for(int i=0;i<100;i++)
    {
        ParamRay<T> ray = pt0.VectorToParamRay(viewVec0);

        if (ray.IsNull())
            return -1;

        ParamPoint<T> pt1 = ray.Advance();
        if (pt1.IsNull() || !pt1.IsEvaluable())
            return -1;

        vec3 pos1, normal1;
        pt1.Evaluate(pos1, normal1);
        vec3 viewVec1 = cameraCenter - pos1;
        viewVec1.normalize();
        real rho1 = viewVec1 * normal1;

        if (rho1 < -0.1)
            return camera.ImageSpaceDistance(startPos,pos1); // could interpolate

        if (rho1 > isovalue)
        {
            // let rho(t) = (1-t) rho_0 + t rho_1
            // solving for t in rho(t) = isovalue:
            //   isovalue = t (rho_1 - rho_0) + rho_0
            // -> t = (isovalue - rho_0) / (rho_1 - rho_0)

            real t = (isovalue - rho0) / (rho1 - rho0);
            ParamPoint<T> isopt = ParamPoint::Interpolate(pt0, pt1, t);
            vec3 isoPos, isoNormal;
            isopt.Evaluate(isoPos, isoNormal);
            return camera.ImageSpaceDistance(startPos, isoPos);
        }

        if (camera.ImageSpaceDistance(startPos, pos1) > maxDistance)
            return maxDistance;

        pt0 = pt1;
        rho0 = rho1;
        pos0 = pos1;
        viewVec0 = viewVec1;
    }

    return maxDistance;
}

template<class T>
bool ParamPoint<T>::RadialCurvature(const vec3 & cameraCenter,real & k_r) const
// finite differences using the "(D_w n) dot w / (w dot w)"
{
    k_r = 0.0;
    if(_sourceVertex != NULL || (_sourceEdge !=NULL && (_t == 0 || _t == 1)) ||
            (_sourceFace !=NULL && (_u==0 || _u==1) && (_v==0 || _v==1) )){
        HbrVertex<T> * vertex = NULL;
        if(_sourceVertex != NULL){
            vertex = _sourceVertex;
        }else if(_sourceEdge!=NULL){
            vertex = _t==0 ? _sourceEdge->GetOrgVertex() : _sourceEdge->GetDestVertex();
        }else{
            if(_u==0)
                if(_v==0)
                    vertex = _sourceFace->GetVertex(0);
                else
                    vertex = _sourceFace->GetVertex(3);
            else
                if(_v==0)
                    vertex = _sourceFace->GetVertex(1);
                else
                    vertex = _sourceFace->GetVertex(2);
        }
        assert(vertex);
        const real t_threshold = pow(.5,REF_LEVEL) + EXTRAORDINARY_REGION_OFFSET+1e-10;
        if(vertex->GetValence() != 4 || vertex->OnBoundary()){
            std::set<HbrFace<T>*> oneRingFaces;
            GetOneRing(vertex,oneRingFaces);
            for(typename std::set<HbrFace<T>*>::iterator it=oneRingFaces.begin(); it!=oneRingFaces.end(); it++){
                HbrFace<T>* face = (*it);
                real u,v;
                GetVertexUV(face,vertex,u,v);
                if(u<0.5)
                    u += t_threshold;
                else
                    u -= t_threshold;
                if(v<0.5)
                    v += t_threshold;
                else
                    v -= t_threshold;
                ParamPoint<T> paramP = ParamPoint<T>(face,u,v);
                assert(paramP.IsEvaluable());
                real radCurv_c;
                paramP.RadialCurvature(cameraCenter,radCurv_c);
                k_r += radCurv_c;
            }
            k_r /= real(oneRingFaces.size());
            return true;
        }
    }

    vec3 limitPosition, limitNormal;
    real k1, k2;
    vec3 pdir1, pdir2;

    Evaluate(limitPosition, limitNormal, &k1, &k2, &pdir1, &pdir2);

    vec3 viewVec = limitPosition - cameraCenter;
    viewVec.normalize();

    real ndotv = limitNormal * viewVec;
    real sintheta = 1.0 - ndotv*ndotv;
    real u = (viewVec * pdir1), u2 = u*u;
    real v = (viewVec * pdir2), v2 = v*v;
    k_r = (k1 * u2 + k2 * v2) / sintheta;

    return true;

    k_r = -123456;

    if(!IsExactlyEvaluable()){
        HbrFace<T> * face = NULL;
        real u,v;

        if (_sourceFace != NULL){
            face = _sourceFace;
            u = _u;
            v = _v;
        }else{
            face = _sourceEdge->GetLeftFace() != NULL ? _sourceEdge->GetLeftFace() : _sourceEdge->GetRightFace();
            assert(face != NULL);
            GetFaceUV<T>(face, *this, u,v);
        }
        real uvert, vvert;
        int vind = NearestVertex(u,v,uvert,vvert);

        const real t_threshold = pow(.5,REF_LEVEL) + EXTRAORDINARY_REGION_OFFSET+1e-10;

        real u2 = (uvert == 0 ? t_threshold+1e-10 : 1-(t_threshold+1e-10));
        real v2 = (vvert == 0 ? t_threshold+1e-10 : 1-(t_threshold+1e-10));

        real alpha = (u - uvert)/(u2 - uvert);
        real beta =  (v - vvert)/(v2 - vvert);

        assert(alpha >=0 && alpha <=1 && beta >= 0 && beta <= 1);

        ParamPoint<T> corners[4] = { ParamPoint<T> ( face->GetVertex(vind)),
                                     ParamPoint<T> ( face, u2, vvert),
                                     ParamPoint<T> ( face, uvert, v2),
                                     ParamPoint<T> ( face, u2, v2) };
        //real weights[4] = { (1-alpha) * (1-beta), alpha * (1-beta), (1-alpha)*beta, alpha * beta };

        k_r = 0.0;
        for(int i=0; i<4; i++){
            assert(corners[i].IsExactlyEvaluable());
            real radCurv;
            corners[i].RadialCurvature(cameraCenter,radCurv);
            k_r += radCurv;
        }
        return k_r;
    }

    vec3 pos0, normal0;
    Evaluate(pos0, normal0);

    //vec3 viewVec = cameraCenter - pos0;
    //viewVec.Normalize();
    //vec3 planeNormal = normalize(viewVec ^ normal0);

#if 0
    HbrFace<T> * face = NULL;

    ParamPoint<T> p1,p2;
    vec3 limitPos, limitN;
    real d1, d2;
    vec3 pos1, normal1;

    bool success = false;

    real max_ndotv = -DBL_MAX;

    std::set<HbrFace<T> *> adjFaces;
    if (_sourceFace == NULL){
        // gather all neighboring faces
        std::list<HbrHalfedge<T> *> edges;
        if (_sourceVertex != NULL)
            _sourceVertex->GetSurroundingEdges(std::back_inserter(edges));
        else
            edges.push_back(_sourceEdge);

        for(typename std::list<HbrHalfedge<T>*>::iterator it = edges.begin();it !=edges.end(); ++it){
            if ((*it)->GetLeftFace() != NULL)
                adjFaces.insert((*it)->GetLeftFace());
            if ((*it)->GetRightFace() != NULL)
                adjFaces.insert((*it)->GetRightFace());
        }
    }else{
        adjFaces.insert(_sourceFace);
    }

    real d;
    const int numSamples = 10;
    int t_f = numSamples;
    ParamPoint<T> intersection;
    for(typename std::set<HbrFace<T> *>::iterator it = adjFaces.begin();
        it != adjFaces.end(); ++it){
        face = *it;

        for(int i=0; i<face->GetNumVertices(); i++){
            HbrHalfedge<T>* edge = face->GetEdge(i);
            if(_sourceEdge && (edge == _sourceEdge || edge == _sourceEdge->GetOpposite())){
                //printf("SKIP\n");
                continue;
            }
            if(_sourceVertex && (edge->GetOrgVertex() == _sourceVertex || edge->GetDestVertex() == _sourceVertex)){
                //printf("SKIP\n");
                continue;
            }
            p1 = ParamPoint<T>(edge->GetOrgVertex());
            // p1.Evaluate(limitPos, limitN);
            // d1 = planeNormal * (limitPos - pos0);
            p2 = ParamPoint<T>(edge->GetDestVertex());
            // p2.Evaluate(limitPos, limitN);
            // d2 = planeNormal * (limitPos - pos0);

            // if((d1>0.0 && d2<0.0) || (d1<0.0 && d2>0.0)){

            ParamPoint<T> leftParam, rightParam;
            int t=1;
            do{
                if(t>numSamples)
                    break;
                if(t==numSamples){
                    leftParam = p1;
                    rightParam = p2;
                }else{
                    leftParam = ParamPoint<T>::Interpolate(*this,p1,real(t)/real(numSamples));
                    rightParam = ParamPoint<T>::Interpolate(*this,p2,real(t)/real(numSamples));
                }

                leftParam.Evaluate(limitPos, limitN);
                d1 = planeNormal * (limitPos - pos0);
                rightParam.Evaluate(limitPos, limitN);
                d2 = planeNormal * (limitPos - pos0);
                t += 1;
            }while((d1>0.0 && d2>0.0) || (d1<0.0 && d2<0.0) && (t<=numSamples));
            //printf("d1=%f d2=%f t=%d\n",double(d1),double(d2),t-1);

            if((d1>0.0 && d2<0.0) || (d1<0.0 && d2>0.0)){
                if(d1>0){
                    real dtmp = d2;
                    p2 = rightParam;
                    d2 = d1;
                    p1 = leftParam;
                    d1 = dtmp;
                }else{
                    p1 = leftParam;
                    p2 = rightParam;
                }
                const int NUM_SAMPLES = 40;

                // Find intersection with the radial plan by root-finding
                int s;
                for(s=0; s<=NUM_SAMPLES; s++){
                    intersection = ParamPoint<T>::Interpolate(p1,p2,0.5);
                    assert(!intersection.IsNull() && intersection.IsEvaluable());
                    intersection.Evaluate(limitPos,limitN);
                    d = planeNormal * (limitPos - pos0);
                    if(d<-1e-10){
                        p1 = intersection;
                        d1 = d;
                    }else if(d>1e-10){
                        p2 = intersection;
                        d2 = d;
                    }else{
                        break;
                    }
                }

                // intersection =  ParamPoint<T>::Interpolate(*this, intersection, 0.1); // shorten the displacement
                // intersection.Evaluate(limitPos,limitN);

                vec3 dir = (limitPos - pos0);
                dir.Normalize();
                real ndotv = dir * viewVec;
                //printf("max_ndotv=%.10f ndotv=%.10f\n",double(max_ndotv),double(ndotv));
                if(t<=t_f && ndotv>=0.0 && s<NUM_SAMPLES){ //ndotv>max_ndotv || (
                    //printf("FOUND %0.20f\n",double(d));
                    max_ndotv = ndotv;
                    pos1 = limitPos;
                    normal1 = limitN;
                    t_f = t;
                    success=true;
                }
            }
        }
    }

    assert(success);

    // pt1 is some arbitrary distance away, so redefine this vector as the "w" vector; might not be mathematically correct
    vec3 w = pos1 - pos0;

    if(w * viewVec<0)
        printf("BACK\n");
    //else
    //  printf("FRONT\n");


    if (w * w == 0){
        // printf("\nw*w==0\n");
        return false;
    }

    k_r = ( ( normal1 - normal0 ) * w ) / (w * w);
    d = planeNormal * w;
#endif

    ParamRay<T> ray = VectorToParamRay(viewVec);

    if (ray.IsNull()){
        ray = VectorToParamRay(-1.*viewVec);
        if (ray.IsNull()){
            return false;
        }
    }

    ParamPoint<T> p3 = ray.Advance();

    if (p3.IsNull() || !p3.IsEvaluable()){
        printf("RAY INTERSECTION IS NULL\n");
        return false;
    }

    p3 =  ParamPoint<T>::Interpolate(*this, p3, 0.05); // shorten the displacement

    if (p3.IsNull() || !p3.IsEvaluable()){
        printf("SHORTENED DISPLACEMENT IS NULL\n");
        return false;
    }

    vec3 pos2, normal2;
    p3.Evaluate(pos2, normal2);

    vec3 w2 = pos2 - pos0;

    //if(w2 * viewVec<0)
    //  printf("BACK\n");

    if (w2 * w2 == 0){
        printf("W.W IS NULL\n");
        return false;
    }

    real k_r2 = ( ( normal2 - normal0 ) * w2 ) / (w2 * w2);
    k_r = k_r2;

    /*
    if(fabs(k_r-k_r2)>0.1 || (k_r>0 && k_r2<0) || (k_r<0 && k_r2>0)){
      printf("\nk_r = %0.10f / k_r2 = %0.10f\n",double(k_r),double(k_r2));
      this->Print();
      intersection.Print();
      p3.Print();
      real l = length(w);
      //w.Normalize();
      printf("w   : %f %f %f / %f / %d\n", double(w[0]), double(w[1]), double(w[2]), double(l), t_f);
      l = length(w2);
      //w2.Normalize();
      printf("w2   : %f %f %f / %f\n", double(w2[0]), double(w2[1]), double(w2[2]), double(l));
      printf("nml1: %f %f %f\n", double(normal1[0]), double(normal1[1]), double(normal1[2]));
      printf("nml2: %f %f %f\n", double(normal2[0]), double(normal2[1]), double(normal2[2]));
      printf("w.vv normalized : %f\n", double(w * viewVec)/sqrt(double((w * w) * (viewVec * viewVec))));
      printf("w2.vv normalized : %f\n", double(w2 * viewVec)/sqrt(double((w2 * w2) * (viewVec * viewVec))));
      printf("w.w : %f\n",double(w*w));
      printf("w2.w2 : %f\n",double(w2*w2));
      real d2 = planeNormal * w2;
      printf("d=%.10f d2=%.10f\n",double(d),double(d2));
      //k_r = k_r2;
    }
   */

    /*
    if ( (normal0 * viewVec) * (normal0 * viewVec) < 0.0001)
    {
      printf("=====================\n");
      printf("pos0: %f %f %f\n", double(pos0[0]), double(pos0[1]), double(pos0[2]));
      printf("pos1: %f %f %f\n", double(pos1[0]), double(pos1[1]), double(pos1[2]));
      printf("nml0: %f %f %f\n", double(normal0[0]), double(normal0[1]), double(normal0[2]));
      printf("nml1: %f %f %f\n", double(normal1[0]), double(normal1[1]), double(normal1[2]));
      printf("vv  : %f %f %f\n", double(viewVec[0]), double(viewVec[1]), double(viewVec[2]));
      printf("w   : %f %f %f\n", double(w[0]), double(w[1]), double(w[2]));
      printf("n.v : %f\n", double(normal0 * viewVec));
      printf("w.vv normalized : %f\n", double(w * viewVec)/sqrt(double((w * w) * (viewVec * viewVec))));
      printf("w.w : %f\n",double(w*w));
      printf("k_r: %f\n", double(k_r));
    }
    */
    return true;
}

/*
bool ParamPoint<T>::IsValid() const
{
  return
    (_sourceVertex == NULL && _sourceEdge == NULL && _sourceFace == NULL) ||
    (_sourceVertex != NULL && _sourceEdge == NULL && _sourceFace == NULL) ||
    (_sourceVertex == NULL && _sourceEdge != NULL && _sourceFace == NULL) ||
    (_sourceVertex == NULL && _sourceEdge == NULL && _sourceFace != NULL);
}
*/


template<class T>
ParamPoint<T>
ParamRay<T>::IntersectSegment(const ParamPoint<T> & p1, const ParamPoint<T> & p2) const
{
    if(IsNull()){
        return ParamPoint<T>();
    }
    ParamPoint<T> rayStart( _sourceFace, _u, _v);
    ParamPoint<T> nextPt = Advance();

    if(!ParamPoint<T>::SameSide(rayStart,nextPt,p1,p2)){
        if(nextPt.SourceEdge()){
            if((nextPt.SourceEdge()->GetOrgVertex()==p1.SourceVertex() && nextPt.SourceEdge()->GetDestVertex()==p2.SourceVertex()) ||
                    (nextPt.SourceEdge()->GetOrgVertex()==p2.SourceVertex() && nextPt.SourceEdge()->GetDestVertex()==p1.SourceVertex())){
                return nextPt;
            }
        }
    }else{
        return ParamPoint<T>();
    }

    std::vector<ParamPoint<T> > pts;
    pts.push_back(rayStart);
    pts.push_back(nextPt);
    pts.push_back(p1);
    pts.push_back(p2);
    std::vector<vec2> abs;
    Chart<T> * chart = ParamPoint<T>::FindChart(pts,abs);

    if(chart == NULL){
        printf("Chart not found\n");
        return ParamPoint<T> ();
    }

    real raydir_a = abs[1][0] - abs[0][0];
    real raydir_b = abs[1][1] - abs[0][1];

    // line segment direction
    real linedir_a = abs[3][0] - abs[2][0];
    real linedir_b = abs[3][1] - abs[2][1];

    real denom = linedir_b * raydir_a - linedir_a * raydir_b;
    real u_a = (linedir_a * (abs[0][1] - abs[2][1])
            - linedir_b * (abs[0][0] - abs[2][0])) / denom;
    real u_b = (raydir_a * (abs[0][1] - abs[2][1])
            - raydir_b * (abs[0][0] - abs[2][0])) / denom;

    ParamPoint<T>  result = chart->ABtoParam(abs[0][0] + u_a * raydir_a, abs[0][1] + u_a * raydir_b);
    //ParamPoint<T>( _sourceFace, abs[0][0] + u_a * raydir_a, abs[0][1] + u_a * raydir_b, 0.0);

    if (!result.IsNull() && u_a>=0.0 && u_a<=1.0 && u_b>=0.0 && u_b<=1.0){
        // && ParamPoint<T>::SameSide(result, p1, rayStart, p2) && ParamPoint<T>::SameSide(result, p2, rayStart, p1)){
        //printf("...found at (a=%f,b=%f)\n",abs[0][0] + u_a * raydir_a, abs[0][1] + u_b * raydir_b);
        //printf("u_a = %f, u_b = %f\n",u_a,u_b);
        return result;
    }else{
        //printf("...IsNull (a=%f, b=%f)\n",abs[0][0] + u_a * raydir_a, abs[0][1] + u_a * raydir_b);
        return ParamPoint<T>();
    }
}

#endif
