#ifndef __PARAMPOINT_H__
#define __PARAMPOINT_H__

#include "subdiv.h"
#include "cameraModel.h"

const int REF_LEVEL = 10;
const double EXTRAORDINARY_REGION_OFFSET = 1e-5; //1e-10;
const bool ALLOW_EXTRAORDINARY_INTERPOLATION = true;

template<class T>
class ParamRay;

template<class T>
class Chart;

// ------- location of a point on a base mesh with vertexdata T and surface evaluator EvalT -------
template<class T>
class ParamPoint 
{
private:
    HbrVertex<T> * _sourceVertex; // source vertex if the point comes from a vertex, or NULL if none

    HbrHalfedge<T> * _sourceEdge; // source edge if the points lies on the interior of a face, or NULL if none
    real _t;     // edge parameter if this point is on an edge. point is (1-t) * org + t * dest

    HbrFace<T> * _sourceFace; // source face if the point comes from the interior of a face, or NULL if none
    real _u, _v;  // face parameters if this is an interior point

    real _normalOffset;

public:
    // -- constructors --
    ParamPoint() { clear(); }
    ParamPoint(HbrVertex<T> * v, real normalOffset = 0);
    ParamPoint(HbrHalfedge<T> * e, real tin, real normalOffset = 0);
    ParamPoint(HbrFace<T> * f, real uin, real vin, real normalOffset = 0);
    void clear() { _sourceVertex = NULL; _sourceEdge = NULL; _sourceFace = NULL; _t = -1; _u = -1; _v = -1; _normalOffset = 0; }

    // -- printout --
    void Print() const;
    void PrintString(char * str) const;

    // -- accessors --
    bool IsNull() const { return _sourceVertex == NULL && _sourceEdge == NULL && _sourceFace == NULL; }
    HbrVertex<T> * SourceVertex() const { return _sourceVertex; }
    HbrHalfedge<T> * SourceEdge() const { return _sourceEdge; }
    real EdgeT() const { return _t; }
    HbrFace<T> * SourceFace() const { return _sourceFace; }
    real FaceU() const { return _u; }
    real FaceV() const { return _v; }
    real NormalOffset() const { return _normalOffset; }
    void SetNormalOffset(real v) { _normalOffset = v; }
    bool IsExtraordinaryVertex() const { return _sourceVertex != NULL && (_sourceVertex->GetValence() != 4 || _sourceVertex->OnBoundary()); }

    // -- evaluation --
    void Evaluate(vec3 & limitPosition, vec3 & limitNormal, real * k1=NULL, real * k2=NULL, vec3 * d1=NULL, vec3 * d2=NULL) const;
    bool IsEvaluable() const;  // can we compute a limit position and normal for this point?
    bool IsExactlyEvaluable() const;
    bool RadialCurvature(const vec3 & cameraCenter, real & k_r) const;
    real IsophoteDistance(const CameraModel & camera, real isovalue, int maxDistance) const;
    void EvaluateByInterpolation(vec3 & limitPosition, vec3 & limitNormal, real* k1=NULL, real* k2=NULL, vec3 * d1=NULL, vec3 * d2=NULL) const;
    //  bool IsValid() const; // debugging

    // -- interpolation --
    // given t\in[0..1], find the source point that (1-t)p0 + tp1, assuming that p0 and p1 lie on the same face,
    // the points are distinct, and 0 < t < 1.
    // if the resulting point is invalid for the desired refinement level, then return false and give the
    // "nearest" evaluable point instead.
    static ParamPoint<T> Interpolate(const ParamPoint<T> & p0, const ParamPoint<T> & p1, real t);
    static ParamPoint<T> MakeExtraordinarySeparator(const ParamPoint<T> & extPoint, const ParamPoint<T> & p2);

    // -- parameterization --
    //  static bool ConvexInAB(const ParamPoint<T> & p0, const ParamPoint<T> & p1,const ParamPoint<T> & p2, const ParamPoint<T> & p3);
    // are P and Q on the same side of the line containing points (E0, E1)?
    static bool SameSide(const ParamPoint<T> & P, const ParamPoint<T> & Q, const ParamPoint<T> & E0, const ParamPoint<T> & E1);

    static Chart<T> * FindChart(const std::vector<ParamPoint<T> > & pts, std::vector<vec2> & abs);
    static Chart<T> * FindChart(const ParamPoint<T>  & p0, const ParamPoint<T>  & p1, real & a0, real & b0, real & a1, real & b1);
    static bool HasCommonChart(const ParamPoint<T> & p1, const ParamPoint<T> & p2);
    static bool ConvexInChart(const ParamPoint<T> & p0, const ParamPoint<T> & p1,const ParamPoint<T> & p2, const ParamPoint<T> & p3);

    ParamRay<T> VectorToParamRay(const vec3 & worldVec) const;
};

template<class T>
class ParamRay
{
private:
    HbrFace<T> * _sourceFace;
    real _u,_v;  // base
    real _du,_dv; // direction

public:
    ParamRay() { _sourceFace = NULL; _u = _v = _du = _dv = -1; }
    ParamRay(HbrFace<T> * sourceFace, real u, real v, real du, real dv);
    bool IsNull() const { return _sourceFace == NULL; }
    ParamPoint<T> IntersectSegment(const ParamPoint<T> & p1, const ParamPoint<T> & p2) const;

    ParamPoint<T> Advance() const;  // advance the ray to the next edge in the base mesh

    friend class ParamPoint<T>;
};

#include "paramPointFunctions.h"

#endif
