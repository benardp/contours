#ifndef __REFINE_CONTOUR_H__
#define __REFINE_CONTOUR_H__

#include <string.h>

#include "subdiv.h"
#include "paramPoint.h"
#include "chart.h"

typedef enum { FRONT, BACK, CONTOUR } FacingType;
typedef enum { RF_NONE, RF_CONTOUR_ONLY, RF_CONTOUR_INCONSISTENT, RF_FULL, RF_OPTIMIZE, RF_RADIAL } RefinementType;

class Vertex {

public:
    Vertex() { }

    Vertex( int /*i*/ ) { }

    Vertex( const Vertex & src ) { _pos[0]=src._pos[0]; _pos[1]=src._pos[1]; _pos[2]=src._pos[2]; }

    ~Vertex( ) { }

    void AddWithWeight(const Vertex& src, real weight, void * =0 ) {
        _pos[0]+=weight*src._pos[0];
        _pos[1]+=weight*src._pos[1];
        _pos[2]+=weight*src._pos[2];
    }

    void AddVaryingWithWeight(const Vertex& , real, void * =0 ) { }

    void Clear( void * =0 ) { _pos[0]=_pos[1]=_pos[2]=0.0f; }

    void SetPosition(real x, real y, real z) { _pos[0]=x; _pos[1]=y; _pos[2]=z; }

    void ApplyVertexEdit(const OpenSubdiv::HbrVertexEdit<Vertex> & edit) {
        const real *src = edit.GetEdit();
        switch(edit.GetOperation()) {
        case OpenSubdiv::HbrHierarchicalEdit<Vertex>::Set:
            _pos[0] = src[0];
            _pos[1] = src[1];
            _pos[2] = src[2];
            break;
        case OpenSubdiv::HbrHierarchicalEdit<Vertex>::Add:
            _pos[0] += src[0];
            _pos[1] += src[1];
            _pos[2] += src[2];
            break;
        case OpenSubdiv::HbrHierarchicalEdit<Vertex>::Subtract:
            _pos[0] -= src[0];
            _pos[1] -= src[1];
            _pos[2] -= src[2];
            break;
        }
    }

    void ApplyMovingVertexEdit(const OpenSubdiv::HbrMovingVertexEdit<Vertex> &) { }

    // custom functions & data not required by Hbr -------------------------

    Vertex( real x, real y, real z ) { _pos[0]=x; _pos[1]=y; _pos[2]=z; }

    const vec3 GetPos() const { return _pos; }

    vec3 _pos;
};

typedef HbrMesh<Vertex>     CatmarkMesh;
typedef HbrVertex<Vertex>   CatmarkVertex;
typedef HbrFace<Vertex>     CatmarkFace;
typedef HbrHalfedge<Vertex> CatmarkHalfedge;

//------------------------------------------------------------------------------

typedef ParamPoint<Vertex> ParamPointCC;  // sourceLoc for a Catmull-Clark surface
typedef ParamRay<Vertex> ParamRayCC;
typedef Chart<Vertex> ChartCC;

struct VertexDataCatmark
{
  vec3 pos;                // limit position
  vec3 normal;             // limit normal
  FacingType facing;       // facing direction, as a function of limit normal
  ParamPointCC sourceLoc;  // parametric location on the source mesh
  real ndotv;

  // information on how this vertex was created
  bool extraordinary;     // source point is an extraordinary point
  HbrVertex<VertexDataCatmark> * extSrc; // if this is an extraordinary region endpoint, the source extr. point
  bool isShifted;         // variable is redundant, could just check if origLoc is null...
  ParamPointCC origLoc;   // if this vertex got "shifted," where did it begin?
  bool shiftSplit;        // created during shifting to make sure parameterizations exist.

  real radialCurvature;   // only computed at the end
  real isophoteDistance;
  real k1;
  real k2;

  bool cusp; //smooth cusp
  HbrVertex<VertexDataCatmark> * radialOrg[2];

  bool rootFindingFailed;
  bool degenerate;
  int id;
  int age;
  VertexDataCatmark() { Clear(); }
  VertexDataCatmark(int i) { Clear(); id = i; }
  void Clear() { id = -1; age = -1; rootFindingFailed = degenerate = extraordinary = isShifted = cusp = false; extSrc = NULL; shiftSplit = false; isophoteDistance = 6; radialOrg[0] = radialOrg[1] = NULL;}
  int DebugString(char *) const;
  vec3 GetPos() { return pos; }
  bool Radial() const { return radialOrg[0]!=NULL; }
  void AddRadialOrg(HbrVertex<VertexDataCatmark> *v) { if(Radial()){ assert(!radialOrg[1]); radialOrg[1] = v; }else{ radialOrg[0] = v;  }}
  bool HasRadialOrg(HbrVertex<VertexDataCatmark> *v) { return radialOrg[0] == v || radialOrg[1] == v; }
  int NumRadialOrg() const { if (radialOrg[0] == NULL) return 0; else if (radialOrg[1] == NULL) return 1; else return 2; }
};

typedef HbrMesh<VertexDataCatmark> Mesh;
typedef HbrVertex<VertexDataCatmark> MeshVertex;
typedef HbrFace<VertexDataCatmark> MeshFace;
typedef HbrHalfedge<VertexDataCatmark> MeshEdge;

template<class T>
class FacePriorityQueue
        // a queue that supports insertion of arbitrary elements and redundant insertions
        // implementation right now is very quick and dirty
{
private:
    std::set<HbrFace<T>*> _readySet;  // all of the faces in the queue
    std::multimap<real,HbrFace<T>*> _queue;  // ordered list of faces


    real _Priority(const HbrFace<T>*) const;  // function that determines the priority of a face


public:
    HbrFace<T> * PopFront(); // return the highest-priority element
    void Insert(HbrFace<T>*); // add a face to the queue (even if it's already there)
    void Remove(HbrFace<T>*); // remove a face from the queue
    int Size() const { return _readySet.size(); }
    bool HasFace(HbrFace<T>* f) const { return _readySet.find(f) != _readySet.end(); }
    void Clear() { _readySet.clear();  _queue.clear(); }
};

typedef FacePriorityQueue<VertexDataCatmark> PriorityQueueCatmark;

void SavePLYFile(HbrMesh<VertexDataCatmark> * outputMesh, const char* prefix, int index, bool meshSilhouettes=true);

bool IsRadialFace(HbrFace<VertexDataCatmark>* face);
bool IsStandardRadialFace(HbrFace<VertexDataCatmark>* face);
bool IsStandardRadialFace(MeshVertex* v0, MeshVertex* v1, MeshVertex* v2);

real TriangleQuality(const MeshVertex * v1, const MeshVertex * v2, const MeshVertex * v3);
real TriangleQuality(MeshVertex * const v[]);
real TriangleQuality(const MeshFace * face);

bool FindBestSplitPointFace(MeshFace * face, bool allowShifts, const vec3 & cameraCenter, Mesh * mesh,
                            std::set<std::pair<MeshVertex*,MeshVertex*> > & badEdges,
                            PriorityQueueCatmark & wiggleQueue, PriorityQueueCatmark & splitQueue);

// sample an initial triangle mesh from a surface, clipping to the view frustum
HbrMesh<VertexDataCatmark> * SurfaceToMesh(CatmarkMesh * surface, int subdivisionLevel,
                                           const CameraModel & cameraModel, bool triangles);

// perform contour filtering on a sampled mesh, in order to have a consistent smooth mesh contour
void RefineContour(HbrMesh<VertexDataCatmark> * mesh, const vec3 & cameraCenter,
		   const RefinementType refinement, const bool allowShifts,
		   const int maxInconsistentSplits);

typedef enum { PREPROCESS, DETECT_CUSP, INSERT_CONTOUR, INSERT_CUSP, INSERT_RADIAL, FLIP_RADIAL, EXTEND_RADIAL, FLIP_EDGE, WIGGLING_PARAM, SPLIT_EDGE, EVERYTHING} RefineRadialStep;

void RefineContourRadial(HbrMesh<VertexDataCatmark> * mesh, const vec3 & cameraCenter, const bool allowShifts, const RefineRadialStep lastStep);

bool EnsureShiftable(MeshVertex * shiftVertex, const ParamPointCC & targetLoc,
                     const vec3 & cameraCenter, Mesh * mesh,
                     PriorityQueueCatmark * wiggleQueue, PriorityQueueCatmark * splitQueue, bool testMode);

void CullBackFaces(HbrMesh<VertexDataCatmark> * mesh);

void WiggleInParamSpace(HbrMesh<VertexDataCatmark> * mesh, const vec3 & cameraCenter);

HbrMesh<VertexDataCatmark> * DuplicateMesh(HbrMesh<VertexDataCatmark> * sourceMesh);

bool FindContour(MeshVertex * vA, MeshVertex * vB, vec3 cameraCenter, ParamPointCC & resultPoint, MeshVertex  * &  extSrc);

MeshVertex * ShiftVertex(MeshVertex * vA, MeshVertex * vB, const ParamPointCC & newLoc, const vec3 & cameraCenter,
                         Mesh * mesh, PriorityQueueCatmark * wiggleQueue, PriorityQueueCatmark * splitQueue,
                         bool testMode, bool enqueueNewFaces=true);

bool SplitZeroCrossingFace(MeshFace * face, Mesh * mesh,const vec3 & cameraCenter, bool allowShifts,
                           PriorityQueueCatmark & wiggleQueue, PriorityQueueCatmark & splitQueue,
                           std::set<std::pair<MeshVertex*,MeshVertex*> > & badEdges,
                           const std::set<std::pair<MeshVertex*,MeshVertex*> > & cuspEdges,
                           bool isCusp=false);

void SplitZeroCrossingEdge(MeshFace * face, int oppVertex, Mesh * mesh,
                           const vec3 & cameraCenter,  const bool allowShifts,
                           PriorityQueueCatmark & wiggleQueue,PriorityQueueCatmark & splitQueue,
                           std::pair<MeshVertex*,MeshVertex*> & badEdge,
                           const std::set<std::pair<MeshVertex*,MeshVertex*> > & cuspEdges, bool isCusp=false);

bool FlipFace(MeshFace * face, Mesh * mesh, const vec3 & cameraCenter,
              PriorityQueueCatmark & wiggleQueue, PriorityQueueCatmark & splitQueue);

void ComputeRadialCurvatures(HbrMesh<VertexDataCatmark> * mesh, const CameraModel & camera, real isovalue, int maxIsophoteDistance);

// The face orientation according to the vertices of the face; CONTOUR if the face is not vertex-consistent
// e.g., for a face that's CCF, CFF, FFF, return F; for CCB, CBB, BBB, return B; otherwise return C.
// (The actual orientation of the face is irrelevant.)
template<class T>
FacingType VertexBasedFacing(const HbrFace<T> * face);

template<class T>
void CullBackFaces(HbrMesh<T> * mesh);

template<class T>
HbrFace<T> * NewFaceDebug(HbrMesh<T> * mesh, int numVertices, int * IDs, bool tryReverse = false);

template<class T>
void OptimizeConsistency(HbrMesh<T> * outputMesh, const vec3 & cameraCenter, real lambda, real epsilon);

void ComputeConsistencyStats(HbrMesh<VertexDataCatmark> * outputMesh, const vec3 & cameraCenter, int & numInconsistent, int & numStrongInconsistent,
                             int &numNonRadial, int &numInconsistentContour, int &numInconsistentRadial);

#ifdef LINK_FREESTYLE
void addRIFDebugPoint(int type, double x, double y, double z, char * debugString, double radialCurvature);
#endif

const int MAX_ROOT_ITERATIONS = 100;
const real DEGENERATE_VERTEX_THRESHOLD = 0.0001; //0.5;
const real MIN_LONGEST_EDGE_LENGTH_PIXELS = 0.001;

const real MAX_SHIFT_PERCENTAGE = 0.2; //1; //0.4;  // 1 = no effective threshold

const int NUM_SPLIT_SAMPLES = 11; // want an odd number so that the midpoint is included
const int NUM_TRANSVERSE_SPLIT_SAMPLES = 0;

const int NUM_WIGGLE_SAMPLES_SQRT = 11;
const int NUM_NORMAL_WIGGLE_SAMPLES = 1; //7;  // 1 means no normal wiggling

const int NUM_INCONSISTENT_SAMPLES = 200;  // set to 0 to not sample. this is implemented in a very inefficient way (many redundant computations)

// these thresholds are important to prevent infinite loops, otherwise the numerics go pear-shaped on tiny triangles
const real MIN_INCONSISTENT_TRIANGLE_AREA = 1e-20; //1e-5;
const real MIN_TRIANGLE_AREA = 1e-20;  //1e-10 or 1e-5?

const bool ENFORCE_SHIFTABLE = true;
const bool REQUIRE_SEPARATORS = false; // only active when extraordinary interpolation is on.  having this on without interpolation will fail.

const real CONTOUR_THRESHOLD = 1e-8; //0.01; //1e-6; //0.000001; // 0.000001;

#include "refineContourFunctions.h"

#endif
