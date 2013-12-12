#ifndef __SUBDIV_H__
#define __SUBDIV_H__

#include "VecMat.h"
#include <osd/cpuEvalLimitContext.h>
#include <osd/cpuEvalLimitController.h>
#include <osd/cpuGLVertexBuffer.h>
#include <osd/cpuVertexBuffer.h>
#include <osd/cpuComputeContext.h>
#include <osd/cpuComputeController.h>
#include <osd/cpuEvalLimitContext.h>
#include <osd/cpuEvalLimitController.h>
#include <osd/drawContext.h>
#include <osd/mesh.h>
#include <osd/vertex.h>
#include <osd/error.h>

using namespace OpenSubdiv;

//------------------------------------------------------------------------------

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

typedef HbrMesh<OsdVertex>              OsdHbrMesh;
typedef FarMesh<OsdVertex>              OsdFarMesh;
typedef FarMeshFactory<OsdVertex>       OsdFarMeshFactory;
typedef FarSubdivisionTables<OsdVertex> OsdFarMeshSubdivision;

//------------------------------------------------------------------------------

class Subdiv {
public:
    static Subdiv& getInstance()
    {
        static Subdiv instance;
        return instance;
    }

    void initialize(CatmarkMesh * surface);

    void Evaluate(OsdEvalCoords coord, vec3 *limitPos, vec3 *tanU, vec3 *tanV, mat2* I, mat2* II);
private:
    Subdiv() {}

    OsdFarMesh * g_farmesh;
    OsdCpuComputeContext * g_osdComputeContext;
    OsdCpuComputeController g_osdComputeController;
    OsdCpuEvalLimitController g_evaluator;

    OsdVertexBufferDescriptor g_idesc, g_odesc;
    OsdCpuVertexBuffer * g_Q, * g_dQu,* g_dQv, * g_dQudQu, * g_dQvdQv,* g_dQudQv;
    OsdCpuVertexBuffer * g_vertexData;
    OsdCpuEvalLimitContext * g_evalCtx;
};

#endif
