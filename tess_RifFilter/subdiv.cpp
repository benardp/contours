#include "subdiv.h"

void Subdiv::initialize(CatmarkMesh * surface)
{
    static HbrCatmarkSubdivision<OsdVertex> _catmark;

    OsdHbrMesh * hmesh = new OsdHbrMesh(&_catmark);

    std::vector<real> g_positions;
    g_positions.resize(surface->GetNumVertices()*3,0.0f);

    for (unsigned i = 0; i < surface->GetNumVertices(); i ++) {
        const CatmarkVertex* v = surface->GetVertex(i);
        OsdVertex vert;
        hmesh->NewVertex(v->GetID(), vert);
        const vec3& pos = v->GetData().GetPos();
        g_positions[i*3+0] = pos[0];
        g_positions[i*3+1] = pos[1];
        g_positions[i*3+2] = pos[2];
    }

    int ptxidx=0;
    for (unsigned i = 0; i < surface->GetNumFaces(); i++) {
        const CatmarkFace* f = surface->GetFace(i);
        const int nv = f->GetNumVertices();
        int fv[nv];
        for (unsigned j = 0; j < nv; j++) {
            fv[j] = f->GetVertexID(j);
        }

        HbrFace<OsdVertex> * face  = hmesh->NewFace(nv,fv,f->GetID());
        face->SetPtexIndex(ptxidx);

        if( nv != 4 )
            ptxidx += nv;
        else
            ptxidx++;
    }

    hmesh->SetInterpolateBoundaryMethod(OsdHbrMesh::k_InterpolateBoundaryEdgeOnly);
    hmesh->Finish();

    // Create FAR mesh
    OsdFarMeshFactory meshFactory(hmesh, 3, /*adaptive*/ true);
    g_farmesh = meshFactory.Create(/*fvar*/ true);

    // Create v-buffer & populate w/ positions
    int nverts = g_farmesh->GetNumVertices();
    g_vertexData = OsdCpuVertexBuffer::Create(3, nverts);
    g_vertexData->UpdateData( &g_positions[0], 0, nverts);

    // Create a Compute context, used to "pose" the vertices
    g_osdComputeContext = OsdCpuComputeContext::Create(g_farmesh);

    g_osdComputeController.Refine(g_osdComputeContext, g_farmesh->GetKernelBatches(), g_vertexData);

    // Create eval context & data buffers
    g_evalCtx = OsdCpuEvalLimitContext::Create(g_farmesh, /*requireFVarData*/ true);

    g_idesc = OsdVertexBufferDescriptor( /*offset*/ 0, /*length*/ 3, /*stride*/ 3 );
    g_odesc = OsdVertexBufferDescriptor( /*offset*/ 0, /*length*/ 3, /*stride*/ 6 );

    g_Q = OsdCpuVertexBuffer::Create(3,1);
    memset( g_Q->BindCpuBuffer(), 0, 3*sizeof(real));

    g_dQu = OsdCpuVertexBuffer::Create(3,1);
    memset( g_dQu->BindCpuBuffer(), 0, 3*sizeof(real));

    g_dQv = OsdCpuVertexBuffer::Create(3,1);
    memset( g_dQv->BindCpuBuffer(), 0, 3*sizeof(real));

    g_dQudQu = OsdCpuVertexBuffer::Create(3,1);
    memset( g_dQudQu->BindCpuBuffer(), 0, 3*sizeof(real));

    g_dQvdQv = OsdCpuVertexBuffer::Create(3,1);
    memset( g_dQvdQv->BindCpuBuffer(), 0, 3*sizeof(real));

    g_dQudQv = OsdCpuVertexBuffer::Create(3,1);
    memset( g_dQudQv->BindCpuBuffer(), 0, 3*sizeof(real));
}

void Subdiv::Evaluate(OsdEvalCoords coord, vec3 *limitPos, vec3 *tanU, vec3 *tanV, mat2* I, mat2* II)
{
    g_evalCtx->GetVertexData().Bind( g_idesc, g_vertexData, g_odesc, g_Q, g_dQu, g_dQv, g_dQudQu, g_dQvdQv, g_dQudQv);
    g_evalCtx->GetVaryingData().Unbind();

    int n = g_evaluator.EvalLimitSample<OsdCpuVertexBuffer,OsdCpuVertexBuffer>(coord,g_evalCtx,0);

    g_evalCtx->GetVertexData().Unbind();

    if (n) {
        real* sample = g_Q->BindCpuBuffer();
        limitPos->setX(sample[0]);
        limitPos->setY(sample[1]);
        limitPos->setZ(sample[2]);
        if(tanU){
            sample = g_dQu->BindCpuBuffer();
            tanU->setX(sample[0]);
            tanU->setY(sample[1]);
            tanU->setZ(sample[2]);
        }
        if(tanV){
            sample = g_dQv->BindCpuBuffer();
            tanV->setX(sample[0]);
            tanV->setY(sample[1]);
            tanV->setZ(sample[2]);
        }
        if(I && II){
            assert(tanU!=0 && tanV!=0);
            sample = g_dQudQu->BindCpuBuffer();
            vec3 dudu = vec3(sample[0], sample[1], sample[2]);
            sample = g_dQvdQv->BindCpuBuffer();
            vec3 dvdv = vec3(sample[0], sample[1], sample[2]);
            sample = g_dQudQv->BindCpuBuffer();
            vec3 dudv = vec3(sample[0], sample[1], sample[2]);

            real xform[2][2] = { {(*tanU)*(*tanU), (*tanU)*(*tanV)}, {(*tanU)*(*tanV), (*tanV)*(*tanV)} };
            I->Set(xform);
            vec3 normal = (*tanU) ^ (*tanV);
            normal.normalize();
            real xform2[2][2] = { {dudu*normal, dudv*normal}, {dudv*normal, dvdv*normal} };
            II->Set(xform2);
        }
    }
}
