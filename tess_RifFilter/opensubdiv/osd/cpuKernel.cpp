//
//   Copyright 2013 Pixar
//
//   Licensed under the Apache License, Version 2.0 (the "Apache License")
//   with the following modification; you may not use this file except in
//   compliance with the Apache License and the following modification to it:
//   Section 6. Trademarks. is deleted and replaced with:
//
//   6. Trademarks. This License does not grant permission to use the trade
//      names, trademarks, service marks, or product names of the Licensor
//      and its affiliates, except as required to comply with Section 4(c) of
//      the License and to reproduce the content of the NOTICE file.
//
//   You may obtain a copy of the Apache License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the Apache License with the above modification is
//   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
//   KIND, either express or implied. See the Apache License for the specific
//   language governing permissions and limitations under the Apache License.
//

#include "../osd/cpuKernel.h"
#include "../osd/vertexDescriptor.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

void OsdCpuComputeFace(
    OsdVertexDescriptor const &vdesc, real * vertex, real * varying,
    const int *F_IT, const int *F_ITa, int vertexOffset, int tableOffset,
    int start, int end) {
    if(vdesc.numVertexElements == 4 && varying == NULL) {
        ComputeFaceKernel<4>
            (vertex, F_IT, F_ITa, vertexOffset, tableOffset, start,  end);
    } else if(vdesc.numVertexElements == 8 && varying == NULL) {
        ComputeFaceKernel<8>
            (vertex, F_IT, F_ITa, vertexOffset, tableOffset, start,  end);
    }
    else {
        for (int i = start + tableOffset; i < end + tableOffset; i++) {
            int h = F_ITa[2*i];
            int n = F_ITa[2*i+1];

            real weight = 1.0f/n;

            // XXX: should use local vertex struct variable instead of
            // accumulating directly into global memory.
            int dstIndex = i + vertexOffset - tableOffset;
            vdesc.Clear(vertex, varying, dstIndex);

            for (int j = 0; j < n; ++j) {
                int index = F_IT[h+j];
                vdesc.AddWithWeight(vertex, dstIndex, index, weight);
                vdesc.AddVaryingWithWeight(varying, dstIndex, index, weight);
            }
        }   
    }
}

void OsdCpuComputeEdge(
    OsdVertexDescriptor const &vdesc, real *vertex, real *varying,
    const int *E_IT, const real *E_W, int vertexOffset, int tableOffset,
    int start, int end) {
    if(vdesc.numVertexElements == 4 && varying == NULL) {
        ComputeEdgeKernel<4>(vertex, E_IT, E_W, vertexOffset, tableOffset,
                             start, end);
    }
    else if(vdesc.numVertexElements == 8 && varying == NULL) {
        ComputeEdgeKernel<8>(vertex, E_IT, E_W, vertexOffset, tableOffset,
                             start, end);    
    }
    else {
        for (int i = start + tableOffset; i < end + tableOffset; i++) {
            int eidx0 = E_IT[4*i+0];
            int eidx1 = E_IT[4*i+1];
            int eidx2 = E_IT[4*i+2];
            int eidx3 = E_IT[4*i+3];

            real vertWeight = E_W[i*2+0];

            int dstIndex = i + vertexOffset - tableOffset;
            vdesc.Clear(vertex, varying, dstIndex);

            vdesc.AddWithWeight(vertex, dstIndex, eidx0, vertWeight);
            vdesc.AddWithWeight(vertex, dstIndex, eidx1, vertWeight);

            if (eidx2 != -1) {
                real faceWeight = E_W[i*2+1];

                vdesc.AddWithWeight(vertex, dstIndex, eidx2, faceWeight);
                vdesc.AddWithWeight(vertex, dstIndex, eidx3, faceWeight);
            }

            vdesc.AddVaryingWithWeight(varying, dstIndex, eidx0, 0.5f);
            vdesc.AddVaryingWithWeight(varying, dstIndex, eidx1, 0.5f);
        }    
    }
}

void OsdCpuComputeVertexA(
    OsdVertexDescriptor const &vdesc, real *vertex, real *varying,
    const int *V_ITa, const real *V_W, int vertexOffset, int tableOffset,
    int start, int end, int pass) {
    if(vdesc.numVertexElements == 4 && varying == NULL) {
        ComputeVertexAKernel<4>(vertex, V_ITa, V_W, vertexOffset, tableOffset,
                             start, end, pass);
    }
    else if (vdesc.numVertexElements == 8 && varying == NULL) {
        ComputeVertexAKernel<8>(vertex, V_ITa, V_W, vertexOffset, tableOffset,
                             start, end, pass);
    }    
    else {
        for (int i = start + tableOffset; i < end + tableOffset; i++) {
            int n     = V_ITa[5*i+1];
            int p     = V_ITa[5*i+2];
            int eidx0 = V_ITa[5*i+3];
            int eidx1 = V_ITa[5*i+4];

            real weight = (pass == 1) ? V_W[i] : 1.0f - V_W[i];

            // In the case of fractional weight, the weight must be inverted since
            // the value is shared with the k_Smooth kernel (statistically the
            // k_Smooth kernel runs much more often than this one)
            if (weight > 0.0f && weight < 1.0f && n > 0)
                weight = 1.0f - weight;

            int dstIndex = i + vertexOffset - tableOffset;

            if (not pass)
                vdesc.Clear(vertex, varying, dstIndex);

            if (eidx0 == -1 || (pass == 0 && (n == -1))) {
                vdesc.AddWithWeight(vertex, dstIndex, p, weight);
            } else {
                vdesc.AddWithWeight(vertex, dstIndex, p, weight * 0.75f);
                vdesc.AddWithWeight(vertex, dstIndex, eidx0, weight * 0.125f);
                vdesc.AddWithWeight(vertex, dstIndex, eidx1, weight * 0.125f);
            }

            if (not pass)
                vdesc.AddVaryingWithWeight(varying, dstIndex, p, 1.0f);    
        }        
    }
}

void OsdCpuComputeVertexB(
    OsdVertexDescriptor const &vdesc, real *vertex, real *varying,
    const int *V_ITa, const int *V_IT, const real *V_W,
    int vertexOffset, int tableOffset, int start, int end) {
    if(vdesc.numVertexElements == 4 && varying == NULL) {
        ComputeVertexBKernel<4>(vertex, V_ITa, V_IT, V_W,
            vertexOffset, tableOffset, start, end);
    }
    else if(vdesc.numVertexElements == 8 && varying == NULL) {
        ComputeVertexBKernel<8>(vertex, V_ITa, V_IT, V_W,
            vertexOffset, tableOffset, start, end);
    }    
    else {
        for (int i = start + tableOffset; i < end + tableOffset; i++) {
            int h = V_ITa[5*i];
            int n = V_ITa[5*i+1];
            int p = V_ITa[5*i+2];

            real weight = V_W[i];
            real wp = 1.0f/static_cast<real>(n*n);
            real wv = (n-2.0f) * n * wp;

            int dstIndex = i + vertexOffset - tableOffset;
            vdesc.Clear(vertex, varying, dstIndex);

            vdesc.AddWithWeight(vertex, dstIndex, p, weight * wv);

            for (int j = 0; j < n; ++j) {
                vdesc.AddWithWeight(vertex, dstIndex, V_IT[h+j*2], weight * wp);
                vdesc.AddWithWeight(vertex, dstIndex, V_IT[h+j*2+1], weight * wp);
            }
            vdesc.AddVaryingWithWeight(varying, dstIndex, p, 1.0f);
        }
    }    
}

void OsdCpuComputeLoopVertexB(
    OsdVertexDescriptor const &vdesc, real *vertex, real *varying,
    const int *V_ITa, const int *V_IT, const real *V_W,
    int vertexOffset, int tableOffset, int start, int end) {
    if(vdesc.numVertexElements == 4 && varying == NULL) {
        ComputeLoopVertexBKernel<4>(vertex, V_ITa, V_IT, V_W, vertexOffset, 
                              tableOffset, start, end);
    }
    else if(vdesc.numVertexElements == 8 && varying == NULL) {
        ComputeLoopVertexBKernel<8>(vertex, V_ITa, V_IT, V_W, vertexOffset, 
                              tableOffset, start, end);    
    }    
    else {
        for (int i = start + tableOffset; i < end + tableOffset; i++) {
            int h = V_ITa[5*i];
            int n = V_ITa[5*i+1];
            int p = V_ITa[5*i+2];

            real weight = V_W[i];
            real wp = 1.0f/static_cast<real>(n);
            real beta = 0.25f * cosf(static_cast<real>(M_PI) * 2.0f * wp) + 0.375f;
            beta = beta * beta;
            beta = (0.625f - beta) * wp;

            int dstIndex = i + vertexOffset - tableOffset;
            vdesc.Clear(vertex, varying, dstIndex);

            vdesc.AddWithWeight(vertex, dstIndex, p, weight * (1.0f - (beta * n)));

            for (int j = 0; j < n; ++j)
                vdesc.AddWithWeight(vertex, dstIndex, V_IT[h+j], weight * beta);

            vdesc.AddVaryingWithWeight(varying, dstIndex, p, 1.0f);
        } 
    }    
}

void OsdCpuComputeBilinearEdge(
    OsdVertexDescriptor const &vdesc, real *vertex, real *varying,
    const int *E_IT, int vertexOffset, int tableOffset, int start, int end) {
    if(vdesc.numVertexElements == 4 && varying == NULL) {
        ComputeBilinearEdgeKernel<4>(vertex, E_IT, vertexOffset, tableOffset, 
                                     start, end);
    }
    else if(vdesc.numVertexElements == 8 && varying == NULL) {
        ComputeBilinearEdgeKernel<8>(vertex, E_IT, vertexOffset, tableOffset, 
                                     start, end);      
    }
    else {
        for (int i = start + tableOffset; i < end + tableOffset; i++) {
            int eidx0 = E_IT[2*i+0];
            int eidx1 = E_IT[2*i+1];

            int dstIndex = i + vertexOffset - tableOffset;
            vdesc.Clear(vertex, varying, dstIndex);

            vdesc.AddWithWeight(vertex, dstIndex, eidx0, 0.5f);
            vdesc.AddWithWeight(vertex, dstIndex, eidx1, 0.5f);

            vdesc.AddVaryingWithWeight(varying, dstIndex, eidx0, 0.5f);
            vdesc.AddVaryingWithWeight(varying, dstIndex, eidx1, 0.5f);
        }    
    }
}

void OsdCpuComputeBilinearVertex(
    OsdVertexDescriptor const &vdesc, real *vertex, real *varying,
    const int *V_ITa, int vertexOffset, int tableOffset, int start, int end) {
    int numVertexElements  = vdesc.numVertexElements;
    int numVaryingElements = vdesc.numVaryingElements;
    real *src, *des;
    for (int i = start + tableOffset; i < end + tableOffset; i++) {
        int p = V_ITa[i];

        int dstIndex = i + vertexOffset - tableOffset;            
        src = vertex + p        * numVertexElements;
        des = vertex + dstIndex * numVertexElements;            
        memcpy(des, src, sizeof(real)*numVertexElements);
        if(varying) {
            src = varying + p        * numVaryingElements;
            des = varying + dstIndex * numVaryingElements;            
            memcpy(des, src, sizeof(real)*numVaryingElements);
        }
    }
}

void OsdCpuEditVertexAdd(
    OsdVertexDescriptor const &vdesc, real *vertex,
    int primVarOffset, int primVarWidth, int vertexOffset, int tableOffset,
    int start, int end,
    const unsigned int *editIndices, const real *editValues) {

    for (int i = start+tableOffset; i < end+tableOffset; i++) {
        vdesc.ApplyVertexEditAdd(vertex,
                                 primVarOffset,
                                 primVarWidth,
                                 editIndices[i] + vertexOffset,
                                 &editValues[i*primVarWidth]);
    }
}

void OsdCpuEditVertexSet(
    OsdVertexDescriptor const &vdesc, real *vertex,
    int primVarOffset, int primVarWidth, int vertexOffset, int tableOffset,
    int start, int end,
    const unsigned int *editIndices, const real *editValues) {

    for (int i = start+tableOffset; i < end+tableOffset; i++) {
        vdesc.ApplyVertexEditSet(vertex,
                                 primVarOffset,
                                 primVarWidth,
                                 editIndices[i] + vertexOffset,
                                 &editValues[i*primVarWidth]);
    }
}

}  // end namespace OPENSUBDIV_VERSION
}  // end namespace OpenSubdiv
