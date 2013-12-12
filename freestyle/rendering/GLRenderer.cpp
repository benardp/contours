
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

#include "../scene_graph/IndexedFaceSet.h"
#include "../scene_graph/NodeDrawingStyle.h"
#include "../scene_graph/NodeLight.h"
#include "../scene_graph/NodeCamera.h"
#include "../scene_graph/NodeTransform.h"
#include "../scene_graph/NodeShape.h"
#include "../scene_graph/OrientedLineRep.h"
#include "../scene_graph/VertexRep.h"
#include "../stroke/Stroke.h"

#include "../scene_graph/TriangleRep.h"

#include "GLRenderer.h"

static GLenum lights[8] = {GL_LIGHT0, 
                           GL_LIGHT1,
                           GL_LIGHT2,
                           GL_LIGHT3,
                           GL_LIGHT4,
                           GL_LIGHT5,
                           GL_LIGHT6,
                           GL_LIGHT7};

extern bool orientableSurfaces;

inline
bool frontFacing(Vec3r p1, Vec3r p2, Vec3r p3, Vec3r viewpoint)
{
    // or maybe backfacing...

    Vec3r b1 = p2 - p1;
    Vec3r b2 = p3 - p1;
    Vec3r n = b1 ^ b2;

    Vec3r pv = p1 - viewpoint;

    return (pv * n) > 0;
}

inline bool frontFacing(const real * iVertices, const unsigned * iVIndices, int index, Vec3r viewpoint)
{
    assert(index>=0);
    Vec3r p1(iVertices[iVIndices[index  ]], iVertices[iVIndices[index  ]+1], iVertices[iVIndices[index  ]+2]);
    Vec3r p2(iVertices[iVIndices[index+1]], iVertices[iVIndices[index+1]+1], iVertices[iVIndices[index+1]+2]);
    Vec3r p3(iVertices[iVIndices[index+2]], iVertices[iVIndices[index+2]+1], iVertices[iVIndices[index+2]+2]);

    return frontFacing(p1,p2,p3, viewpoint);
}

void GLRenderer::visitIndexedFaceSet(IndexedFaceSet& ifs)
{
    /*GLuint dl = ifs.displayList();
  if(dl != 0){
    glCallList(dl);
    return;
  }*/
    unsigned int fIndex = 0;

    const real * vertices = ifs.vertices();
    const real * normals = ifs.normals();
    const real * texCoords = ifs.texCoords();
    const Material *const* materials = ifs.materials();
    const unsigned *vindices = ifs.vindices();
    const unsigned *nindices = ifs.nindices();
    const unsigned *mindices = ifs.mindices();
    const unsigned *tindices = ifs.tindices();
    const unsigned numfaces = ifs.numFaces();
    const IndexedFaceSet::TRIANGLES_STYLE * faceStyle = ifs.trianglesStyle();
    const unsigned *numVertexPerFace = ifs.numVertexPerFaces();


    const unsigned* pvi = vindices;
    const unsigned* pni = nindices;
    const unsigned* pmi = mindices;
    const unsigned* pti = tindices;


    //dl = glGenLists(1);
    //glNewList(dl, GL_COMPILE_AND_EXECUTE);
    for(fIndex=0; fIndex<numfaces; fIndex++)
    {
        switch(faceStyle[fIndex])
        {
        case IndexedFaceSet::TRIANGLE_STRIP:
            //     printf("TRIANGLE STRIP\n");
            RenderTriangleStrip(vertices, normals, materials, texCoords, pvi, pni, pmi, pti, numVertexPerFace[fIndex]);
            break;
        case IndexedFaceSet::TRIANGLE_FAN:
            //      printf("TRIANGLE FAN\N");
            RenderTriangleFan(vertices, normals, materials, texCoords, pvi, pni, pmi, pti, numVertexPerFace[fIndex]);
            break;
        case IndexedFaceSet::TRIANGLES:
            //      printf("TRIANGLES\n");
            RenderTriangles(vertices, normals, materials, texCoords, pvi, pni, pmi, pti, numVertexPerFace[fIndex]);
            break;
        }
        pvi += numVertexPerFace[fIndex];
        pni += numVertexPerFace[fIndex];
        if(pmi)
            pmi += numVertexPerFace[fIndex];
        if(pti)
            pti += numVertexPerFace[fIndex];
    }
    //glEndList();
    //ifs.SetDisplayList(dl);
}

void GLRenderer::visitNodeTransform(NodeTransform& tn) {
    if(tn.scaled())
        glEnable(GL_NORMALIZE);
}

void GLRenderer::visitNodeTransformBefore(NodeTransform& tn) {
    glPushMatrix();

    // Now apply transform
    applyTransform(tn.matrix());
}

void GLRenderer::visitNodeTransformAfter(NodeTransform& tn) {
    glPopMatrix();
}

void GLRenderer::visitNodeLight(NodeLight& ln)  
{
    if(true != ln.isOn())
        return;

    int number = ln.number();

    glLightfv(lights[number], GL_AMBIENT, ln.ambient());
    glLightfv(lights[number], GL_DIFFUSE, ln.diffuse());
    glLightfv(lights[number], GL_SPECULAR, ln.specular());
    glLightfv(lights[number], GL_POSITION, ln.position());

    glEnable(lights[number]);
}

void GLRenderer::visitNodeCamera(NodeCamera& cn)  
{
    const double * mvm = cn.modelViewMatrix();
    const double * pm = cn.projectionMatrix();

    printf("applying camera node\n");

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMultMatrixd(pm);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glMultMatrixd(mvm);


}

void GLRenderer::visitNodeDrawingStyleBefore(NodeDrawingStyle& ds) {
    glPushAttrib(GL_ALL_ATTRIB_BITS);
}

void GLRenderer::visitNodeDrawingStyleAfter(NodeDrawingStyle&) {
    glPopAttrib();
}

void GLRenderer::RenderTriangleStrip( const real *iVertices, 
                                      const real *iNormals,
                                      const Material *const* iMaterials,
                                      const real *iTexCoords,
                                      const unsigned* iVIndices,
                                      const unsigned* iNIndices,
                                      const unsigned* iMIndices,
                                      const unsigned* iTIndices,
                                      const unsigned iNVertices)
{
    if (_frontBackStyle)
        printf("WARNING: FRONT-BACK RENDERING STYLE NOT IMPLEMENTED FOR TRIANGLE STRIPS\n");

    unsigned index = -1;
    glBegin(GL_TRIANGLE_STRIP);
    for(unsigned int i=0; i<iNVertices; i++)
    {
        if(iMIndices){
            if(iMIndices[i] != index){
                visitMaterial(*(iMaterials[iMIndices[i]]));
                index = iMIndices[i];
            }
        }

        if(iTIndices){
            glTexCoord2f(   iTexCoords[iTIndices[i]],
                            iTexCoords[iTIndices[i]+1]);
        }

        glNormal3r(iNormals[iNIndices[i]],
                   iNormals[iNIndices[i]+1],
                   iNormals[iNIndices[i]+2]);

        glVertex3r( iVertices[iVIndices[i]],
                    iVertices[iVIndices[i]+1],
                    iVertices[iVIndices[i]+2]);
    }
    glEnd();
}

void GLRenderer::RenderTriangleFan( const real *iVertices, 
                                    const real *iNormals,
                                    const Material *const* iMaterials,
                                    const real *iTexCoords,
                                    const unsigned* iVIndices,
                                    const unsigned* iNIndices,
                                    const unsigned* iMIndices,
                                    const unsigned* iTIndices,
                                    const unsigned iNVertices)
{
    if (_frontBackStyle)
        printf("WARNING: FRONT-BACK RENDERING STYLE NOT IMPLEMENTED FOR TRIANGLE FANS\n");

    unsigned index = -1;
    glBegin(GL_TRIANGLE_FAN);
    for(unsigned int i=0; i<iNVertices; i++)
    {
        if(iMIndices){
            if(iMIndices[i] != index){
                visitMaterial(*(iMaterials[iMIndices[i]]));
                index = iMIndices[i];
            }
        }
        if(iTIndices){
            glTexCoord2f(   iTexCoords[iTIndices[i]],
                            iTexCoords[iTIndices[i]+1]);
        }

        glNormal3r(iNormals[iNIndices[i]],
                   iNormals[iNIndices[i]+1],
                   iNormals[iNIndices[i]+2]);

        glVertex3r( iVertices[iVIndices[i]],
                    iVertices[iVIndices[i]+1],
                    iVertices[iVIndices[i]+2]);
    }
    glEnd();
}

const int frontFaceColor[3] = { 255, 229, 180 }; // peach
const int backFaceColor[3] = { 64, 178, 255 }; // sky blue


void GLRenderer::RenderTriangles( const real *iVertices, 
                                  const real *iNormals,
                                  const Material *const* iMaterials,
                                  const real *iTexCoords,
                                  const unsigned* iVIndices,
                                  const unsigned* iNIndices,
                                  const unsigned* iMIndices,
                                  const unsigned* iTIndices,
                                  const unsigned iNVertices)
{
    unsigned index = -1;
    glBegin(GL_TRIANGLES);
    for(unsigned int i=0; i<iNVertices; i++)
    {
        if(iMIndices){
            if(iMIndices[i] != index){
                visitMaterial(*(iMaterials[iMIndices[i]]));
                index = iMIndices[i];
            }
        }
        if(iTIndices){
            glTexCoord2f(   iTexCoords[iTIndices[i]],
                            iTexCoords[iTIndices[i]+1]);
        }

        glNormal3r(iNormals[iNIndices[i]],
                   iNormals[iNIndices[i]+1],
                   iNormals[iNIndices[i]+2]);

        if (_frontBackStyle && (i%3) ==0)
        {
            // check if this is front or back-facing by looking at the current triangle

            bool fb = frontFacing(iVertices, iVIndices, i, _fbViewpoint);

            if (fb)
                glColor3ub(backFaceColor[0], backFaceColor[1], backFaceColor[2]);
            else
                glColor3ub(frontFaceColor[0], frontFaceColor[1], frontFaceColor[2]);

            /*
    if (fb)
      glColor3ub(128,128,255);
    else
      glColor3ub(255,64,255);
    */
        }

        glVertex3r( iVertices[iVIndices[i]],
                    iVertices[iVIndices[i]+1],
                    iVertices[iVIndices[i]+2]);
    }
    glEnd();
}

void GLRenderer::visitLineRep( LineRep& iLine) 
{
    if(iLine.width() != 0)
        glLineWidth(iLine.width());

    switch(iLine.style())
    {
    case LineRep::LINES:
        glBegin(GL_LINES);
        break;
    case LineRep::LINE_STRIP:
        glBegin(GL_LINE_STRIP);
        break;
    case LineRep::LINE_LOOP:
        glBegin(GL_LINE_LOOP);
        break;
    default:
        return;
    }

    const vector<Vec3r>& vertices = iLine.vertices();
    float step=1.f/vertices.size();
    vector<Vec3r>::const_iterator v;

    for(v=vertices.begin(); v!=vertices.end(); v++)
        glVertex3r((*v)[0], (*v)[1], (*v)[2]);

    glEnd();
}


void GLRenderer::visitTriangleRep( TriangleRep& iTriangle) 
{
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    switch(iTriangle.style())
    {
    case TriangleRep::FILL:
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        break;
    case TriangleRep::LINES:
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINES);
        break;
    default:
        return;
    }

    glBegin(GL_TRIANGLES);
    for(int i=0; i<3; ++i)
    {
        color(iTriangle.color(i)[0], iTriangle.color(i)[1], iTriangle.color(i)[2]);
        glVertex3r(iTriangle.vertex(i)[0], iTriangle.vertex(i)[1], iTriangle.vertex(i)[2]);
    }

    glEnd();



    glPopAttrib();

}

void GLRenderer::visitOrientedLineRep(OrientedLineRep& iLine) 
{
    switch(iLine.style())
    {
    case LineRep::LINES:
        glBegin(GL_LINES);
        break;
    case LineRep::LINE_STRIP:
        glBegin(GL_LINE_STRIP);
        break;
    case LineRep::LINE_LOOP:
        glBegin(GL_LINE_LOOP);
        break;
    default:
        return;
    }

    int i=0;
    int ncolor = iLine.getId().getFirst()%3;

    const vector<Vec3r>& vertices = iLine.vertices();
    float step=1.f/vertices.size();
    vector<Vec3r>::const_iterator v;
    for(v=vertices.begin(); v!=vertices.end(); v++)
    {
        switch(ncolor)
        {
        case 0:
            color(i*step,0.f,0.f);
            break;
        case 1:
            color(0.f, i*step, 0.f);
            break;
        case 2:
            color(0.f, 0.f, i*step);
            break;
        default:
            color(i*step, i*step,i*step);
            break;
        }
        i++;
        glVertex3r((*v)[0], (*v)[1], (*v)[2]);
    }

    glEnd();
}

void GLRenderer::visitVertexRep( VertexRep& iVertex) 
{
    if(iVertex.pointSize() != 0.f)
        glPointSize(iVertex.pointSize());

    glBegin(GL_POINTS);
    glVertex3r(iVertex.x(), iVertex.y(), iVertex.z());
    glEnd();

    //  printf("drawig point at %f %f %f\n", iVertex.x(), iVertex.y(), iVertex.z() );
}

void GLRenderer::visitDrawingStyle(DrawingStyle& iDrawingStyle) 
{
    // Drawing Style management
    switch(iDrawingStyle.style())
    {
    case DrawingStyle::FILLED:
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glShadeModel(GL_SMOOTH);
        //   _overrideColors = false;
        break;

    case DrawingStyle::LINES:
        glDisable(GL_BLEND);
        glDisable(GL_LINE_SMOOTH);
        glShadeModel(GL_FLAT);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glLineWidth(iDrawingStyle.lineWidth());
        //    _overrideColors = true;
        //glColor3ub(0,0,0);
        break;

    case DrawingStyle::POINTS:
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_BLEND);
        glEnable(GL_POINT_SMOOTH);
        glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
        glPointSize(iDrawingStyle.pointSize());
        _overrideColors = true;
        //   glColor3ub(0,0,0);
        break;

    case DrawingStyle::INVISIBLE:
        glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
        glDepthMask(0);
        _overrideColors = true;
        break;

    default:
        break;
    }

    glLineWidth(iDrawingStyle.lineWidth());
    glPointSize(iDrawingStyle.pointSize());

    // FIXME
    if(true == iDrawingStyle.lightingEnabled())
        glEnable(GL_LIGHTING);
    else
        glDisable(GL_LIGHTING);
}

void GLRenderer::visitMaterial(Material& m) {
    const float* diff = m.diffuse();
    const float* amb = m.ambient();
    const float* spec = m.specular();
    const float* em = m.emission();

    GLenum rm = orientableSurfaces ? GL_FRONT : GL_FRONT_AND_BACK;

    RenderColor(diff);
    glMaterialfv(rm, GL_AMBIENT, amb);
    glMaterialfv(rm, GL_DIFFUSE, diff);
    glMaterialfv(rm, GL_SPECULAR, spec);
    glMaterialfv(rm, GL_EMISSION, em);
    glMaterialf(rm, GL_SHININESS, m.shininess());
}

void GLRenderer::visitMaterial(const Material& m) {
    const float* diff = m.diffuse();
    const float* amb = m.ambient();
    const float* spec = m.specular();
    const float* em = m.emission();

    GLenum rm = orientableSurfaces ? GL_FRONT : GL_FRONT_AND_BACK;

    RenderColor(diff);
    glMaterialfv(rm, GL_AMBIENT, amb);
    glMaterialfv(rm, GL_DIFFUSE, diff);
    glMaterialfv(rm, GL_SPECULAR, spec);
    glMaterialfv(rm, GL_EMISSION, em);
    glMaterialf(rm, GL_SHININESS, m.shininess());
}
void GLRenderer::applyTransform( const Matrix44r &iMatrix)  
{
    double m[16];
    for(int lign=0; lign<4; lign++)
        for(int column=0; column<4; column++)
            m[column*4+lign] = iMatrix(lign, column);

    glMultMatrixr(m);
}

void GLRenderer::RenderColor( const float *rgba)  
{
    if (!_overrideColors)
        glColor4fv(rgba);
} 

/*
void GLRenderer::SetupRendering()
{
  push attrib bits

  if (_renderStyle == NO_STYLE)
    return;

  if (_renderStyle == SHADED_STYLE)
    {

      // setup smooth shading here


    }

  if (_renderStyle == LINE_STYLE || _renderStyle == FRONTBACK_STYLE)
    {

      // setup flat shading here


    }
}

void GLRenderer::PostRendering()
{

  pop bits

*/
