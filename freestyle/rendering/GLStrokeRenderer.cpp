
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

#include "GLUtils.h"
#include <QImage>
#include <QFileInfo>
#include <qgl.h>
#include <QFile>
#include "GLStrokeRenderer.h"


#ifdef WIN32
# include "extgl.h"
#endif // WIN32

//AARON: UNCOMMENTED THIS LINE IN ORDER TO MAKE IT COMPILE
//#define glBlendEquation(x)

extern int outputWidth, outputHeight; // temporarily here for debugging

GLStrokeRenderer::GLStrokeRenderer()
:StrokeRenderer()
{
  _textureManager = new GLTextureManager;
}

GLStrokeRenderer::~GLStrokeRenderer()
{
  if(0 != _textureManager)
  {
    delete _textureManager;
    _textureManager = 0;
  }
}

float initialColor(float x, float avTex=0.5)
{
  float y=(1-x)/avTex;
  return (y>1 ? 1 : y);
}
//float complementColor(float x, float avTex=0.5)
//{
//  float y=(1-x)/avTex-1;
//  return (y<0 ? 0 : y);
//}

float complementColor(float x, float avTex=0.5)
{
  float y=(1-x);///avTex-1;
  return (y<0 ? 0 : y);
}

void GLStrokeRenderer::RenderStrokeRep(StrokeRep *iStrokeRep) const
{
  CHECK_FOR_ERROR;

  glPushAttrib(GL_COLOR_BUFFER_BIT);
  Stroke::MediumType strokeType = iStrokeRep->getMediumType();
  // int averageTextureAlpha=0.5; //default value
  // if (strokeType==OIL_STROKE)
    // averageTextureAlpha=0.75; 
  // if (strokeType>=NO_BLEND_STROKE)
    // averageTextureAlpha=1.0; 
  // if (strokeType<0)
    // {
      // renderNoTexture(iStrokeRep);
      // return;
    // }
  int i;
  glDisable(GL_CULL_FACE);
  glDisable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glShadeModel(GL_SMOOTH);
  glDisable(GL_DEPTH_TEST);
    
  glEnable(GL_BLEND);
  //  glDisable(GL_BLEND);

  if(strokeType==Stroke::DRY_MEDIUM)
    {
      glBlendEquation(GL_MIN);  //changed this from GL_MAX in order to work for a black background image instead of white
    }
  else if(strokeType==Stroke::OPAQUE_MEDIUM || strokeType==Stroke::DRY_MEDIUM)
    {
      glBlendEquation(GL_FUNC_ADD);
      //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

      //      glBlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_REVERSE_SUBTRACT);
      glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA,
			  GL_ONE_MINUS_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
  else
    {
            glBlendEquation(GL_FUNC_ADD);
      //      glBlendFunc(GL_SRC_ALPHA, GL_ONE);

      //      glBlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_SUBTRACT);
      glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE,
			  GL_ONE_MINUS_SRC_ALPHA, GL_ONE);

      //      printf("using opaque\n");
    }


  //  glBlendFunc(GL_ONE, GL_ZERO);

  //  glDisable(GL_TEXTURE_2D);
  glEnable(GL_BLEND);
  //glDisable(GL_BLEND);
  //  glDisable(GL_TEXTURE_1D);
  glEnable(GL_TEXTURE_2D);


  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
  //glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

  //first texture, basically the only one for lighter strokes
  glBindTexture(GL_TEXTURE_2D, iStrokeRep->getTextureId()); 
  //glBindTexture(GL_TEXTURE_2D, _textureManager.getPaperTextureIndex()); 


  // TODO:   READ BACK PIXEL TRANSFER, PIXEL MAP, MATRICES, ETC.
  /*
  // loop over transfer functions and print out glIsEnabled()
  GLenum funcs[] = { GL_TEXTURE_1D,
                     GL_INDEX_LOGIC_OP, 
                     GL_COLOR_LOGIC_OP,
		     GL_COLOR_MATERIAL,
		     GL_COLOR_SUM,
		     GL_COLOR_TABLE,
		     GL_CONVOLUTION_1D,
		     GL_CONVOLUTION_2D,
		     GL_INDEX_LOGIC_OP,
		     GL_MAP1_COLOR_4,
		     GL_MAP1_INDEX,
		     GL_MAP1_TEXTURE_COORD_1,
		     GL_MAP1_TEXTURE_COORD_2,
		     GL_MAP1_TEXTURE_COORD_3,
		     GL_MAP1_TEXTURE_COORD_4,
		     GL_MAP2_INDEX,
		     GL_POST_COLOR_MATRIX_COLOR_TABLE,
		     GL_POST_CONVOLUTION_COLOR_TABLE,
		     GL_SEPARABLE_2D };
  for(int i =0; i<17;i++)
    printf("[(%d) %0X: %s]\n", i, funcs[i], (glIsEnabled(funcs[i]) ? "ENABLED" : "disabled"));

  
  GLfloat cmat[16];
  glGetFloatv(GL_COLOR_MATRIX, cmat);
  printf("color matrix: %f %f %f %f; %f %f %f %f; %f %f %f %f; %f %f %f %f\n",
	 cmat[0], cmat[1], cmat[2], cmat[3], cmat[4],cmat[5], cmat[6], cmat[7], cmat[8],
	 cmat[9], cmat[10], cmat[11], cmat[12], cmat[13], cmat[14], cmat[15]);*/


  //      static bool printed = false;

  //      if (!printed)
  //{
	  //	  printf("first point:(%f, %f) ",svRep->point2d()[0], svRep->point2d()[1]);
	  /*
	  printf("glstrokerenderer: (outputWidth = %d, outputHeight = %d)\n", outputWidth, outputHeight);
	  double matrix[16];
	  glGetDoublev(GL_MODELVIEW_MATRIX, matrix);
	  printf("Modelview= [%f %f %f %f; %f %f %f %f; %f %f %f %f; %f %f %f %f]'\n", 
		 matrix[0], matrix[1], matrix[2], matrix[3],matrix[4],matrix[5],matrix[6],matrix[7],matrix[8],matrix[9], 
		 matrix[10],matrix[11],matrix[12],matrix[13],matrix[14],matrix[15]);  
	  glGetDoublev(GL_PROJECTION_MATRIX, matrix);
	  printf("Projection= [%f %f %f %f; %f %f %f %f; %f %f %f %f; %f %f %f %f]'\n", 
		 matrix[0], matrix[1], matrix[2], matrix[3],matrix[4],matrix[5],matrix[6],matrix[7],matrix[8],matrix[9], 
		 matrix[10],matrix[11],matrix[12],matrix[13],matrix[14],matrix[15]);

	  double viewp[4];
	  glGetDoublev(GL_VIEWPORT, viewp);
	  printf("GL1: Viewport = [%f %f %f %f]\n", viewp[0], viewp[1], viewp[2], viewp[3]);
	  */



	  //	  printed = true;
  //	}
      /*
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_BLEND);

  glColor4f(1,.5,.2,.5);
  glBegin(GL_TRIANGLES);
  glVertex2f(0,0);
  glVertex2f(100,50);
  glVertex2f(50,100);
  glEnd();
  
  glEnable(GL_TEXTURE_2D);
  glEnable(GL_BLEND);
      */

      //  glViewport(0,0,outputWidth, outputHeight);
  
  vector<Strip*>& strips = iStrokeRep->getStrips();
  for(vector<Strip*>::iterator s=strips.begin(), send=strips.end();
  s!=send;
  ++s){
    Strip::vertex_container& vertices = (*s)->vertices();



    glBegin(GL_TRIANGLE_STRIP);
    for(Strip::vertex_container::iterator v=vertices.begin(), vend=vertices.end();
    v!=vend;
    ++v){
      StrokeVertexRep * svRep = (*v);
      Vec3r color = svRep->color();
      real alpha = svRep->alpha();
      glColor4f(complementColor(color[0]), complementColor(color[1]), 
      		complementColor(color[2]), alpha);
      //      glColor4f(1,0,0,1);
      
      static bool printed2 = false;
      /*
      if (!printed2)
	{
	  printf("first point: (%f, %f)\n", svRep->point2d()[0], svRep->point2d()[1]);
	  printed2 =true;
	}
      */

      glTexCoord2f(svRep->texCoord()[0],svRep->texCoord()[1] );
      glVertex2f(svRep->point2d()[0], svRep->point2d()[1]);
    }  
    glEnd();
  }
//  if (strokeType>=NO_BLEND_STROKE) return;
  //  //return;
  //
  //  //second texture, the complement, for stronger strokes
  //  glBindTexture(GL_TEXTURE_2D, _textureManager.getTextureIndex(2*strokeType+1)); 
  //  glBegin(GL_TRIANGLE_STRIP);
  //  for(i=0; i<_sizeStrip; i++)
  //  {
  //    glColor4f(complementColor(_color[i][0]), 
  //      complementColor(_color[i][1]), 
  //      complementColor(_color[i][2]), _alpha[i]);
  //    glTexCoord2f(_texCoord[i][0],_texCoord[i][1] );
  //    glVertex2f(_vertex[i][0], _vertex[i][1]);
  //  }
  //  glEnd();

  /*
	  double viewp[4];
	  glGetDoublev(GL_VIEWPORT, viewp);
	  printf("GL2: Viewport = [%f %f %f %f]\n", viewp[0], viewp[1], viewp[2], viewp[3]);
  */

  glPopAttrib();

  CHECK_FOR_ERROR;
}

void GLStrokeRenderer::RenderStrokeRepBasic(StrokeRep *iStrokeRep) const
{
  CHECK_FOR_ERROR;

  glPushAttrib(GL_COLOR_BUFFER_BIT);
  Stroke::MediumType strokeType = iStrokeRep->getMediumType();
  int i;
  glDisable(GL_CULL_FACE);
  glDisable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glShadeModel(GL_SMOOTH);
  glDisable(GL_DEPTH_TEST);
    
  glEnable(GL_BLEND);

  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_TEXTURE_2D);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
  //glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

  //first texture, basically the only one for lighter strokes
  glBindTexture(GL_TEXTURE_2D, iStrokeRep->getTextureId()); 
  //glBindTexture(GL_TEXTURE_2D, _textureManager.getPaperTextureIndex()); 
  
  vector<Strip*>& strips = iStrokeRep->getStrips();
  for(vector<Strip*>::iterator s=strips.begin(), send=strips.end();
  s!=send;
  ++s){
    Strip::vertex_container& vertices = (*s)->vertices();
    glBegin(GL_TRIANGLE_STRIP);
    for(Strip::vertex_container::iterator v=vertices.begin(), vend=vertices.end();
    v!=vend;
    ++v){
      StrokeVertexRep * svRep = (*v);
      Vec3r color = svRep->color();
      real alpha = svRep->alpha();
      glColor4f(color[0], 
              color[1], 
              color[2], alpha);
      glTexCoord2f(svRep->texCoord()[0],svRep->texCoord()[1] );
      glVertex2f(svRep->point2d()[0], svRep->point2d()[1]);
    }  
    glEnd();
  }
  glPopAttrib();

  CHECK_FOR_ERROR;
}

//No Texture
//void GLStrokeRenderer::renderNoTexture(StrokeRep *iStrokeRep) const
//{
//  Stroke::MediumType strokeType = iStrokeRep->getMediumType();
//  int sizeStrip = iStrokeRep->sizeStrip();
//  const Vec3r *color = iStrokeRep->colors();
//  const Vec2r *vertex = iStrokeRep->vertices();
//  const float *alpha = iStrokeRep->alpha();
//  
//  glDisable(GL_LIGHTING);
//  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//  glShadeModel(GL_SMOOTH);
//  glDisable(GL_DEPTH_TEST);
//
//  //if (strokeType==NO_TEXTURE_STROKE)
//  if(strokeType < 0)
//    {
//      glDisable(GL_BLEND);
//      glDisable(GL_TEXTURE_2D);
//      glBegin(GL_TRIANGLE_STRIP);
//      for(int i=0; i<sizeStrip; i++)
//	{ 
//	  glColor4f(complementColor(color[i][0]), 
//		    complementColor(color[i][1]), 
//		    complementColor(color[i][2]), alpha[i]);
//	  glVertex2f(vertex[i][0], vertex[i][1]);
//	} 
//      glEnd();
//    } 
//  else
//    {
//      //#ifdef WIN32
//      //glBlendEquation(GL_FUNC_REVERSE_SUBTRACT);
//      glBlendEquation(GL_ADD);
//      //#endif
//      glEnable(GL_BLEND);
//      glBlendFunc(GL_SRC_ALPHA, GL_ONE);
//      glDisable(GL_TEXTURE_2D);
//      glBegin(GL_TRIANGLE_STRIP);
//      for(int i=0; i<sizeStrip; i++)
//	{ 
//	  glColor4f(complementColor(color[i][0]), 
//		    complementColor(color[i][1]), 
//		    complementColor(color[i][2]), alpha[i]);
//	  glVertex2f(vertex[i][0], vertex[i][1]);
//	} 
//      glEnd();
//    }
//  //	cerr<<"color="<<_color[1][0]<<", "<<_color[1][1]<<", "<<_color[1][2]<<") "<<endl;
//	
//	
//}


/**********************************/
/*                                */
/*                                */
/*         GLTextureManager         */
/*                                */
/*                                */
/**********************************/

//#define TEXTURES_DIR ROOT_DIR "/data/textures"

GLTextureManager::GLTextureManager ()
: TextureManager()
{
  //_brushes_path = Config::getInstance()...
}

GLTextureManager::~GLTextureManager ()
{
}

void
GLTextureManager::loadPapers ()
{
  unsigned size = _papertextures.size();
  _papertexname = new unsigned[size];
  GLuint *tmp = new GLuint[size];
  glGenTextures(size, tmp);
  for(int i=0;i<size;++i){
    _papertexname[i] = tmp[i];
  }
  delete [] tmp;

  // Papers textures
  cout << "Loading papers textures..." << endl;

  for (unsigned i = 0; i < size; i++)
    preparePaper(_papertextures[i].c_str(), _papertexname[i]);

  cout << "Done." << endl << endl;
}

void GLTextureManager::loadStandardBrushes()
{
  //  getBrushTextureIndex(TEXTURES_DIR "/brushes/charcoalAlpha.bmp", Stroke::HUMID_MEDIUM);
  //  getBrushTextureIndex(TEXTURES_DIR "/brushes/washbrushAlpha.bmp", Stroke::HUMID_MEDIUM);
  //  getBrushTextureIndex(TEXTURES_DIR "/brushes/oil.bmp", Stroke::HUMID_MEDIUM);
  //  getBrushTextureIndex(TEXTURES_DIR "/brushes/oilnoblend.bmp", Stroke::HUMID_MEDIUM);
  //  getBrushTextureIndex(TEXTURES_DIR "/brushes/charcoalAlpha.bmp", Stroke::DRY_MEDIUM);
  //  getBrushTextureIndex(TEXTURES_DIR "/brushes/washbrushAlpha.bmp", Stroke::DRY_MEDIUM);
  //  getBrushTextureIndex(TEXTURES_DIR "/brushes/opaqueDryBrushAlpha.bmp", Stroke::OPAQUE_MEDIUM);
  //  getBrushTextureIndex(TEXTURES_DIR "/brushes/opaqueBrushAlpha.bmp", Stroke::OPAQUE_MEDIUM);
  _defaultTextureId = getBrushTextureIndex("smoothAlpha.bmp", Stroke::OPAQUE_MEDIUM);
}


unsigned
GLTextureManager::loadBrush(string sname, Stroke::MediumType mediumType)
{
  GLuint texId;
  glGenTextures(1, &texId);
  bool found = false;
  vector<string> pathnames;
  QString path;
  StringUtils::getPathName(TextureManager::Options::getBrushesPath(),
  sname,
  pathnames);
  for (vector<string>::const_iterator j = pathnames.begin(); j != pathnames.end(); j++) {
    path = j->c_str();
    if(QFile::exists(path)){
      found = true;
      break;
    }
  }
  if(!found)
    return 0;
  // Brush texture
  cout << "Loading brush texture..." << endl;
  switch(mediumType){
  case Stroke::DRY_MEDIUM:
    prepareTextureLuminance(path.toStdString(), texId);
    break;
  case Stroke::HUMID_MEDIUM:
  case Stroke::OPAQUE_MEDIUM:
  default:
    prepareTextureAlpha(path.toStdString(), texId);
    break;
  }
  cout << "Done." << endl << endl;

  return texId;
}

bool 
GLTextureManager::prepareTextureAlpha (string sname, GLuint itexname)
{
  const char * name = sname.c_str();
  QImage qim(name);
  QFileInfo fi(name);
  QString filename = fi.fileName();
  if (qim.isNull()) 
    {
      cerr << "  Error: unable to read \"" << name << "\"" << endl;
      return false;
    }
  if (qim.depth()>8)
    {
      cerr<<"  Error: \""<< name <<"\" has "<<qim.depth()<<" bits/pixel"<<endl;
      return false;
    }
  //		qim=QGLWidget::convertToGLFormat( qimOri );

  glBindTexture(GL_TEXTURE_2D, itexname);
  //glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
		  GL_LINEAR);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
  //	      GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
		  GL_LINEAR);     

  glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, qim.width(), qim.height(), 0, 
	       GL_ALPHA, GL_UNSIGNED_BYTE, qim.bits());	

  cout << "  \"" << filename.toStdString() << "\" loaded with "<< qim.depth() << " bits per pixel" << endl;

  return true;

}

bool 
GLTextureManager::prepareTextureLuminance (string sname, GLuint itexname)
{
  const char * name = sname.c_str();
  QImage qim(name);
  QFileInfo fi(name);
  QString filename = fi.fileName();
  if (qim.isNull()) 
    {
      cerr << "  Error: unable to read \"" << name << "\"" << endl;
      return false;
    }
  if (qim.depth() > 8)
    {
      cerr<<"  Error: \""<<name<<"\" has "<<qim.depth()<<" bits/pixel"<<endl;
      return false;
    }

  glBindTexture(GL_TEXTURE_2D, itexname);
  //glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
		  GL_LINEAR);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
  //	      GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
		  GL_LINEAR);     

  glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, qim.width(), qim.height(), 0, 
	       GL_LUMINANCE, GL_UNSIGNED_BYTE, qim.bits());	

  cout << "  \"" << filename.toStdString() << "\" loaded with "<< qim.depth() << " bits per pixel" << endl;

  return true;

}

bool 
GLTextureManager::prepareTextureLuminanceAndAlpha (string sname, GLuint itexname)
{
  const char * name = sname.c_str();
  QImage qim(name);
  QFileInfo fi(name);
  QString filename = fi.fileName();
  if (qim.isNull()) 
    {
      cerr << "  Error: unable to read \"" << name << "\"" << endl;
      return false;
    }
  if (qim.depth() > 8)
    {
      cerr<<"  Error: \""<<name<<"\" has "<<qim.depth()<<" bits/pixel"<<endl;
      return false;
    }
					   
  glBindTexture(GL_TEXTURE_2D, itexname);
  //glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
					     
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
		  GL_LINEAR);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
  //	      GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
		  GL_LINEAR);     
						     
  glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, qim.width(), qim.height(), 0, 
	       GL_LUMINANCE, GL_UNSIGNED_BYTE, qim.bits());	
  glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, qim.width(), qim.height(), 0, 
	       GL_ALPHA, GL_UNSIGNED_BYTE, qim.bits());	
							 
  cout << "  \"" << filename.toStdString() << "\" loaded with "<< qim.depth() << " bits per pixel" << endl;						   

  return true;
							     
}

bool 
GLTextureManager::preparePaper (const char *name, GLuint itexname)
{
  QImage qim(name);
  QFileInfo fi(name);
  QString filename = fi.fileName();
  if (qim.isNull()) 
    {
      cerr << "  Error: unable to read \"" << name << "\"" << endl;
      return false;
    }
  if (qim.depth()!=32)
    {
      cerr<<"  Error: \""<<name<<"\" has "<<qim.depth()<<" bits/pixel"<<endl;
      return false;
    }
  QImage qim2=QGLWidget::convertToGLFormat( qim );

  glBindTexture(GL_TEXTURE_2D, itexname);
	
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
		  GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
		  GL_LINEAR);     

  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, qim.width(), qim.height(), 0, 
	       GL_RGBA, GL_UNSIGNED_BYTE, qim2.bits());	

  cout << "  \"" << filename.toStdString() << "\" loaded with "<< qim.depth() << " bits per pixel" << endl;

  return true;
}

