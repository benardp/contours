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

#ifndef GLUTILS_H
#define GLUTILS_H

#include "../system/FreestyleConfig.h"

int LIB_RENDERING_EXPORT isExtensionSupported(const char *extension);
void LIB_RENDERING_EXPORT *glutils_extgl_GetProcAddress(const char *name);

# include <stdio.h>

# ifdef __MACH__
#  include <OpenGL/gl.h>
#  include <OpenGL/glu.h>
# else
#  include <GL/glew.h>
#  include <GL/gl.h>
#  include <GL/glu.h>
# endif

#ifndef CHECK_FOR_ERROR
#  ifdef QT_NO_DEBUG      // NDEBUG
#     define CHECK_FOR_ERROR   ;
#  else
#     define CHECK_FOR_ERROR   checkForError(__FILE__, __LINE__, __func__);
#  endif
#endif

inline
GLenum checkForError(const char *file, int line, const char *func)
{
  GLenum errCode;
  const GLubyte *errString;

  if ((errCode = glGetError()) != GL_NO_ERROR)
    {
      errString = gluErrorString(errCode);
      printf("OpenGL error: %s. %s() at %s:%d",errString,func, file,line);

      //      if (loc != NULL)
      //      	printf(" (%s)",loc);
      
      printf("\n");
    }

  return errCode;
}

#endif
