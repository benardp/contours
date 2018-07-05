
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

#include <Python.h>
#include <QApplication>
#include "../rendering/GLUtils.h"
#include "Controller.h"
#include "AppMainWindow.h"
#include "AppConfig.h"
#include "AppGLWidget.h"
#include "Run.h"

// Global
Controller	*g_pController;

bool useCameraFromRIB = false;
bool offscreenRendering = false;
double RIB_cameraModelview[16];
double RIB_cameraProjection[16];
int output_viewport_x, output_viewport_y, output_viewport_width, output_viewport_height;
int window_viewport_x, window_viewport_y, window_viewport_width, window_viewport_height;
double RIB_camera_center[3];
double RIB_znear, RIB_zfar;
int windowWidth, windowHeight;
int outputWidth, outputHeight;
extern bool orientableSurfaces;

vector<const char*> styleNames;


struct RIFDebugPoint
{
    DebugPoint::PointType ptType;
    Vec3r pos;
    char * debugString;
    double radialCurvature;
};

vector<RIFDebugPoint> RIFdebugPoints;

void setupCamera(float top, float bottom, float left, float right,
                 float pixelaspect, float aspectratio,
                 float near, float far, float focalLength,
                 Matrix4x4 worldTransform)
{

    printf("worldTransform= ");
    worldTransform.print();


    double wt[16] = {
        worldTransform.data[0][0], worldTransform.data[1][0], worldTransform.data[2][0], worldTransform.data[3][0],
        worldTransform.data[0][1], worldTransform.data[1][1], worldTransform.data[2][1], worldTransform.data[3][1],
        worldTransform.data[0][2], worldTransform.data[1][2], worldTransform.data[2][2], worldTransform.data[3][2],
        worldTransform.data[0][3], worldTransform.data[1][3], worldTransform.data[2][3], worldTransform.data[3][3],
    };


    // compute R^T * t   (assumes the upper-left component is orthonormal (i.e., rotation or reflection)
    for(int i=0;i<3;i++)
        RIB_camera_center[i] =
                -worldTransform.data[0][i] * worldTransform.data[0][3] +
                -worldTransform.data[1][i] * worldTransform.data[1][3] +
                -worldTransform.data[2][i] * worldTransform.data[2][3];

    printf("camera center = %f %f %f\n", RIB_camera_center[0],
           RIB_camera_center[1], RIB_camera_center[2]);
    //  view->resize(xres,yres);

    useCameraFromRIB = true;

    printf("wt = [%f %f %f %f; %f %f %f %f; %f %f %f %f; %f %f %f %f]'\n",
           wt[0], wt[1], wt[2], wt[3],wt[4],wt[5],wt[6],wt[7],wt[8],wt[9],
           wt[10],wt[11],wt[12],wt[13],wt[14],wt[15]);

    /*
  double proj[16] = { 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, (near + far) / (far - near), 1/focalLength,
              0, 0, (-2*near * far) / (far - near), 0};
*/

    double proj[16] = { focalLength, 0, 0, 0,
                        0, focalLength, 0, 0,
                        0, 0, far/        (far - near), 1,
                        0, 0, -near * far/(far - near), 0 };

    printf("proj = [%f %f %f %f; %f %f %f %f; %f %f %f %f; %f %f %f %f]'\n",
           proj[0], proj[1], proj[2], proj[3],proj[4],proj[5],proj[6],proj[7],proj[8],proj[9],
           proj[10],proj[11],proj[12],proj[13],proj[14],proj[15]);

    for(int i=0;i<16;i++)
        RIB_cameraModelview[i] = wt[i];

    for(int j=0;j<16;j++)
        RIB_cameraProjection[j] = proj[j];

    // setup viewport transform for output image (offscreen buffer)
    output_viewport_width = 2 * outputWidth / (right - left);
    output_viewport_x = -output_viewport_width * (left + 1) / 2;// (right-left);

    output_viewport_height = 2 * outputHeight / (top - bottom);
    output_viewport_y = -output_viewport_height * (bottom + 1) / 2;//(top-bottom);

    printf("output dimensions = %d, %d\n", outputWidth, outputHeight);

    // setup viewport transform for window
    window_viewport_width = 2 * windowWidth / (right - left);
    window_viewport_x = -window_viewport_width * (left + 1) /2; //(right-left);

    window_viewport_height = 2 * windowHeight / (top - bottom);
    window_viewport_y = -window_viewport_height * (bottom + 1) /2; //(top-bottom);

    RIB_znear = near;
    RIB_zfar = far;
}

GLuint color_tex_FBO;
GLuint fbo;
GLuint depth_rb;

void setupFBOrendering()
{
    glGenTextures(1, &color_tex_FBO);
    glBindTexture(GL_TEXTURE_2D, color_tex_FBO);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    //NULL means reserve texture memory, but texels are undefined
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, outputWidth, outputHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);

    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    //Attach 2D texture to this FBO
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color_tex_FBO, 0);

    glGenRenderbuffers(1, &depth_rb);
    glBindRenderbuffer(GL_RENDERBUFFER, depth_rb);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, outputWidth, outputHeight);

    //Attach depth buffer to FBO
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_rb);

    //Does the GPU support current FBO configuration?
    GLenum status;
    status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    switch(status)
    {
    case GL_FRAMEBUFFER_COMPLETE:
        break;
    default:
        printf("error: FBO configuration not handled, status: 0x%X\n",status);
        exit(1);
    }

    //and now you can render to GL_TEXTURE_2D
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    offscreenRendering = true;
}

void saveFBOandCleanup(const char * filename, bool save)
{
    // read the FBO into an array

    printf("creating temporary buffer FBO\n");
    GLubyte * pixels = new GLubyte [outputWidth*outputHeight*4];
    //  GLubyte * alpha = new GLubyte [outputWidth*outputHeight];

    if (pixels == NULL)// || alpha == NULL)
    {
        printf("ERROR ALLOCATING TEMPORARY BUFFER OF SIZE\n");
        exit(1);
    }

    printf("reading pixels\n");

    glFlush();
    glReadPixels(0, 0, outputWidth, outputHeight, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
    //  glReadPixels(0, 0, outputWidth, outputHeight, GL_ALPHA, GL_UNSIGNED_BYTE, alpha);
    glFlush();

    printf("swapping buffers\n");

    // set it back to render to the back-buffer as normal
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    printf("Deleting resources\n");

    //Delete resources
    glDeleteTextures(1, &color_tex_FBO);
    glDeleteRenderbuffers(1, &depth_rb);

    //Bind 0, which means render to back buffer, as a result, fb is unbound
    glDeleteFramebuffers(1, &fbo);

    glDrawBuffer(GL_FRONT);
    glDrawPixels(outputWidth, outputHeight, GL_RGBA, GL_UNSIGNED_BYTE, pixels);

    glDrawBuffer(GL_BACK);

    glFlush();

    if(save){
        // save it to a file

        printf("Creating image buffer\n");

        QImage im(pixels, outputWidth, outputHeight, QImage::Format_ARGB32);

        printf("mirroring, converting RGBA->ARGB, and saving\n");

        // mirror the image vertically and then save it

        QImage im2 = im.mirrored(false,true);

        // fix up RGBA values
        for(int x=0;x<im2.width();x++)
            for(int y=0;y<im2.height();y++)
            {
                QRgb v = im2.pixel(x,y);
                QRgb v2 = qRgba(qBlue(v), qGreen(v), qRed(v), qAlpha(v));
                //	QRgb v2 = qRgba(qAlpha(v), qRed(v), qGreen(v), qBlue(v));
                im2.setPixel(x,y,v2);
            }

        bool res = im2.save(filename,"TIFF");    //  im.save("test.jpg","JPG");

        delete [] pixels;

        if (res)
        {
            printf("Saved:\n");
            printf("\tsos %s\n",filename);
        }
        else
            printf("ERROR saving file \"%s\".\n",filename);
    }

    CHECK_FOR_ERROR;

    offscreenRendering = false;
}


void addStyleFS(const char *styleFilename)
{
    styleNames.push_back(styleFilename);
}

void clearStylesFS()
{
    styleNames.clear();
}

QApplication *app = NULL;
AppMainWindow *mainWindow = NULL;

void run(const char * meshFilename, const char * snapshotFilename, const char * outputEPSPolyline, const char * outputEPSThick,
         Matrix4x4 worldTransform,
         float left, float right, float bottom, float top,
         float pixelaspect, float aspectratio,
         float near, float far, float focalLength,
         int outputWidthArg, int outputHeightArg,
         int windowWidthArg, int windowHeightArg,
         //	 float fov,
         int visAlgorithm, bool useConsistency, bool runInteractive,  double cuspTrimThreshold, double graftThreshold, double wiggleFactor,
         const char * pythonLibPath, bool saveLayers)
{
    outputWidth = outputWidthArg;
    outputHeight  = outputHeightArg;

    windowWidth = windowWidthArg + 28; // eye-balled estimates of the dimensions of the window decorations
    windowHeight = windowHeightArg + 70;

    orientableSurfaces = true; //orientable;


    printf("Inputs: %f %f %f %f    %f %f %f %f   %d %d %d %d %d\n",
           left,right,bottom,top,pixelaspect,aspectratio,near,far,
           outputWidthArg,outputHeightArg,windowHeightArg,windowHeightArg,visAlgorithm);
    worldTransform.print();


    if (runInteractive)
    {
        outputWidth = windowWidthArg;
        outputHeight = windowHeightArg;
    }

    char windowName[500];
    sprintf(windowName,"Freestyle: %s", meshFilename);

    if (app == NULL)
    {
        int argc = 1;
        char * argv[] = { windowName } ;
        //      char ** argv = new char*[1];
        //      argv[0] = strdup("Freestyle");


        // sets the paths
        QApplication::setColorSpec(QApplication::ManyColor);
        app = new QApplication(argc, argv);
        Q_INIT_RESOURCE(freestyle);

        Config::Path pathconfig;

        QGLFormat myformat;
        myformat.setAlpha(true);
        QGLFormat::setDefaultFormat( myformat );

        mainWindow = new AppMainWindow(NULL, windowName);

        g_pController = new Controller;
        g_pController->SetMainWindow(mainWindow);
        g_pController->SetView(mainWindow->pQGLWidget);

        mainWindow->setGeometry(50,50,windowWidth, windowHeight); // on the Mac, having the window higher causes it to fall under the taskbar
        mainWindow->show();

        CHECK_FOR_ERROR;
    }
    else
    {
        // delete the old data from the controller
        g_pController->CloseFile();
        g_pController->Clear();  // clears the canvas and removes style modules

        CHECK_FOR_ERROR;
    }

    printf("after init: ");
    g_pController->printRowCount();


    //  windowWidth = outputWidth;
    //  windowHeight = outputHeight;

    CHECK_FOR_ERROR;

    if (pythonLibPath != NULL && strlen(pythonLibPath) > 0)
    {
        char * cmd = new char[25+strlen(pythonLibPath)];
        sprintf(cmd,"sys.path.append('%s')",pythonLibPath);

        printf("EXECUTING %s\n", cmd);

        g_pController->interpreter()->interpretCmd("import sys");
        g_pController->interpreter()->interpretCmd(cmd);

        delete[] cmd;
    }


    //  g_pController->InsertStyleModule(0, styleFilename); // should the first arg be 1?
    //  printf("Num styles = %d\n", styleNames.size());
    //  for(vector<char*>::iterator it = styleNames.begin(); it!=styleNames.end(); it++)
    //      printf("\t%s\n", *it);

    int i=0;
    for(vector<const char*>::iterator it = styleNames.begin(); it!=styleNames.end(); it++)
    {
        g_pController->AddStyleModule(*it);
        g_pController->toggleLayer(i++, true);
    }

    printf("after styles added: ");
    g_pController->printRowCount();

    //  double wiggleFactor = 0; //0.001;   // amount to fix Sanjay problems: 0.005.  0.01 is too much and 0.0001 is too little. this causes crashes on other frames.

    printf("Wiggle Factor = %f\n", wiggleFactor);

    g_pController->Load3DSFile(meshFilename,wiggleFactor);

    setupCamera( top,  bottom,  left,  right, pixelaspect,  aspectratio, near,  far, focalLength, worldTransform);

    CHECK_FOR_ERROR;

    ViewMapBuilder::visibility_algo va;

    switch(visAlgorithm)
    {
    case 0: va = ViewMapBuilder::ray_casting; break;
    case 1: va = ViewMapBuilder::region_based; break;
    case 2: va = ViewMapBuilder::punch_out; break;
    default: printf("Invalid visibility algorithm specified\n"); exit(1);
    }

    g_pController->setVisibilityAlgo( va, useConsistency );

    g_pController->SetCuspTrimThreshold(cuspTrimThreshold);
    g_pController->SetGraftThreshold(graftThreshold);

    g_pController->ComputeViewMap();


    if (runInteractive)
        for(vector<RIFDebugPoint>::iterator it = RIFdebugPoints.begin(); it != RIFdebugPoints.end(); ++it)
            g_pController->addRIFDebugPoint((*it).ptType,(*it).pos, (*it).debugString, (*it).radialCurvature);// (*it).ndotv,

    CHECK_FOR_ERROR;

    AppGLWidget * view = g_pController->view();

    if (view->draw3DsceneEnabled())
        view->toggle3D();

    CHECK_FOR_ERROR;

    if (!view->draw2DsceneEnabled())
        view->toggle2D();

    CHECK_FOR_ERROR;

    // printf("Rendering to FBO\n");
    // setupFBOrendering();

    CHECK_FOR_ERROR;

    // draw it on the image

    g_pController->DrawStrokes();

    CHECK_FOR_ERROR;


    // saveFBOandCleanup(snapshotFilename,!saveLayers);

    // CHECK_FOR_ERROR;

    if(saveLayers){
        g_pController->savePSLayers(outputEPSPolyline, true, 2);
        g_pController->savePSLayers(outputEPSThick, false, 0);
    }else{
        g_pController->savePSSnapshot(outputEPSPolyline, true, 2);
        printf("\topen %s\n",outputEPSPolyline);
        g_pController->savePSSnapshot(outputEPSThick, false, 0);
        printf("\topen %s\n",outputEPSThick);
        QString svgName = QString(outputEPSPolyline);
        svgName.chop(3);
        svgName.append("svg");
        g_pController->SaveSVG(qPrintable(svgName), true, 1);
    }

    if (runInteractive){        
        app->exec();
    }
}






// routines for interfacing with the RIF

void run2(const char * meshFilename, const char * snapshotFilename, const char * svgFilename, const char * psFilename,
          float worldTransform[16],
          float left, float right, float bottom, float top,
          float pixelaspect, float aspectratio,
          float near, float far, float focalLength,
          int outputWidthArg, int outputHeightArg,
          int windowWidthArg, int windowHeightArg,
          int visAlgorithm, bool useConsistency, bool runInteractive,
          double cuspTrimThreshold, double graftThreshold,double wiggleFactor,
          const char * pythonLibPath, bool saveLayers)
{
    Matrix4x4 worldTransformMatrix
            (worldTransform[0],worldTransform[1],worldTransform[2],worldTransform[3],
             worldTransform[4],worldTransform[5],worldTransform[6],worldTransform[7],
             worldTransform[8],worldTransform[9],worldTransform[10],worldTransform[11],
             worldTransform[12],worldTransform[13],worldTransform[14],worldTransform[15]);


    run(meshFilename,snapshotFilename,svgFilename,psFilename, worldTransformMatrix,left,right,bottom,top,pixelaspect,
        aspectratio,near,far,focalLength,outputWidthArg,outputHeightArg,windowWidthArg,windowHeightArg,
        visAlgorithm,useConsistency, runInteractive,cuspTrimThreshold, graftThreshold, wiggleFactor, pythonLibPath,saveLayers);

}


// type: 0 is front-facing, 1 is back-facing
void addRIFDebugPoint(int type, double x, double y, double z, char * debugString, double radialCurvature)
//		      bool rootFindingFailed, bool degenerate)
{
    DebugPoint::PointType pt;

    switch(type)
    {
    case 0: pt = DebugPoint::FRONT_FACING_VERTEX;break;

    case 1: pt = DebugPoint::BACK_FACING_VERTEX;break;

    case 2: pt = DebugPoint::CONTOUR_VERTEX;break;

    case 3: pt = DebugPoint::CUSP; break;

    default:
        pt = DebugPoint::ERROR; break;
    }

    //  printf("input string: %s\n", debugString);

    RIFDebugPoint t;
    t.pos = Vec3r(x,y,z);
    t.ptType = pt;
    t.debugString = strdup(debugString);
    t.radialCurvature = radialCurvature;
    //  t.ndotv = ndotv;
    //  t.rootFindingFailed = rootFindingFailed;
    //  t.degenerate = degenerate;

    RIFdebugPoints.push_back(t);
}

/*
void addRIFFacing(int faceNum, int vfint)
{
  RIFfacings[faceNum] = vfint;
}
*/
