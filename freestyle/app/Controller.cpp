
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

// Must be included before any QT header, because of moc
#include "../system/PythonInterpreter.h"

#include <fstream>
#include <float.h>
#include <qfileinfo.h>
#include <qprocess.h>
#include <qstring.h>

#include "AppGLWidget.h"
#include "AppMainWindow.h"
#include "AppProgressBar.h"
#include "AppStyleWindow.h"
#include "AppOptionsWindow.h"
#include "AppAboutWindow.h"
#include "AppCanvas.h"
#include "AppConfig.h"
#include "AppDensityCurvesWindow.h"

#include "../system/StringUtils.h"
#include "../scene_graph/PLYFileLoader.h"
#include "../scene_graph/NodeShape.h"
#include "../scene_graph/NodeTransform.h"
#include "../scene_graph/NodeLight.h"
#include "../scene_graph/NodeDrawingStyle.h"
#include "../winged_edge/WingedEdgeBuilder.h"
#include "../winged_edge/WEdge.h"
#include "../scene_graph/VertexRep.h"
#include "../winged_edge/WXEdgeBuilder.h"
#include "../scene_graph/ScenePrettyPrinter.h"
#include "../winged_edge/WFillGrid.h"

#include "../view_map/ViewMapTesselator.h"
#include "../stroke/StrokeTesselator.h"
#include "../view_map/ViewMapIO.h"
#include "Controller.h"
#include "../view_map/ViewMap.h"
#include "../winged_edge/Curvature.h"
#include "QGLBasicWidget.h"
#include <qimage.h>
#include "../image/Image.h"
#include "../view_map/SteerableViewMap.h"
#include "../stroke/PSStrokeRenderer.h"
#include "../stroke/SVGStrokeRenderer.h"
#include "../stroke/TextStrokeRenderer.h"
#include "../stroke/StyleModule.h"


extern bool useCameraFromRIB;
extern double RIB_cameraModelview[16];
extern double RIB_cameraProjection[16];
extern int output_viewport_x, output_viewport_y, output_viewport_width, output_viewport_height;
extern int window_viewport_x, window_viewport_y, window_viewport_width, window_viewport_height;
extern double RIB_camera_center[3];
extern double RIB_znear, RIB_zfar;
extern int outputWidth, outputHeight;

//extern NodeGroup * regionDebugNode;
extern NodeGroup * punchOutDebugNode;
extern NodeGroup * visDebugNode;


extern WFace * debugInconsFace;
extern WFace * debugTargetFace;

const int frontFaceColor[3] = {254, 242, 192}; //{ 255, 240, 200 }; // peach
const int backRadialFaceColor[3] =  {7, 137, 233}; //{ 145, 247, 110 };
//const int backFaceColor[3] = { 120, 220, 255 }; // sky blue
const int backFaceColor[3] = {139, 191, 230}; //{ 64, 178, 255 }; // sky blue
const int frontRadialFaceColor[3] = {255, 206, 5}; //{ 63, 175, 131};


unsigned int getName(void*obj);
void resetNames();


int testY = -1;

void printTestY()
{
    printf("testY = %d\n", testY);
}

//------

Controller::Controller()
{
    const QString sep(Config::DIR_SEP.c_str());
    const QString filename = Config::Path::getInstance()->getHomeDir() + sep +
            Config::OPTIONS_DIR + sep + Config::OPTIONS_CURRENT_DIRS_FILE;
    _current_dirs = new ConfigIO(filename, Config::APPLICATION_NAME + "CurrentDirs", true);

    _RootNode = new NodeGroup;
    _RootNode->addRef();

    _SilhouetteNode = NULL;
    _ViewMapVisNode = NULL;
    _ViewMapColorNode = NULL;
    //_ProjectedSilhouette = NULL;
    //_VisibleProjectedSilhouette = NULL;

    _DebugNode = new NodeGroup;
    _DebugNode->addRef();

    _PODebugNode = new NodeGroup;
    _PODebugNode->addRef();

    _winged_edge = NULL;

    _pMainWindow = NULL;
    _pView = NULL;

    _edgeTesselationNature = (Nature::SILHOUETTE | Nature::BORDER | Nature::CREASE);

    _ProgressBar = new AppProgressBar;
    _SceneNumFaces = 0;
    _minEdgeSize = DBL_MAX;
    _bboxDiag = 0;

    _ViewMap = 0;

    _Canvas = 0;

    _cuspTrimThreshold = 0;
    _graftThreshold = 0;
    _VisibilityAlgo = ViewMapBuilder::ray_casting;

    //_VisibilityAlgo = ViewMapBuilder::ray_casting_fast;

    _Canvas = new AppCanvas;

    _inter = new PythonInterpreter;
    _EnableQI = true;
    _ComputeRidges = true;
    _ComputeSteerableViewMap = false;
    _ComputeSuggestive = true;
    _sphereRadius = 1.0;
}

Controller::~Controller()
{
    if(NULL != _RootNode)
    {
        int ref = _RootNode->destroy();
        if(0 == ref)
            delete _RootNode;
    }

    if(NULL != _SilhouetteNode)
    {
        int ref = _SilhouetteNode->destroy();
        if(0 == ref)
            delete _SilhouetteNode;
    }

    if (_ViewMapVisNode != NULL)
    {
        int ref = _ViewMapVisNode->destroy();
        if (ref == 0)
            delete _ViewMapVisNode;
    }

    if (_ViewMapColorNode != NULL)
    {
        int ref = _ViewMapColorNode->destroy();
        if (ref == 0)
            delete _ViewMapColorNode;
    }

    if(NULL != _DebugNode)
    {
        int ref = _DebugNode->destroy();
        if(0 == ref)
            delete _DebugNode;
    }

    if(_PODebugNode != NULL)
    {
        int ref = _PODebugNode->destroy();
        if (ref == 0)
            delete _PODebugNode;
    }

    //  if(NULL != _VisibleProjectedSilhouette)
    //    {
    //      int ref = _VisibleProjectedSilhouette->destroy();
    //      if(0 == ref)
    //	delete _VisibleProjectedSilhouette;
    //    }

    //  if(NULL != _ProjectedSilhouette)
    //    {
    //      int ref = _ProjectedSilhouette->destroy();
    //      if(0 == ref)
    //	delete _ProjectedSilhouette;
    //    }

    if(NULL != _ProgressBar)
    {
        delete _ProgressBar;
        _ProgressBar = NULL;
    }

    if(_winged_edge) {
        delete _winged_edge;
        _winged_edge = NULL;
    }

    if(0 != _ViewMap)
    {
        delete _ViewMap;
        _ViewMap = 0;
    }

    if(0 != _Canvas)
    {
        delete _Canvas;
        _Canvas = 0;
    }

    if (_inter) {
        delete _inter;
        _inter = NULL;
    }

    //  if(_pDensityCurvesWindow){
    //    delete _pDensityCurvesWindow;
    //    _pDensityCurvesWindow = 0;
    //  }
    delete _current_dirs;
}

void Controller::SetView(AppGLWidget *iView)
{
    if(NULL == iView)
        return;

    _pView = iView;
    //_pView2D->setGeometry(_pView->rect());
    _Canvas->SetViewer(_pView);
}

void Controller::SetMainWindow(AppMainWindow *iMainWindow)
{
    _pMainWindow = iMainWindow;
    _ProgressBar->setQTProgressBar(_pMainWindow->qtProgressDialog());
    _pStyleWindow = new AppStyleWindow(_pMainWindow, "StyleWindow");
    _pOptionsWindow = new AppOptionsWindow(_pMainWindow, "MainWindow");
    _pDensityCurvesWindow = new AppDensityCurvesWindow(_pMainWindow, "MainWindow");
}

int Controller::Load3DSFile(const char *iFileName, double wiggleFactor)
{

    if (_pView)
        _pView->setUpdateMode(false);

    //_pMainWindow->InitProgressBar("Loading 3DS Model", 4);
    _ProgressBar->reset();
    _ProgressBar->setLabelText("Loading 3DS Model");
    _ProgressBar->setTotalSteps(3);
    _ProgressBar->setProgress(0);

    //_pMainWindow->setProgressLabel("Reading File");
    //_pMainWindow->setProgressLabel("Cleaning mesh");

    _pMainWindow->DisplayMessage("Reading File");
    _pMainWindow->DisplayMessage("Cleaning Mesh");

    PLYFileLoader sceneLoader(iFileName);

    //_RootNode->AddChild(BuildSceneTest());

    _Chrono.start();

    NodeGroup *maxScene = sceneLoader.Load();

    if (maxScene == NULL) {
        printf("UNABLE TO OPEN 3DS FILE %s\n",iFileName);
        fflush(stdout);
        exit(1);

        //    _ProgressBar->setProgress(3);
        //    return 1;
    }

    printf("Mesh cleaning    : %lf\n", _Chrono.stop());
    fflush(stdout);
    _SceneNumFaces += sceneLoader.numFacesRead();

    if(sceneLoader.minEdgeSize() < _minEdgeSize)
    {
        _minEdgeSize = sceneLoader.minEdgeSize();
        _EPSILON = _minEdgeSize*1e-6;
        if(_EPSILON < DBL_MIN)
            _EPSILON = 0.0;
    }

    cout << "Epsilon computed : " << (double)_EPSILON << endl;

    printf("Faces read: %d, Num faces: %d\n", sceneLoader.numFacesRead(), _SceneNumFaces);

    _ProgressBar->setProgress(1);

    // DEBUG
    //   ScenePrettyPrinter spp;
    //   maxScene->accept(spp);

    // print out the scene graph
    //  maxScene->serialize(0);

    _RootNode->AddChild(maxScene);
    _RootNode->UpdateBBox(); // FIXME: Correct that by making a Renderer to compute the bbox


    _pView->SetModel(_RootNode);
    _pView->FitBBox();

    _pMainWindow->DisplayMessage("Building Winged Edge structure");
    _Chrono.start();

    WXEdgeBuilder wx_builder;
    maxScene->accept(wx_builder);
    _winged_edge = wx_builder.getWingedEdge();


    if (wiggleFactor != 0)
    {
        for(vector<WShape*>::iterator it = _winged_edge->getWShapes().begin();
            it != _winged_edge->getWShapes().end(); ++it)
            for(vector<WVertex*>::iterator vit = (*it)->GetVertexList().begin();
                vit != (*it)->GetVertexList().end(); ++vit)
            {
                Vec3r randvec( wiggleFactor*(drand48() - .5),
                               wiggleFactor*(drand48() - .5),
                               wiggleFactor*(drand48() - .5));
                (*vit)->GetVertex() += randvec;
            }
    }



    /*

  // some debugging
  setupNames();
  printf("Num names = %d\n", _numNames);
  //  assert(_faceNames.find(300) != _faceNames.end());
  debugTargetFace = _faceNames[307];
  debugInconsFace = _faceNames[310];
  //  debugTargetFace = _faceNames[300];
  //  debugInconsFace = _faceNames[309];
  printf("debugTargetFace = %08X\n", debugTargetFace);
  printf("debugInconsFace = %08X\n", debugInconsFace);

  */




    printf("WEdge building   : %lf\n", _Chrono.stop());

    _ProgressBar->setProgress(2);

    _pMainWindow->DisplayMessage("Building Grid");
    _Chrono.start();

    _Grid.clear();
    Vec3r size;
    for(unsigned int i=0; i<3; i++)
    {
        size[i] = fabs(_RootNode->bbox().getMax()[i] - _RootNode->bbox().getMin()[i]);
        size[i] += size[i]/10.0; // let make the grid 1/10 bigger to avoid numerical errors while computing triangles/cells intersections
        if(size[i]==0){
            cout << "Warning: the bbox size is 0 in dimension "<<i<<endl;
        }
    }
    _Grid.configure(Vec3r(_RootNode->bbox().getMin() - size / 20.0), size,
                    _SceneNumFaces);

    // Fill in the grid:
    WFillGrid fillGridRenderer(&_Grid, _winged_edge);
    fillGridRenderer.fillGrid();

    printf("Grid building    : %lf\n", _Chrono.stop());

    // DEBUG
    _Grid.displayDebug();

    _ProgressBar->setProgress(3);

    _pView->SetDebug(_DebugNode);
    _pView->SetPODebug(_PODebugNode);

    //delete stuff
    //  if(0 != ws_builder)
    //    {
    //      delete ws_builder;
    //      ws_builder = 0;
    //    }
    _pView->updateGL();
    QFileInfo qfi(iFileName);
    string basename((const char*)qfi.fileName().toAscii().data());
    _ListOfModels.push_back(basename);

    cout << "Triangles nb     : " << _SceneNumFaces << endl;
    _bboxDiag = (_RootNode->bbox().getMax()-_RootNode->bbox().getMin()).norm();
    cout << "Bounding Box     : " << (double)_bboxDiag << endl;

    return 0;
}



void Controller::CloseFile()
{
    WShape::SetCurrentId(0);
    _pView->DetachModel();
    _ListOfModels.clear();
    if(NULL != _RootNode)
    {
        int ref = _RootNode->destroy();
        if(0 == ref)
            _RootNode->addRef();

        _RootNode->clearBBox();
    }

    _pView->DetachSilhouette();
    if (NULL != _SilhouetteNode)
    {
        int ref = _SilhouetteNode->destroy();
        if(0 == ref)
        {
            delete _SilhouetteNode;
            _SilhouetteNode = NULL;
        }
    }
    if (_ViewMapVisNode != NULL)
    {
        int ref = _ViewMapVisNode->destroy();
        if (ref == 0)
        {
            delete _ViewMapVisNode;
            _ViewMapVisNode = NULL;
        }
    }

    if (_ViewMapColorNode != NULL)
    {
        int ref = _ViewMapColorNode->destroy();
        if (ref == 0)
        {
            delete _ViewMapColorNode;
            _ViewMapColorNode = NULL;
        }
    }

    //  if(NULL != _ProjectedSilhouette)
    //    {
    //      int ref = _ProjectedSilhouette->destroy();
    //      if(0 == ref)
    //	{
    //	  delete _ProjectedSilhouette;
    //	  _ProjectedSilhouette = NULL;
    //	}
    //    }
    //  if(NULL != _VisibleProjectedSilhouette)
    //    {
    //      int ref = _VisibleProjectedSilhouette->destroy();
    //      if(0 == ref)
    //	{
    //	  delete _VisibleProjectedSilhouette;
    //	  _VisibleProjectedSilhouette = NULL;
    //	}
    //  }

    _pView->DetachDebug();
    if(NULL != _DebugNode)
    {
        int ref = _DebugNode->destroy();
        if(0 == ref)
            _DebugNode->addRef();
    }

    _pView->DetachPODebug();
    if (NULL != _PODebugNode)
    {
        int ref = _PODebugNode->destroy();
        if (0 == ref)
            _PODebugNode->addRef();
    }

    if(_winged_edge) {
        delete _winged_edge;
        _winged_edge = NULL;
    }

    Interface1D::printRefStats();

    _ViewMap->checkPointers("deleting");

    // We deallocate the memory:
    if(NULL != _ViewMap)
    {
        delete _ViewMap;
        _ViewMap = 0;
    }

    // clears the canvas
    _Canvas->Erase();

    // clears the grid
    _Grid.clear();
    _SceneNumFaces = 0;
    _minEdgeSize = DBL_MAX;
    //  _pView2D->DetachScene();
    //  if(NULL != _SRoot)
    //  {
    //    int ref = _SRoot->destroy();
    //    if(0 == ref)
    //    {
    //      //_SRoot->addRef();
    //      delete _SRoot;
    //      _SRoot = NULL;
    //    }
    //  }

    // Added by Aaron in hopes of alleviating a memory leak
    Interface1D::printRefStats();
    printf("RESET\n");
    Operators::reset();

    Interface1D::printRefStats();
    Interface1D::eraseAllI1Ds();
    Interface1D::printRefStats();

    resetNames();
}

//  static const streamsize buffer_size = 512 * 1024;

void Controller::SaveViewMapFile(const char *oFileName)
{
    if (!_ViewMap)
        return;

    ofstream ofs(oFileName, ios::binary);
    if (!ofs.is_open()) {
        _pMainWindow->DisplayMessage("Error: Cannot save this file");
        cerr << "Error: Cannot save this file" << endl;
        return;
    }
    //    char buffer[buffer_size];
    //  #if defined(__GNUC__) && (__GNUC__ < 3)
    //    ofs.rdbuf()->setbuf(buffer, buffer_size);
    //  # else
    //    ofs.rdbuf()->pubsetbuf(buffer, buffer_size);
    //  #endif
    _Chrono.start();

    ofs << Config::VIEWMAP_MAGIC.toAscii().data() << endl << Config::VIEWMAP_VERSION.toAscii().data() << endl;

    // Write the models filenames
    ofs << _ListOfModels.size() << endl;
    for (vector<string>::const_iterator i = _ListOfModels.begin(); i != _ListOfModels.end(); i++)
        ofs << *i << "\n";

    // Save the camera position
    float position[3];
    float orientation[4];
    _pView->getCameraState(position, orientation);
    ofs.write((char*)position, 3 * sizeof(*position));
    ofs.write((char*)orientation, 4 * sizeof(*orientation));

    // Write ViewMap
    if (ViewMapIO::save(ofs, _ViewMap, _ProgressBar)) {
        _Chrono.stop();
        cerr << "Error: Cannot save this file" << endl;
        return;
    }

    real d = _Chrono.stop();
    cout << "ViewMap saving   : " << (double)d << endl;
}

void Controller::LoadViewMapFile(const char *iFileName, bool only_camera)
{
    ifstream ifs(iFileName, ios::binary);
    if (!ifs.is_open()) {
        _pMainWindow->DisplayMessage("Error: Cannot load this file");
        cerr << "Error: Cannot load this file" << endl;
        return;
    }
    //    char buffer[buffer_size];
    //  #if defined(__GNUC__) && (__GNUC__ < 3)
    //    ifs.rdbuf()->setbuf(buffer, buffer_size);
    //  # else
    //    ifs.rdbuf()->pubsetbuf(buffer, buffer_size);
    //  #endif

    // Test File Magic and version
    char tmp_buffer[256];
    QString test;

    ifs.getline(tmp_buffer, 255);
    test = tmp_buffer;
    if (test != Config::VIEWMAP_MAGIC) {
        _pMainWindow->DisplayMessage(
                    (QString("Error: This is not a valid .") + Config::VIEWMAP_EXTENSION + QString(" file")).toAscii().data());
        cerr << "Error: This is not a valid ." << Config::VIEWMAP_EXTENSION.toAscii().data() << " file" << endl;
        return;
    }
    ifs.getline(tmp_buffer, 255);
    test = tmp_buffer;
    if (test != Config::VIEWMAP_VERSION && !only_camera) {
        _pMainWindow->DisplayMessage(
                    (QString("Error: This version of the .") + Config::VIEWMAP_EXTENSION + QString(" file format is no longer supported")).toAscii().data());
        cerr << "Error: This version of the ." << Config::VIEWMAP_EXTENSION.toAscii().data() << " file format is no longer supported" << endl;
        return;
    }

    // Read the models filenames and open them (if not already done)
    string tmp;
    vector<string> tmp_vec;
    unsigned models_nb, i;

    ifs.getline(tmp_buffer, 255);
    models_nb = atoi(tmp_buffer);
    for (i = 0; i < models_nb; i++) {
        ifs.getline(tmp_buffer, 255);
        tmp = tmp_buffer;
        tmp_vec.push_back(tmp);
    }
    if (_ListOfModels != tmp_vec && !only_camera) {
        CloseFile();
        vector<string> pathnames;
        int err = 0;
        for (vector<string>::const_iterator i = tmp_vec.begin(); i != tmp_vec.end(); i++)
        {
            pathnames.clear();
            StringUtils::getPathName(ViewMapIO::Options::getModelsPath(), *i, pathnames);
            for (vector<string>::const_iterator j = pathnames.begin(); j != pathnames.end(); j++)
                if (!(err = Load3DSFile(j->c_str())))
                    break;
            if (err) {
                _pMainWindow->DisplayMessage("Error: cannot find the right model(s)");
                cerr << "Error: cannot find model \"" << *i << "\" - check the path in the Options" << endl;
                return;
            }
        }
    }

    // Set the camera position
    float position[3];
    float orientation[4];
    ifs.read((char*)position, 3 * sizeof(*position));
    ifs.read((char*)orientation, 4 * sizeof(*orientation));
    _pView->setCameraState(position, orientation);
    _pView->saveCameraState();

    if (only_camera) {
        _pMainWindow->DisplayMessage("Camera parameters loaded");
        return;
    }

    // Reset ViewMap
    if(NULL != _ViewMap)
    {
        delete _ViewMap;
        _ViewMap = 0;
    }
    _pView->DetachSilhouette();
    if (NULL != _SilhouetteNode)
    {
        int ref = _SilhouetteNode->destroy();
        if(0 == ref)
            delete _SilhouetteNode;
    }
    if (NULL != _ViewMapVisNode)
    {
        int ref = _ViewMapVisNode->destroy();
        if(0 == ref)
            delete _ViewMapVisNode;
    }
    if (NULL != _ViewMapColorNode)
    {
        int ref = _ViewMapColorNode->destroy();
        if(0 == ref)
            delete _ViewMapColorNode;
    }
    //  if(NULL != _ProjectedSilhouette)
    //    {
    //      int ref = _ProjectedSilhouette->destroy();
    //      if(0 == ref)
    //	delete _ProjectedSilhouette;
    //    }
    //  if(NULL != _VisibleProjectedSilhouette)
    //    {
    //      int ref = _VisibleProjectedSilhouette->destroy();
    //      if(0 == ref)
    //	{
    //	  delete _VisibleProjectedSilhouette;
    //	  _VisibleProjectedSilhouette = 0;
    //	}
    // }
    _ViewMap = new ViewMap();

    // Read ViewMap
    _Chrono.start();
    if (ViewMapIO::load(ifs, _ViewMap, _ProgressBar)) {
        _Chrono.stop();
        _pMainWindow->DisplayMessage(
                    (QString("Error: This is not a valid .") + Config::VIEWMAP_EXTENSION + QString(" file")).toAscii().data());
        cerr << "Error: This is not a valid ." << Config::VIEWMAP_EXTENSION.toAscii().data() << " file" << endl;
        return;
    }

    // Update display
    _pMainWindow->DisplayMessage("Updating display");
    ViewMapTesselator3D sTesselator3d;
    //ViewMapTesselator2D sTesselator2d;
    //sTesselator2d.SetNature(_edgeTesselationNature);
    sTesselator3d.SetNature(_edgeTesselationNature); // this doesn't do anything
    sTesselator3d.SetShowVisibleOnly(true);
    sTesselator3d.SetColoring(ViewMapTesselator3D::ID_COLOR);

    // Tesselate the 3D edges:
    _SilhouetteNode = sTesselator3d.Tesselate(_ViewMap);
    _SilhouetteNode->addRef();

    // Tesselate 2D edges
    //  _ProjectedSilhouette = sTesselator2d.Tesselate(_ViewMap);
    //  _ProjectedSilhouette->addRef();
    //
    _pView->AddSilhouette(_SilhouetteNode);
    //_pView->Add2DSilhouette(_ProjectedSilhouette);

    printf("LOADING IS OUTDATED BECAUSE IT DOESN'T CREATE THE VISUALIZATIONS THE SAME AS NORMAL\n");


    // Update options window
    _pOptionsWindow->updateViewMapFormat();

    real d = _Chrono.stop();
    cout << "ViewMap loading  : " << (double)d << endl;

    // Compute the Directional ViewMap:
    if(_ComputeSteerableViewMap){
        ComputeSteerableViewMap();
    }

    // Reset Style modules modification flags
    resetModified(true);
}


void Controller::ComputeViewMap()
{
    if (!_ListOfModels.size())
        return;

    if(NULL != _ViewMap)
    {
        delete _ViewMap;
        _ViewMap = 0;
    }

    _pView->DetachDebug();
    if(NULL != _DebugNode)
    {
        int ref = _DebugNode->destroy();
        if(0 == ref)
            _DebugNode->addRef();
    }

    _pView->DetachPODebug();
    if (NULL != _PODebugNode)
    {
        int ref = _PODebugNode->destroy();
        if (0 == ref)
            _PODebugNode->addRef();
    }

    _pView->DetachSilhouette();
    if (NULL != _SilhouetteNode)
    {
        int ref = _SilhouetteNode->destroy();
        if(0 == ref)
            delete _SilhouetteNode;
    }


    // reset the region IDs and region edges
    ViewMapBuilder::ResetGroupingData(*_winged_edge);


    // retrieve the 3D viewpoint and transformations information
    //----------------------------------------------------------
    // Save the viewpoint context at the view level in order
    // to be able to restore it later:
    _pView->saveCameraState();

    // Restore the context of view:
    // we need to perform all these operations while the
    // 3D context is on.
    _pView->Set3DContext();
    float src[3] = { 0, 0, 0 };
    float vp_tmp[3];
    real mv[4][4];
    real proj[4][4];
    int viewport[4];
    real focalLength;
    float znear, zfar;
    float fovy_radian;
    float aspect;

    if (useCameraFromRIB)
    {
        vp_tmp[0] = RIB_camera_center[0];
        vp_tmp[1] = RIB_camera_center[1];
        vp_tmp[2] = RIB_camera_center[2];

        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
            {
                // note: it seems that these are transposed of the normal layout?
                mv[i][j] = RIB_cameraModelview[i*4+j];
                proj[i][j] = RIB_cameraProjection[i*4+j];
            }

        viewport[0] = output_viewport_x;
        viewport[1] = output_viewport_y;
        viewport[2] = output_viewport_width;
        viewport[3] = output_viewport_height;

        focalLength = -1;   // RMAN uses +Z as viewing direction, OpenGL uses -Z

        znear = RIB_znear;
        zfar = RIB_zfar;
        aspect = float(outputWidth)/ outputHeight;
        fovy_radian = M_PI /2;
    }
    else
    {
        // retrieve the projection matrix:
        _pView->RetriveModelViewMatrix((double *)mv);
        _pView->RetrieveProjectionMatrix((double *)proj);
        _pView->camera()->getWorldCoordinatesOf(src, vp_tmp);
        _pView->RetrieveViewport(viewport);
        focalLength = _pView->GetFocalLength();


        aspect = _pView->GetAspect();
        fovy_radian = _pView->GetFovyRadian();
        znear = _pView->znear();
        zfar = _pView->zfar();
    }
    /*
  printf("focal length = %f\n", focalLength);
  printf("mv= [%f %f %f %f; %f %f %f %f; %f %f %f %f; %f %f %f %f]'\n",
     mv[0][0], mv[1][0], mv[2][0], mv[3][0],
     mv[0][1], mv[1][1], mv[2][1], mv[3][1],
     mv[0][2], mv[1][2], mv[2][2], mv[3][2],
     mv[0][3], mv[1][3], mv[2][3], mv[3][3]);
  printf("proj= [%f %f %f %f; %f %f %f %f; %f %f %f %f; %f %f %f %f]'\n",
     proj[0][0], proj[1][0], proj[2][0], proj[3][0],
     proj[0][1], proj[1][1], proj[2][1], proj[3][1],
     proj[0][2], proj[1][2], proj[2][2], proj[3][2],
     proj[0][3], proj[1][3], proj[2][3], proj[3][3]);
  */


    Vec3r vp(vp_tmp[0], vp_tmp[1], vp_tmp[2]);

    // Flag the WXEdge structure for silhouette edge detection:
    //----------------------------------------------------------

    _Chrono.start();
    if (_SceneNumFaces > 2000){
        edgeDetector.SetProgressBar(_ProgressBar);
    }

    edgeDetector.SetViewpoint(Vec3r(vp));
    edgeDetector.enableRidgesAndValleysFlag(_ComputeRidges);
    edgeDetector.enableSuggestiveContours(_ComputeSuggestive);
    edgeDetector.setSphereRadius(_sphereRadius);
    edgeDetector.setSuggestiveContourKrDerivativeEpsilon(_suggestiveContourKrDerivativeEpsilon);
    edgeDetector.setUseConsistency(_useConsistency);
    edgeDetector.processShapes(*_winged_edge);

    // after this step, a bunch of faces have flagged facelayers attached with "ta" and "tb" values
    // indicating where smooth edges lie

    real duration = _Chrono.stop();
    printf("Feature lines    : %lf\n", duration);


    // Builds the view map structure from the flagged WSEdge structure:
    //----------------------------------------------------------
    ViewMapBuilder vmBuilder;
    vmBuilder.SetProgressBar(_ProgressBar);
    vmBuilder.SetEnableQI(_EnableQI);
    vmBuilder.SetViewpoint(Vec3r(vp));

    vmBuilder.SetTransform(mv, proj, viewport, focalLength, aspect, fovy_radian);
    vmBuilder.SetFrustum(znear, zfar);

    vmBuilder.SetGrid(&_Grid);

    vmBuilder.SetUseConsistency(_useConsistency);

    vmBuilder.SetCuspTrimThreshold(_cuspTrimThreshold);
    vmBuilder.SetGraftThreshold(_graftThreshold);

    // Builds a tesselated form of the silhouette for display purpose:
    // (Not sure this is still used)
    //---------------------------------------------------------------
    ViewMapTesselator3D sTesselator3d;
    //ViewMapTesselator2D sTesselator2d;
    //sTesselator2d.SetNature(_edgeTesselationNature);
    sTesselator3d.SetNature(_edgeTesselationNature);
    
    _Chrono.start();
    // Build View Map
    _ViewMap = vmBuilder.BuildViewMap(*_winged_edge, _VisibilityAlgo, _EPSILON);
    _ViewMap->setScene3dBBox(_RootNode->bbox());

    assert(_ViewMap->ViewEdges().size() > 0);

    //Tesselate the 3D edges:
    sTesselator3d.SetShowVisibleOnly(true);
    sTesselator3d.SetColoring(ViewMapTesselator3D::TYPE);
    _SilhouetteNode = sTesselator3d.Tesselate(_ViewMap);
    _SilhouetteNode->addRef();

    sTesselator3d.SetShowVisibleOnly(false);
    sTesselator3d.SetColoring(ViewMapTesselator3D::TYPE);
    _ViewMapVisNode = sTesselator3d.Tesselate(_ViewMap);
    _ViewMapVisNode->addRef();

    sTesselator3d.SetShowVisibleOnly(true);
    sTesselator3d.SetColoring(ViewMapTesselator3D::ID_COLOR);
    _ViewMapColorNode = sTesselator3d.Tesselate(_ViewMap);
    _ViewMapColorNode->addRef();

    // Tesselate 2D edges
    //  _ProjectedSilhouette = sTesselator2d.Tesselate(_ViewMap);
    //  _ProjectedSilhouette->addRef();


    _DebugNode->AddChild(visDebugNode);
    _pView->SetDebug(_DebugNode);

    // generate region debugging vis

    // make a NodeGroup that contains all vertices with unique colors based on region
    if (_VisibilityAlgo == ViewMapBuilder::region_based)
    {
        /*      _PODebugNode->AddChild(regionDebugNode);

      _pView->DetachDebug();
      if(NULL != _DebugNode)
    {
      int ref = _DebugNode->destroy();
      if(0 == ref)
        _DebugNode->addRef();
    }

      // could also just add the new node as a child of the current debug node
      _DebugNode = regionDebugNode;
      _DebugNode->addRef();
      */
        _pView->SetPODebug(_PODebugNode);
    }

    if (_VisibilityAlgo == ViewMapBuilder::punch_out)
    {
        _PODebugNode->AddChild(punchOutDebugNode);
        /*      _pView->DetachDebug();
      if(NULL != _DebugNode)
    {
      int ref = _DebugNode->destroy();
      if(0 == ref)
        _DebugNode->addRef();
    }

      // could also just add the new node as a child of the current debug node
      _DebugNode = punchOutDebugNode;
      _DebugNode->addRef();
      _pView->SetDebug(_DebugNode);
      */
        _pView->SetPODebug(_PODebugNode);
    }


    duration = _Chrono.stop();
    printf("ViewMap building : %lf\n", duration);

    // FIXME DEBUG
    //    vector<ViewVertex*>& vvertices = _ViewMap->ViewVertices();
    //    for(vector<ViewVertex*>::iterator vv=vvertices.begin(), vvend=vvertices.end();
    //    vv!=vvend;
    //    ++vv){
    //      TVertex * tvertex = (*vv)->castToTVertex();
    //      if(!tvertex)
    //        continue;
    //      cout << "TVertex : " << tvertex->getId() << endl;
    //      if (!(tvertex->frontEdgeA().first))
    //        cout << "null FrontEdgeA" << endl;
    //      if (!(tvertex->frontEdgeB().first))
    //        cout << "null FrontEdgeB" << endl;
    //      if (!(tvertex->backEdgeA().first))
    //        cout << "null BackEdgeA" << endl;
    //      if (!(tvertex->backEdgeB().first))
    //        cout << "null backEdgeB" << endl;
    //    }
    //    cout << "-----------" << endl;
    //    vector<SVertex*>& svertices = _ViewMap->SVertices();
    //    unsigned i = 0;
    //    for(vector<SVertex*>::iterator sv = svertices.begin(), svend = svertices.end();
    //        sv != svend && i < 10;
    //        ++sv, ++i) {
    //      cout << "SVertex - Id : " << (*sv)->getId() << endl;
    //      cout << "SVertex - P3D : " << (*sv)->point3D() << endl;
    //      cout << "SVertex - P2D : " << (*sv)->point2D() << endl;
    //      set<Vec3r>::const_iterator i;
    //      unsigned tmp;
    //      for (i = (*sv)->normals().begin(), tmp = 0;
    // 	  i != (*sv)->normals().end();
    // 	  i++, tmp++);
    //      cout << "SVertex - Normals : " << tmp << endl;
    //      cout << "SVertex - FEdges : " << (*sv)->fedges().size() << endl;
    //    }
    //    cout << "-----------" << endl;
    //    vector<FEdge*>& fedges = _ViewMap->FEdges();
    //    for(vector<FEdge*>::iterator fe = fedges.begin(), feend = fedges.end();
    //        fe != feend && i < 10;
    //        ++fe, ++i) {
    //      cout << "FEdge - Id: " << (*fe)->getId() << endl;
    //      cout << "FEdge - Occl: " << (*fe)->getOccludeeIntersection() << endl;
    //    }
    //    cout << "-----------" << endl;
    // END DEBUG

    // FIXME GLDEBUG
    //====================================================================
    // CUSPS
    //=======
    //  vector<ViewEdge*>& vedges = _ViewMap->ViewEdges();
    //  //typedef ViewEdgeInternal::fedge_iterator_base<Nonconst_traits<FEdge*> > fedge_iterator;
    //  //fedge_iterator fit = vedges[0]->fedge_iterator_begin();
    //  for(vector<ViewEdge*>::iterator ve=vedges.begin(), veend=vedges.end();
    //  ve!=veend;
    //  ++ve){
    //    if((!((*ve)->getNature() & Nature::SILHOUETTE)) || (!((*ve)->fedgeA()->isSmooth())))
    //      continue;
    //    FEdge *fe = (*ve)->fedgeA();
    //    FEdge * fefirst = fe;
    //    //ViewEdge::fedge_iterator fit = (*ve)->fedge_iterator_begin();
    //    //ViewEdge::vertex_iterator vit = (*ve)->vertices_begin();
    //
    //    Material mat;
    //    //    for(; !(fe.end()); ++fe){
    //    bool first = true;
    //    bool front = true;
    //    bool positive = true;
    //    do{
    //      FEdgeSmooth * fes = dynamic_cast<FEdgeSmooth*>(fe);
    //      Vec3r A((fes)->vertexA()->point3d());
    //      Vec3r B((fes)->vertexB()->point3d());
    //      Vec3r AB(B-A);
    //      AB.normalize();
    //      LineRep * lrep = new LineRep(A,B);
    //      silhouetteDebugShape->AddRep(lrep);
    //      Vec3r m((A+B)/2.0);
    //      Vec3r crossP(AB^(fes)->normal());
    //      crossP.normalize();
    //      Vec3r viewvector(m-vp);
    //      viewvector.normalize();
    //      if(first){
    //        if(((crossP)*(viewvector)) > 0)
    //          positive = true;
    //        else
    //          positive = false;
    //        first = false;
    //      }
    //      if(positive){
    //        if(((crossP)*(viewvector)) < -0.2)
    //          positive = false;
    //      }else{
    //        if(((crossP)*(viewvector)) > 0.2)
    //          positive = true;
    //      }
    //      if(positive)
    //        mat.SetDiffuse(1,1,0,1);
    //      else
    //        mat.SetDiffuse(1,0,0,1);
    //      lrep->SetMaterial(mat);
    //      fe = fe->nextEdge();
    //    }while((fe!=0) && (fe!=fefirst));
    //  }
    //====================================================================
    // END FIXME GLDEBUG

    _pView->AddSilhouette(_SilhouetteNode);
    _pView->AddViewMapVisNode(_ViewMapVisNode);
    _pView->AddViewMapColorNode(_ViewMapColorNode);
    //_pView->AddSilhouette(_WRoot);
    //_pView->Add2DSilhouette(_ProjectedSilhouette);
    //_pView->Add2DVisibleSilhouette(_VisibleProjectedSilhouette);
    _pView->AddDebug(_DebugNode);

    // Draw the steerable density map:
    //--------------------------------
    if(_ComputeSteerableViewMap){
        ComputeSteerableViewMap();
    }
    // Reset Style modules modification flags
    resetModified(true);
}

void Controller::ComputeSteerableViewMap(){
    if((!_Canvas) || (!_ViewMap))
        return;

    if(_ProgressBar){
        _ProgressBar->reset();
        _ProgressBar->setLabelText("Computing Steerable ViewMap");
        _ProgressBar->setTotalSteps(3);
        _ProgressBar->setProgress(0);
    }
    
    // Build 4 nodes containing the edges in the 4 directions
    NodeGroup *ng[Canvas::NB_STEERABLE_VIEWMAP];
    unsigned i;
    real c = 32.f/255.f; // see SteerableViewMap::readSteerableViewMapPixel() for information about this 32.
    for(i=0; i<Canvas::NB_STEERABLE_VIEWMAP; ++i){
        ng[i] = new NodeGroup;
    }
    NodeShape *completeNS = new NodeShape;
    completeNS->material().SetDiffuse(c,c,c,1);
    ng[Canvas::NB_STEERABLE_VIEWMAP-1]->AddChild(completeNS);
    SteerableViewMap * svm = _Canvas->getSteerableViewMap();
    svm->Reset();

    _pMainWindow->DisplayMessage("Dividing up edges");
    ViewMap::fedges_container& fedges = _ViewMap->FEdges();
    LineRep * fRep;
    NodeShape *ns;
    for(ViewMap::fedges_container::iterator f=fedges.begin(), fend=fedges.end();
        f!=fend;
        ++f){
        if((*f)->viewedge()->qi() != 0)
            continue;
        fRep = new LineRep((*f)->vertexA()->point2d(),(*f)->vertexB()->point2d()) ;
        completeNS->AddRep(fRep); // add to the complete map anyway
        real *oweights = svm->AddFEdge(*f);
        for(i=0; i<Canvas::NB_STEERABLE_VIEWMAP-1; ++i){
            ns = new NodeShape;
            double wc = oweights[i]*c;
            if(oweights[i] == 0)
                continue;
            ns->material().SetDiffuse(wc, wc, wc, 1);
            ns->AddRep(fRep);
            ng[i]->AddChild(ns);
        }
    }
    if(_ProgressBar)
        _ProgressBar->setProgress(1);
    _pMainWindow->DisplayMessage("Rendering Steerable ViewMap");
    GrayImage *img[Canvas::NB_STEERABLE_VIEWMAP];
    //#ifdef WIN32
    QGLBasicWidget offscreenBuffer(_pView, "SteerableViewMap", _pView->width(), _pView->height());
    QPixmap pm;
    QImage qimg;
    for(i=0; i<Canvas::NB_STEERABLE_VIEWMAP; ++i){
        offscreenBuffer.AddNode(ng[i]);
        //img[i] = new GrayImage(_pView->width(), _pView->height());
        //offscreenBuffer.readPixels(0,0,_pView->width(), _pView->height(), img[i]->getArray());
        pm = offscreenBuffer.renderPixmap(_pView->width(), _pView->height());

        if(pm.isNull())
            cout << "BuildViewMap Warning: couldn't render the steerable ViewMap" << endl;
        //pm.save(QString("steerable")+QString::number(i)+QString(".bmp"), "BMP");
        // FIXME!! Lost of time !
        qimg = pm.toImage();
        // FIXME !! again!
        img[i] = new GrayImage(_pView->width(), _pView->height());
        for(unsigned y=0;y<img[i]->height();++y){
            for(unsigned x=0;x<img[i]->width();++x){
                //img[i]->setPixel(x,y,(float)qGray(qimg.pixel(x,y))/255.f);
                img[i]->setPixel(x,y,(float)qGray(qimg.pixel(x,y)));
                //        float c = qGray(qimg.pixel(x,y));
                //        img[i]->setPixel(x,y,qGray(qimg.pixel(x,y)));
            }
        }
        offscreenBuffer.DetachNode(ng[i]);
        ng[i]->destroy();
        delete ng[i];
        // check
        //    qimg = QImage(_pView->width(), _pView->height(), 32);
        //    for(y=0;y<img[i]->height();++y){
        //      for(unsigned x=0;x<img[i]->width();++x){
        //        float v = img[i]->pixel(x,y);
        //        qimg.setPixel(x,y,qRgb(v,v,v));
        //      }
        //    }
        //    qimg.save(QString("newsteerable")+QString::number(i)+QString(".bmp"), "BMP");
    }
    //#else
    // 	// LINUX
    //   QGLBasicWidget offscreenBuffer(_pView, "SteerableViewMap", _pView->width(), _pView->height());

    //   float * buffer = 0;
    //   for(i=0; i<Canvas::NB_STEERABLE_VIEWMAP; ++i){
    //     offscreenBuffer.AddNode(ng[i]);
    // 	offscreenBuffer.draw();
    //     img[i] = new GrayImage(_pView->width(), _pView->height());
    //     buffer = img[i]->getArray();
    //     offscreenBuffer.readPixels(0,0,_pView->width(), _pView->height(), buffer);
    //     for(unsigned y=0;y<img[i]->height();++y){
    //       for(unsigned x=0;x<img[i]->width();++x){
    //         img[i]->setPixel(x,y,255.f *img[i]->pixel(x,y));
    //       }
    //     }

    //     offscreenBuffer.DetachNode(ng[i]);
    //     ng[i]->destroy();
    //     delete ng[i];
    //   }
    // #endif
    if(_ProgressBar)
        _ProgressBar->setProgress(2);
    _pMainWindow->DisplayMessage("Building Gaussian Pyramids");
    svm->buildImagesPyramids(img,false,0,1.f);
    if(_ProgressBar)
        _ProgressBar->setProgress(3);
}

void Controller::saveSteerableViewMapImages(){
    SteerableViewMap * svm = _Canvas->getSteerableViewMap();
    if(!svm){
        cerr << "the Steerable ViewMap has not been computed yet" << endl;
        return;
    }
    svm->saveSteerableViewMap();
}

void Controller::setVisibilityAlgo(ViewMapBuilder::visibility_algo alg, bool useConsistency)
{
    _VisibilityAlgo = alg;
    _useConsistency = useConsistency;
}


void Controller::toggleVisibilityAlgo() 
{
    if (_VisibilityAlgo == ViewMapBuilder::region_based)
    {
        _VisibilityAlgo = ViewMapBuilder::ray_casting;
        _pMainWindow->DisplayMessage("Visibility algorithm switched to \"ray casting\"");
    }
    else if(_VisibilityAlgo == ViewMapBuilder::ray_casting) {
        _VisibilityAlgo = ViewMapBuilder::ray_casting_fast;
        _pMainWindow->DisplayMessage("Visibility algorithm switched to \"fast ray casting\"");
    }
    else if (_VisibilityAlgo == ViewMapBuilder::ray_casting_fast) {
        _VisibilityAlgo = ViewMapBuilder::ray_casting_very_fast;
        _pMainWindow->DisplayMessage("Visibility algorithm switched to \"very fast ray casting\"");
    }
    else {
        _VisibilityAlgo = ViewMapBuilder::region_based;
        _pMainWindow->DisplayMessage("Visibility algorithm switched to \"region-based\"");
    }
}

void Controller::setQuantitativeInvisibility(bool iBool)
{
    _EnableQI = iBool;
}

bool Controller::getQuantitativeInvisibility() const
{
    return _EnableQI;
}

void Controller::setComputeRidgesAndValleysFlag(bool iBool){
    _ComputeRidges = iBool;
}

bool Controller::getComputeRidgesAndValleysFlag() const {
    return _ComputeRidges;
}
void Controller::setComputeSuggestiveContoursFlag(bool b){
    _ComputeSuggestive = b;
}

bool Controller::getComputeSuggestiveContoursFlag() const {
    return _ComputeSuggestive;
}
void Controller::setComputeSteerableViewMapFlag(bool iBool){
    _ComputeSteerableViewMap = iBool;
}

bool Controller::getComputeSteerableViewMapFlag() const {
    return _ComputeSteerableViewMap;
}
void Controller::setFrontBufferFlag(bool iBool)
{
    AppGLWidget::setFrontBufferFlag(iBool);
}

bool Controller::getFrontBufferFlag() const
{
    return AppGLWidget::getFrontBufferFlag();
}

void Controller::setBackBufferFlag(bool iBool)
{
    AppGLWidget::setBackBufferFlag(iBool);
}

bool Controller::getBackBufferFlag() const
{
    return AppGLWidget::getBackBufferFlag();
}

void Controller::DrawStrokes()
{
    if(_ViewMap == 0)
        return;

    _Chrono.start();
    _Canvas->Draw();
    real d = _Chrono.stop();
    cout << "Strokes drawing  : " << (double)d << endl;
    resetModified();
}


void Controller::InsertStyleModule(unsigned index, const char *iFileName)
{
    QFileInfo fi(iFileName);
    QString ext = fi.suffix();
    assert(ext == "py");
    if (ext != "py") {
        cerr << "Error: Cannot load \"" << fi.fileName().toAscii().data()
             << "\", unknown extension" << endl;
        return;
    }
    StyleModule* sm = new StyleModule(iFileName, _inter);
    _Canvas->InsertStyleModule(index, sm);

}

void Controller::AddStyleModule(const char *iFileName)
{
    _pStyleWindow->Add(iFileName);
}

void Controller::RemoveStyleModule(unsigned index)
{
    _Canvas->RemoveStyleModule(index);
}

void Controller::Clear()
{
    _Canvas->Clear();
    _pStyleWindow->clearPlayList();

    //  _pStyleWindow->PlayList->setCurrentCell(0,0);
    //  _pStyleWindow->PlayList->clear();
}

void Controller::ReloadStyleModule(unsigned index, const char * iFileName)
{
    StyleModule* sm = new StyleModule(iFileName, _inter);
    _Canvas->ReplaceStyleModule(index, sm);
}

void Controller::ExposeStyleWindow()
{
    _pStyleWindow->show();
}

void Controller::ExposeOptionsWindow()
{
    _pOptionsWindow->show();
}

void Controller::ExposeHelpWindow()
{
    QStringList cmd_list = _browser_cmd.split(" ");
    for (QStringList::iterator it = cmd_list.begin();
         it != cmd_list.end();
         ++it)
        (*it).replace("%s", _help_index);
    QProcess browser(0);
    QString exe = cmd_list.first();
    cmd_list.removeFirst();
    browser.start(exe, cmd_list);
}

void Controller::ExposeAboutWindow()
{
    AppAboutWindow::display();
}

void Controller::SwapStyleModules(unsigned i1, unsigned i2)
{
    _Canvas->SwapStyleModules(i1, i2);
}


void Controller::toggleLayer(unsigned index, bool iDisplay)
{
    _Canvas->SetVisible(index, iDisplay);
    _pView->updateGL();
}

void Controller::setModified(unsigned index, bool iMod)
{
    _pStyleWindow->setModified(index, iMod);
    _Canvas->setModified(index, iMod);
    updateCausalStyleModules(index + 1);
}

void Controller::updateCausalStyleModules(unsigned index) {
    vector<unsigned> vec;
    _Canvas->causalStyleModules(vec, index);
    for (vector<unsigned>::const_iterator it = vec.begin(); it != vec.end(); it++) {
        _pStyleWindow->setModified(*it, true);
        _Canvas->setModified(*it, true);
    }
}

void Controller::saveSnapshot(bool b) {
    _pView->saveSnapshot(b);
}

void Controller::savePSLayers(const QString& baseName, bool polyline, int polylineWidth){
    for(int i=0; i<_Canvas->getNumStyleModules(); i++){
        QString currentName(baseName);
        QString moduleName(_Canvas->getStyleModule(i)->getFileName().c_str());
        moduleName.chop(3);
        int idx = moduleName.lastIndexOf('/');
        currentName.replace('#',moduleName.right(moduleName.size()-idx-1));
        printf("\topen %s\n",currentName.toAscii().data());
        PSStrokeRenderer psRenderer((const char*)currentName.toAscii().data(), outputWidth, outputHeight, polyline, polylineWidth);
        _Canvas->Canvas::RenderLayer(&psRenderer,i);
        psRenderer.Close();
    }
}

void Controller::savePSSnapshot(const QString& iFileName, bool polyline, int polylineWidth){
    PSStrokeRenderer psRenderer((const char*)iFileName.toAscii().data(), outputWidth, outputHeight, polyline, polylineWidth);
    _Canvas->Canvas::Render(&psRenderer);
    psRenderer.Close();
}

void Controller::saveTextSnapshot(const QString& iFileName){
    TextStrokeRenderer textRenderer((const char*)iFileName.toAscii().data());
    _Canvas->Render(&textRenderer);
    textRenderer.Close();
}

void Controller::SaveSVG(const char * svgFilename, bool polyline, int polylineWidth)
{
    SVGStrokeRenderer svgRenderer( svgFilename, outputWidth, outputHeight, polyline, polylineWidth );
    _Canvas->Canvas::Render(&svgRenderer);
    printf("\topen %s\n", svgFilename);
}

void Controller::captureMovie() {
    _pView->captureMovie();
}

void Controller::resetModified(bool iMod)
{
    _pStyleWindow->resetModified(iMod);
    _Canvas->resetModified(iMod);
}

FEdge* Controller::SelectFEdge(real x, real y)
{
    if (!_ViewMap)
        return NULL;

    FEdge *fedge = (FEdge*)_ViewMap->GetClosestFEdge(x,y);
    ViewEdge *selection = fedge->viewedge();
    _pView->SetSelectedFEdge(fedge);
    _Canvas->SetSelectedFEdge(fedge);
    return fedge;
}

ViewEdge* Controller::SelectViewEdge(real x, real y)
{
    if (!_ViewMap)
        return NULL;

    FEdge *fedge = (FEdge*)_ViewMap->GetClosestFEdge(x,y);
    ViewEdge *selection = fedge->viewedge();
    _pView->SetSelectedFEdge(fedge);
    _Canvas->SetSelectedFEdge(fedge);
    return selection;
}

NodeGroup * Controller::BuildRep(vector<ViewEdge*>::iterator vedges_begin, 
                                 vector<ViewEdge*>::iterator vedges_end)
{
    ViewMapTesselator2D tesselator2D;
    Material mat;
    mat.SetDiffuse(1,1,0.3,1);
    //  tesselator2D.SetMaterial(mat);

    return (tesselator2D.Tesselate(vedges_begin, vedges_end));
}

void Controller::toggleEdgeTesselationNature(Nature::EdgeNature iNature)
{
    _edgeTesselationNature ^= (iNature);
    ComputeViewMap();
}

void		Controller::setModelsDir(const QString& dir) {
    _current_dirs->setValue("models/dir", dir);
}

QString		Controller::getModelsDir() const {
    QString dir = ".";
    _current_dirs->getValue("models/dir", dir);
    return dir;
}

void		Controller::setModulesDir(const QString& dir) {
    _current_dirs->setValue("modules/dir", dir);
}

QString		Controller::getModulesDir() const {
    QString dir = ".";
    _current_dirs->getValue("modules/dir", dir);
    return dir;
}

void		Controller::setPapersDir(const QString& dir) {
    _current_dirs->setValue("papers/dir", dir);
}

QString		Controller::getPapersDir() const {
    QString dir = Config::Path::getInstance()->getPapersDir();
    _current_dirs->getValue("papers/dir", dir);
    return dir;
}

void		Controller::setHelpIndex(const QString& index) {
    _help_index = index;
}

QString		Controller::getHelpIndex() const {
    return _help_index;
}

void		Controller::setBrowserCmd(const QString& cmd) {
    _browser_cmd = cmd;
}

QString		Controller::getBrowserCmd() const {
    return _browser_cmd;
}

void		Controller::resetInterpreter() {
    if (_inter)
        _inter->reset();
}

void Controller::displayMessage(const char * msg, bool persistent){
    _pMainWindow->DisplayMessage(msg, persistent);
}

void Controller::displayDensityCurves(int x, int y){
    SteerableViewMap * svm = _Canvas->getSteerableViewMap();
    if(!svm)
        return;

    unsigned i,j;
    typedef vector<Vec3r> densityCurve;
    vector<densityCurve> curves(svm->getNumberOfOrientations()+1);
    vector<densityCurve> curvesDirection(svm->getNumberOfPyramidLevels());

    // collect the curves values
    unsigned nbCurves = svm->getNumberOfOrientations()+1;
    unsigned nbPoints = svm->getNumberOfPyramidLevels();
    if(!nbPoints)
        return;

    // build the density/nbLevels curves for each orientation
    for(i=0;i<nbCurves; ++i){
        for(j=0; j<nbPoints; ++j){
            curves[i].push_back(Vec3r(j, svm->readSteerableViewMapPixel(i, j, x, y), 0));
        }
    }
    // build the density/nbOrientations curves for each level
    for(i=0;i<nbPoints; ++i){
        for(j=0; j<nbCurves; ++j){
            curvesDirection[i].push_back(Vec3r(j, svm->readSteerableViewMapPixel(j, i, x, y), 0));
        }
    }

    // display the curves
    for(i=0; i<nbCurves; ++i)
        _pDensityCurvesWindow->SetOrientationCurve(i, Vec2d(0,0), Vec2d(nbPoints, 1), curves[i], "scale", "density");
    for(i=1; i<=8; ++i)
        _pDensityCurvesWindow->SetLevelCurve(i, Vec2d(0,0), Vec2d(nbCurves, 1), curvesDirection[i], "orientation", "density");
    _pDensityCurvesWindow->show();
}



void Controller::printRowCount() const 
{ 
    printf("rowCount: %d, currentRow: %d\n", _pStyleWindow->PlayList->rowCount(),
           _pStyleWindow->PlayList->currentRow());

}


void Controller::renderWEdge(bool selectionMode,bool wireframe,DebugVisOptions options)
{
    if (_winged_edge == NULL)
        return;

    //  if (!selectionMode)
    //    {
    //      glDisable(GL_LIGHTING);
    //      glDisable(GL_COLOR_MATERIAL);
    //    }

    if (options.lighting && !wireframe)
    {

        GLfloat pos1[4] = { 0, 0, -1, 0}; // light 0 is on by default, at pos (0,0,1,0)
        //      GLfloat dif1[4] = { .6, .6, .6, 1};
        GLfloat spec[4] = {0,0,0,1};
        GLfloat ambient[4] = { .2, .2, .2, 1};
        GLfloat zero[4] = { 0,0,0,1};
        //      glMatrixMode(GL_MODELVIEW);
        //      glPushMatrix();
        //      glLoadIdentity();
        //      glLightfv(GL_LIGHT1,GL_POSITION,pos1);
        if (options.showWireframe)
        {
            GLfloat difA[4] = { .8, .8, .8, 1};
            glLightfv(GL_LIGHT0,GL_AMBIENT,ambient);
            glLightfv(GL_LIGHT0,GL_DIFFUSE,difA);
        }
        else
        {
            GLfloat difB[4] = { .6, .6, .6, 1};
            glLightfv(GL_LIGHT0,GL_AMBIENT,ambient);
            glLightfv(GL_LIGHT0,GL_DIFFUSE,difB);
        }
        //      glLightfv(GL_LIGHT0,GL_DIFFUSE,dif1);

        // seems to have no effect
        //      glLightfv(GL_LIGHT0,GL_SPECULAR, spec); // disable specular terms
        //      glLightfv(GL_LIGHT1,GL_SPECULAR, spec); // disable specular terms

        //      glPopMatrix();
        //      glEnable(GL_LIGHT1);

        glShadeModel(GL_FLAT);

        glEnable(GL_LIGHTING);
        glEnable(GL_COLOR_MATERIAL);
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    }
    else
    {
        glDisable(GL_LIGHTING);
    }

    glShadeModel(options.nDotVshading ? GL_SMOOTH : GL_FLAT);
    //  glShadeModel(GL_FLAT);

    /*
  glEnable(GL_LIGHT0);
  
  float amb[4] = {1,1,1,1};
  float dif[4] = {1,1,1,1};
  float pos[4] = {1,1,0,1};

  Vec3r viewpoint = SilhouetteGeomEngine::GetViewpoint();
  pos[0] = viewpoint[0];
  pos[1] = viewpoint[0];
  pos[2] = viewpoint[0];

//  glLightfv(GL_LIGHT0, GL_AMBIENT, amb);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, dif);
  glLightfv(GL_LIGHT0, GL_POSITION, pos);
*/


    for(vector<WShape*>::iterator wsit = _winged_edge->getWShapes().begin();
        wsit != _winged_edge->getWShapes().end(); ++wsit)
        for(vector<WFace*>::iterator fit = (*wsit)->GetFaceList().begin();
            fit != (*wsit)->GetFaceList().end(); ++fit)
        {
            WXFace * face = (WXFace*)(*fit);

            if (face->sourcePOB() != NULL) // skip synthesized PO geometry
                continue;

            int vfint = face->GetRIFdata(); // does the RIF think this is a vertex-based front face? 1: front, 2: back, 3: invalid

            // if showmesh is off, then we must be showing only inconsistent.  check if this face is inconsistent.
            if (!options.showMesh && ( (face->front() && vfint == 1) || (!face->front() && vfint == 2)))
                if((options.showRadial && !face->radial()) || !options.showRadial)
                    continue;

            Vec3r v0 = face->GetVertex(0)->GetVertex();
            Vec3r v1 = face->GetVertex(1)->GetVertex();
            Vec3r v2 = face->GetVertex(2)->GetVertex();

            int name=-1;

            if (selectionMode || !wireframe)
                name = getName(face);

            if (!selectionMode && !wireframe)
            {
                if (name == options.selectionName)
                {
                    glColor3f(1,1,0);

                    printf("Triangle: %08X\n", face);
                    printf("Triangle area = %f\n", ((v1-v0)^(v2-v0)).norm()/2);
                    printf("Edge lengths: %f %f %f\n",
                           (v0-v1).norm(), (v2-v1).norm(), (v0-v2).norm());
                    printf("Neighbors: %08X, %08X, %08X\n", face->GetBordingFace(0), face->GetBordingFace(1), face->GetBordingFace(2));
                    printf("Facing: %s,  VBO (RIFfacing): %d (1=front, 2=back, 3=undefined)\n", face->front() ? "FRONT" : "BACK", face->GetRIFdata());
                }
                else
                {
                    if (options.normals){
                        float line_len = 1.f;
                        glColor3f(0.7, 0.7, 0);
                        glBegin(GL_LINES);
                        for(int i=0; i<3; i++){
                            Vec3r v = face->GetVertex(i)->GetVertex();
                            Vec3r n = face->GetVertex(i)->GetNormal();
                            n.normalize();
                            glVertex3f(v[0],v[1],v[2]);
                            glVertex3f(v[0] + line_len * n[0],
                                       v[1] + line_len * n[1],
                                       v[2] + line_len * n[2]);
                        }
                        glEnd();
                    }

                    if (!options.nDotVshading)
                    {
                         if (face->front())
                             if(options.showRadial && face->radial())
                                glColor3ub(frontRadialFaceColor[0], frontRadialFaceColor[1], frontRadialFaceColor[2]);
                             else
                                glColor3ub(frontFaceColor[0], frontFaceColor[1], frontFaceColor[2]);
                        else
                             if(options.showRadial && face->radial())
                                glColor3ub(backRadialFaceColor[0], backRadialFaceColor[1], backRadialFaceColor[2]);
                             else
                                glColor3ub(backFaceColor[0], backFaceColor[1], backFaceColor[2]);

                        /*

          if (vfint == 3 && options.showInconsistentFaces)   // RIF couldn't determine
            glColor3f(.1,.1,.1);
          else
                      if (face->front() && (vfint == 1 || !options.showInconsistentFaces)) // front-face
              glColor3ub(frontFaceColor[0], frontFaceColor[1], frontFaceColor[2]);
            //		      glColor3ub(255,229,180); // peach
              //		      glColor3f(1,.5,1); // magenta
            else
                        if (!face->front() && ( vfint == 2 || !options.showInconsistentFaces)) //back-face
            glColor3ub(backFaceColor[0], backFaceColor[1], backFaceColor[2]);
            //			glColor3ub(178,255,255); // sky blue
            //			glColor3ub(191,255,0); // lime green
            //			glColor3f(0,.5,1); // back-face
              else
            if (!face->front() && vfint == 1) // inconsistent face
              glColor3f(1,0,.25);
            else
              if (face->front() && vfint ==2) // inconsistent face
                glColor3f(0,0,1);
              else
                glColor3f(0,0,0); // shouldn't ever get to this case
*/
                    }
                }
            }

            if (!selectionMode && wireframe)
            {
                if (!options.showInconsistentFaces ||
                        ((face->front() && vfint == 1) ||
                        (!face->front() && vfint == 2)))
                {
                    if (!options.showWireframe)
                        continue;

                    glColor3ub(65,65,65);
                    //                glColor3f(.1,.1,.1);
                    glLineWidth(1);
                }
                else
                {
                    std::vector<WVertex*> vertices;
                    face->RetrieveVertexList(vertices);
                    bool onContour = false;
                    for(std::vector<WVertex*>::iterator vit=vertices.begin(); vit!=vertices.end() && !onContour; vit++){
                        WVertex* v = (*vit);
                        std::vector<WEdge*> edges = v->GetEdges();
                        for(std::vector<WEdge*>::iterator eit=edges.begin(); eit!=edges.end(); eit++){
                            WEdge* e = (*eit);
                            if((e->GetaFace() && e->GetaFace()->GetRIFdata() != face->GetRIFdata()) ||
                                    (e->GetbFace() && e->GetbFace()->GetRIFdata() != face->GetRIFdata())){
                                onContour = true;
                                break;
                            }
                        }
                    }

                    glLineWidth(6);
//                    if(onContour)
//                        glColor3f(.4,.1,.4);
//                    else
                        glColor3f(.8,.1,.8);
                    //                switch(vfint)
                    //                {
                    //                case 3:  glColor3f(.1,.1,.1); break;
                    //                case 1:  glColor3f(1,0,.25); break;
                    //                case 2:  glColor3f(0,0,1); break;
                    //                default: glColor3f(1,1,1); break; // shouldn't get here
                    //                }
                }
            }

            if (selectionMode)
                glPushName(name);

            Vec3f faceNormal = face->GetNormal();

            float ndotv1 = face->GetVertex(0)->GetSurfaceNdotV();
            float ndotv2 = face->GetVertex(1)->GetSurfaceNdotV();
            float ndotv3 = face->GetVertex(2)->GetSurfaceNdotV();

            //	float scaleFac = 1;
            float rho = 1.0;

            glBegin(GL_TRIANGLES);

            glNormal3f(faceNormal[0],faceNormal[1],faceNormal[2]);

            if (options.nDotVshading && name != options.selectionName){
                //	  if (ndotv1 > rho)
                //	    glColor3f(1,0,0);
                //	  else
                if (ndotv1 > 0)
                    glColor3f(ndotv1/rho,0,0);
                else
                    glColor3f(0,0,-ndotv1/rho);

            //	glNormal3f(n1[0],n1[1],n1[2]);
            }

            glVertex3f(v0[0],v0[1],v0[2]);

            if (options.nDotVshading  && name != options.selectionName){
                //	  if (ndotv2 > rho)
                //	    glColor3f(1,0,0);
                //	  else
                if (ndotv2 > 0)
                    glColor3f(ndotv2/rho,0,0);
                else
                    glColor3f(0,0,-ndotv2/rho);
            }

            //	glNormal3f(n2[0],n2[1],n2[2]);
            glVertex3f(v1[0],v1[1],v1[2]);

            if (options.nDotVshading  && name != options.selectionName){
                if (ndotv3 > 0)
                    glColor3f(ndotv3/rho,0,0);
                else
                    glColor3f(0,0,-ndotv3/rho);
            }

            //	glNormal3f(n3[0],n3[1],n3[2]);
            glVertex3f(v2[0],v2[1],v2[2]);

            glEnd();

//            if (!selectionMode && options.showRadial){

//                for(int i=0; i<3; i++){
//                    //                    if(!(face->front(true) ^ ((WXFace*)face->GetBordingFace(i))->front(true)))
//                    //                        continue;

//                    //                    for(int j=1; j<=2; j++){
//                    Vec3r normal = face->GetVertexNormal(i);
//                    Vec3r pos = face->GetVertex(i)->GetVertex();

//                    Vec3r cameraCenter = Vec3r(RIB_camera_center[0],RIB_camera_center[1],RIB_camera_center[2]);
//                    Vec3r viewVec = cameraCenter - pos;
//                    Vec3r planeNormal = viewVec ^ normal;
//                    planeNormal.normalize();

//                    for(int j=1; j<=2; j++){
//                        Vec3r pos2 = face->GetVertex((i+j)%3)->GetVertex();
//                        real d = planeNormal * (pos2 - pos);
//                        // printf("d = %f\n",d);
//                        if(fabs(d)<0.005){
//                            glLineWidth(6);
//                            glColor3f(0.98f,0.57f,0.57f);
//                            glBegin(GL_LINES);
//                            glVertex3f(pos[0],pos[1],pos[2]);
//                            glVertex3f(pos2[0],pos2[1],pos2[2]);
//                            glEnd();
//                        }
//                    }
//                }
//            }
            //            }

            if (selectionMode)
                glPopName();
        }

    glDisable(GL_LIGHTING);

    if (!selectionMode)
    {
        glEnable(GL_LIGHTING);
        glEnable(GL_COLOR_MATERIAL);
        glShadeModel(GL_SMOOTH);
    }
}



void Controller::printSelected(GLuint selection)
{
    printf("Selected name: %d\n", selection);
}


void Controller::render(bool selectionMode, DebugVisOptions options)
{
    if (_ViewMap != NULL)
        _ViewMap->render3D(selectionMode,options);


    if (options.showMesh || options.showInconsistentFaces)
    {
        if (selectionMode)
            renderWEdge(true,false,options);
        else
        {
            glLineWidth(1);
            glEnable(GL_POLYGON_OFFSET_FILL);
            glPolygonOffset(1,1);
            renderWEdge(false,false,options);
            glDisable(GL_POLYGON_OFFSET_FILL);

            if (options.showWireframe || options.showInconsistentFaces)
            {
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                glColor3ub(65,65,65);
                renderWEdge(false,true,options);
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            }
        }
    }
}
