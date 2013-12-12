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

#include <iostream>
#include <QTextStream>
#include <QImage>
#include <QTableWidget>
#include <QTextEdit>
#include <QMouseEvent>
#include "../rendering/GLUtils.h"
#include "../stroke/Canvas.h"
#include "AppGLWidget.h"
#include "../scene_graph/NodeLight.h"
#include "../rendering/GLRenderer.h"
#include "../rendering/GLSelectRenderer.h"
#include "../rendering/GLBBoxRenderer.h"
#include "../rendering/GLMonoColorRenderer.h"
#include "Controller.h"
#include "../view_map/Silhouette.h"
#include "../view_map/ViewMap.h"
#include "../scene_graph/LineRep.h"
#include "../scene_graph/NodeShape.h"
#include "../scene_graph/VertexRep.h"
#include "AppConfig.h"

GLuint texture = 0;

bool AppGLWidget::_frontBufferFlag = false;
bool AppGLWidget::_backBufferFlag = true;


extern bool offscreenRendering;
//extern bool useCameraFromRIB;
extern double RIB_cameraModelview[16];
extern double RIB_cameraProjection[16];
extern double RIB_camera_center[3];
extern int output_viewport_x, output_viewport_y, output_viewport_width, output_viewport_height;
extern int window_viewport_x, window_viewport_y, window_viewport_width, window_viewport_height;
extern int outputWidth, outputHeight;


namespace qglviewer {
ScreenManipulatedCameraFrame ::ScreenManipulatedCameraFrame()
{
    setFlySpeed(0.0);
    removeFromMouseGrabberPool();
    scaleFactor = 1;
    trans = Vec();

    connect(&flyTimer_, SIGNAL(timeout()), SLOT(flyUpdate()));
}

void ScreenManipulatedCameraFrame::mouseMoveEvent(QMouseEvent* const event, Camera* const camera)
{
    if(action_ == QGLViewer::TRANSLATE){
        const QPoint delta = prevPos_ - event->pos();
        trans += Vec(static_cast<float>(-delta.x()), static_cast<float>(delta.y()), 0.0);
    }else if(action_== QGLViewer::ZOOM){
        const float zoomCoef = 2.f * scaleFactor;
        scaleFactor += zoomCoef * (event->y() - prevPos_.y()) / (float)camera->screenHeight();
        if(scaleFactor<1.f) scaleFactor = 1.f;
    }

    if (action_ != QGLViewer::NO_MOUSE_ACTION){
        prevPos_ = event->pos();
        Q_EMIT manipulated();
    }
}

void ScreenManipulatedCameraFrame::wheelEvent(QWheelEvent* const event, Camera* const camera)
{
    if(action_ == QGLViewer::ZOOM){
        const float wheelSensitivityCoef = 8E-4f;
        scaleFactor += event->delta() * wheelSensitivity() * wheelSensitivityCoef;
        if(scaleFactor<1.f) scaleFactor = 1.f;

        Q_EMIT manipulated();
    }
    if (previousConstraint_)
        setConstraint(previousConstraint_);

    const int finalDrawAfterWheelEventDelay = 400;

    // Starts (or prolungates) the timer.
#if QT_VERSION >= 0x040000
    flyTimer_.setSingleShot(true);
    flyTimer_.start(finalDrawAfterWheelEventDelay);
#else
    flyTimer_.start(finalDrawAfterWheelEventDelay, true);
#endif

    action_ = QGLViewer::NO_MOUSE_ACTION;
}
}


int getRegion(unsigned int name);


AppGLWidget::AppGLWidget(QWidget *iParent, const char *iName)
    : QGLViewer(iParent)
{
    _Fovy        = 30.f;
    _RenderStyle = LINE;
    _ModelRootNode = new NodeDrawingStyle;
    _SilhouetteRootNode = new NodeDrawingStyle;
    _DebugRootNode = new NodeDrawingStyle;
    _punchOutRootNode = new NodeDrawingStyle;
    _ViewMapVisNode = new NodeDrawingStyle;
    _ViewMapColorNode = new NodeDrawingStyle;

    _RootNode.AddChild(_ModelRootNode);

    _SilhouetteRootNode->SetStyle(DrawingStyle::LINES);
    _SilhouetteRootNode->SetLightingEnabled(false);
    _SilhouetteRootNode->SetLineWidth(2.f);  // ignored
    _SilhouetteRootNode->SetPointSize(3.f);  // ignored

    _RootNode.AddChild(_SilhouetteRootNode);

    _ViewMapVisNode->SetStyle(DrawingStyle::LINES);
    _ViewMapVisNode->SetLightingEnabled(false);
    _ViewMapVisNode->SetLineWidth(5.f);   // ignored
    _ViewMapVisNode->SetPointSize(8.f); // ignored

    _RootNode.AddChild(_ViewMapVisNode);

    _ViewMapColorNode->SetStyle(DrawingStyle::LINES);
    _ViewMapColorNode->SetLightingEnabled(false);
    _ViewMapColorNode->SetLineWidth(5.f);   // ignored
    _ViewMapColorNode->SetPointSize(8.f); // ignored

    _RootNode.AddChild(_ViewMapColorNode);

    _DebugRootNode->SetStyle(DrawingStyle::LINES);
    _DebugRootNode->SetLightingEnabled(false);
    _DebugRootNode->SetLineWidth(1.f);

    _RootNode.AddChild(_DebugRootNode);

    _DebugRootNode->SetStyle(DrawingStyle::LINES);
    _DebugRootNode->SetLightingEnabled(false);
    _DebugRootNode->SetLineWidth(1.f);

    _punchOutRootNode->SetStyle(DrawingStyle::LINES);
    _punchOutRootNode->SetLightingEnabled(false);
    _punchOutRootNode->SetLineWidth(1.f);

    _minBBox = __min(__min(_ModelRootNode->bbox().getMin()[0],
                     _ModelRootNode->bbox().getMin()[1]),
            _ModelRootNode->bbox().getMin()[2]);
    _maxBBox = __max(__max(_ModelRootNode->bbox().getMax()[0],
                     _ModelRootNode->bbox().getMax()[1]),
            _ModelRootNode->bbox().getMax()[2]);

    _maxAbs = __max(rabs(_minBBox), rabs(_maxBBox));
    _minAbs = __min(rabs(_minBBox), rabs(_maxBBox));

    camera()->setZNearCoefficient(0.0001);
    _standardCamera = camera();

    // 2D Scene
    //  _pFENode = new NodeDrawingStyle;
    //  _pFENode->SetStyle(DrawingStyle::LINES);
    //  _pFENode->SetLightingEnabled(false);
    //  _pFENode->SetLineWidth(1.f);
    //
    //  _p2DNode.AddChild(_pFENode);
    //
    //  _pVisibleSilhouetteNode = new NodeDrawingStyle;
    //  _pVisibleSilhouetteNode->SetStyle(DrawingStyle::LINES);
    //  _pVisibleSilhouetteNode->SetLightingEnabled(false);
    //  _pVisibleSilhouetteNode->SetLineWidth(3.f);
    //
    //  _p2DNode.AddChild(_pVisibleSilhouetteNode);
    //
    _p2DSelectionNode = new NodeDrawingStyle;
    _p2DSelectionNode->SetLightingEnabled(false);
    _p2DSelectionNode->SetStyle(DrawingStyle::LINES);
    _p2DSelectionNode->SetLineWidth(5.f);

    _p2DNode.AddChild(_p2DSelectionNode);

    _pGLRenderer = new GLRenderer;
    _pSelectRenderer = new GLSelectRenderer;
    _pBBoxRenderer = new GLBBoxRenderer;
    _pMonoColorRenderer = new GLMonoColorRenderer;
    _pDebugRenderer = new GLDebugRenderer;

    _pMainWindow = NULL;
    _cameraStateSaved = false;
    _drawBBox = false;
    _silhouette = false;
    //  _fedges = false;
    //  _drawViewMap = false;
    _debug = false;
    _punchOutDebug = false;
    _selection_mode = false;
    _Draw2DScene = true;
    _Draw3DScene = true;
    _drawEnvMap = false;
    _currentEnvMap = 1;
    _maxId = 0;
    _blendFunc = 0;

    const QString sep(Config::DIR_SEP.c_str());
    const QString filename = Config::Path::getInstance()->getHomeDir() + sep +
            Config::OPTIONS_DIR + sep + Config::OPTIONS_QGLVIEWER_FILE;
    setStateFileName(filename);

    //get camera frame:
    qglviewer::Camera * cam = camera();
    qglviewer::ManipulatedFrame *  fr = cam->frame() ;
    _enableUpdateSilhouettes = false;
    connect(fr, SIGNAL(modified()), this, SLOT(updateSilhouettes()));
    _captureMovie = false;
    //  _frontBufferFlag = false;
    //  _backBufferFlag = true;
    _record = false;

    _SurfaceRenderStyle = SMOOTH_SURFACE_STYLE;


    _visOptions.showMesh = true;
    _visOptions.showViewEdges = true;
    _visOptions.showPoints = false;
    _visOptions.showRIFPoints = false;
    _visOptions.visibleOnly = true;
    _visOptions.edgeColors = DebugVisOptions::EC_TYPE;
    _visOptions.selectionName = (GLuint)-1;
    _visOptions.showPunchouts = false;
    _visOptions.limitRegion = false;
    _visOptions.regionIndex = -1;
    _visOptions.showPOCuspTesselation = false;
    _visOptions.showPairs = true;
    _visOptions.showSpecificPoints = false;
    _visOptions.pointsToShow = DebugPoint::ERROR;
    _visOptions.radialCurvaturePoints = false;
    _visOptions.showViewVertices = false;
    _visOptions.nDotVshading = false;
    _visOptions.showWireframe = true;
    _visOptions.showInconsistentFaces = true;
    _visOptions.showRadial = true;
    _visOptions.lighting = true;
    _visOptions.normals = false;

    _frontCamera.setFrame(&_sframe);
    _frontCameraMode = false;
}

AppGLWidget::~AppGLWidget()
{
    int ref = _RootNode.destroy();

    _Light.destroy();
    ref = _p2DNode.destroy();

    if(NULL != _pGLRenderer)
    {
        delete _pGLRenderer;
        _pGLRenderer = NULL;
    }

    if(NULL != _pSelectRenderer)
    {
        delete _pSelectRenderer;
        _pSelectRenderer = NULL;
    }

    if(NULL != _pBBoxRenderer)
    {
        delete _pBBoxRenderer;
        _pBBoxRenderer = NULL;
    }

    if(NULL != _pMonoColorRenderer)
    {
        delete _pMonoColorRenderer;
        _pMonoColorRenderer = NULL;
    }

    if(NULL != _pDebugRenderer)
    {
        delete _pDebugRenderer;
        _pDebugRenderer = NULL;
    }
}

void AppGLWidget::SetMainWindow(QMainWindow *iMainWindow) {
    _pMainWindow = iMainWindow;
}
void AppGLWidget::captureMovie()
{
    _captureMovie = true;
    setSnapshotFormat("PNG");
    setSnapshotFileName("anim");
    camera()->playPath(0);
}

void
AppGLWidget::updateSilhouettes()
{
    if(!_enableUpdateSilhouettes || !g_pController)
        return;
    g_pController->ComputeViewMap();
    g_pController->DrawStrokes();
    if(_captureMovie)
    {
        if(!camera()->keyFrameInterpolator(0)->interpolationIsStarted())
        {
            _captureMovie = false;
            return;
        }
        saveSnapshot(true);
    }
}

void
AppGLWidget::select(const QMouseEvent *e) {

    // 3D Shape selection

    if (_Draw3DScene) {

        printf("Selecting (%d,%d)\n",int(e->x()), int(e->y()));

        // Make openGL context current
        makeCurrent();

        const unsigned SENSITIVITY = 4;
        const unsigned NB_HITS_MAX = 64;

        // Prepare the selection mode
        static GLuint hits[NB_HITS_MAX];

        glSelectBuffer(NB_HITS_MAX, hits);
        glRenderMode(GL_SELECT);
        glInitNames();

        // Loads the matrices
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        GLint viewport[4];

        if(_frontCameraMode){
            glGetIntegerv(GL_VIEWPORT, viewport);
            gluPickMatrix(static_cast<GLdouble>(e->x()), static_cast<GLdouble>(camera()->screenHeight() - e->y()), SENSITIVITY, SENSITIVITY, viewport);

            glMultMatrixd(RIB_cameraProjection);

            glMatrixMode(GL_MODELVIEW);
            glLoadMatrixd(RIB_cameraModelview);
        }else{
            camera()->getViewport(viewport);
            gluPickMatrix(static_cast<GLdouble>(e->x()), static_cast<GLdouble>(e->y()), SENSITIVITY, SENSITIVITY, viewport);

            // loadProjectionMatrix() first resets the GL_PROJECTION matrix with a glLoadIdentity.
            // Give false as a parameter in order to prevent this and to combine the matrices.
            camera()->loadProjectionMatrix(false);

            camera()->loadModelViewMatrix();
        }

        // Render scene with objects ids
        g_pController->render(true,_visOptions);



        //    _pSelectRenderer->setSelectRendering(true);
        //    DrawScene(_pSelectRenderer);
        glFlush();

        CHECK_FOR_ERROR;

        // Get the results
        GLint nb_hits = glRenderMode(GL_RENDER);

        if (nb_hits <= 0) {
            printf("====== No selection ====\n");
            _visOptions.selectionName = (GLuint)-1;
            return;
        }

        // Interpret results
        unsigned int zMin = hits[1];
        unsigned int selected = hits[3];
        for (int i=1; i<nb_hits; ++i)
            if (hits[i*4+1] < zMin)
            {
                zMin = hits[i*4+1];
                selected = hits[i*4+3];
            }

        _visOptions.selectionName = selected;
        _visOptions.regionIndex = getRegion(selected);



        //    if (_visOptions.regionIndex == -1)
        //      _visOptions.limitRegion = false;

        printf("======= Selection (ID: %d, Region: %d) =====\n", selected, _visOptions.regionIndex);



        //    _pSelectRenderer->setSelectedId(selected);
        //cout << "SHAPE" << endl;
        //    cout << "-----" << endl;
        //    cout << "Id: " << _pSelectRenderer->getSelectedId() << endl;
        //    cout << endl;

        CHECK_FOR_ERROR;

        return;
    }

    if (!_Draw2DScene)
        return;

    // ViewMap selection

    FEdge *fe = g_pController->SelectFEdge(e->x(), height()-e->y());
    if (!fe)
        return;
    ViewEdge * ve = fe->viewedge();

    if (ve) {
        cout << "VIEWEDGE" << endl;
        cout << "--------" << endl;
        cout << "ViewEdge Id: " << ve->getId().getFirst() << ", " << ve->getId().getSecond() << endl;
        cout << "Shape Id: " << ve->shape_id() << endl;
        cout << "Nature: " << ve->getNature() << endl;
        cout << "QI: " << ve->qi() << endl;
        if(ve->aShape())
            cout << "Occludee: " << ve->aShape()->getId() << endl;
        else
            cout << "Occludee: NULL" << endl ;
        cout << endl;

        cout << "FEDGE" << endl;
        cout << "-----" << endl;
        cout << "FEdge Id: " << fe->getId().getFirst() << ", " << fe->getId().getSecond() << endl;
        cout << "Vertex A Id: " << fe->vertexA()->getId() << endl;
        cout << "Vertex B Id: " << fe->vertexB()->getId() << endl;
        cout << endl;

        vector<ViewEdge*> vedges;
        vedges.push_back(ve);
        _p2DSelectionNode->AddChild(g_pController->BuildRep(vedges.begin(), vedges.end()));
        // FEdge
        LineRep * fedgeRep = new LineRep(fe->vertexA()->point2d(), fe->vertexB()->point2d());
        fedgeRep->SetWidth(3.f);
        NodeShape * fedgeNode = new NodeShape;
        fedgeNode->AddRep(fedgeRep);
        fedgeNode->material().SetDiffuse(0.2, 1, 0.2, 1.0);
        _p2DSelectionNode->AddChild(fedgeNode);
        //SVertex A
        Vec3r A(fe->vertexA()->point2d());
        VertexRep * aVertexRep = new VertexRep(A.x(), A.y(), A.z());
        aVertexRep->SetPointSize(3.f);
        NodeShape * aVertexNode = new NodeShape;
        aVertexNode->AddRep(aVertexRep);
        aVertexNode->material().SetDiffuse(1, 0, 0, 1.0);
        _p2DSelectionNode->AddChild(aVertexNode);
        // and its fedges
        const vector<FEdge*>& afedges = fe->vertexA()->fedges();
        vector<FEdge*>::const_iterator f=afedges.begin(), fend=afedges.end();
        for(;
            f!=fend;
            ++f)
        {
            LineRep * lrep = new LineRep((*f)->vertexA()->point2d(), (*f)->vertexB()->point2d());
            lrep->SetWidth(1.f);
            aVertexNode->AddRep(lrep);
        }
        //SVertex B
        Vec3r B(fe->vertexB()->point2d());
        VertexRep * bVertexRep = new VertexRep(B.x(), B.y(), B.z());
        bVertexRep->SetPointSize(3.f);
        NodeShape * bVertexNode = new NodeShape;
        bVertexNode->AddRep(bVertexRep);
        bVertexNode->material().SetDiffuse(0, 0, 1, 1.0);
        _p2DSelectionNode->AddChild(bVertexNode);
        // and its fedges
        const vector<FEdge*>& bfedges = fe->vertexB()->fedges();
        f=bfedges.begin();
        fend=bfedges.end();
        for(;
            f!=fend;
            ++f)
        {
            LineRep * lrep = new LineRep((*f)->vertexA()->point2d(), (*f)->vertexB()->point2d());
            lrep->SetWidth(1.f);
            bVertexNode->AddRep(lrep);
        }

    }
}


void
AppGLWidget::mousePressEvent(QMouseEvent *e)
{
    _p2DSelectionNode->destroy();
    if (e->button() == Qt::LeftButton)
    {
        if(e->modifiers() == Qt::ShiftModifier)
        {
            select(e);
        }
        else if(e->modifiers() == Qt::ControlModifier)
        {
            // Density Observation
            g_pController->displayDensityCurves(e->x(), height()-1-e->y());
        }else{
            QGLViewer::mousePressEvent(e);
        }
        updateGL();
    }
    else
        QGLViewer::mousePressEvent(e);
}

void
AppGLWidget::mouseReleaseEvent  (  QMouseEvent *    e  ) 
{
    //  if(g_pController)
    //    g_pController->ComputeViewMap();
    //  g_pController->DrawStrokes();
    QGLViewer::mouseReleaseEvent(e);
}

void
AppGLWidget::keyPressEvent(QKeyEvent* e)
{
    switch (e->key())
    {
    case Qt::Key_U:
        _enableUpdateSilhouettes = !_enableUpdateSilhouettes;
        break;
    case Qt::Key_Escape:
        break;
    case Qt::Key_V:
        _visOptions.visibleOnly = !_visOptions.visibleOnly;
        updateGL();
        break;
    case Qt::Key_R:
        if(e->modifiers() == Qt::ShiftModifier){
            _record = !_record;
            if(_record){
                setSnapshotFormat("JPEG");
                setSnapshotFileName("anim");
                g_pController->displayMessage("record", true);
            }else{
                g_pController->displayMessage("");
            }

        }
        else if(e->modifiers() == Qt::ControlModifier && _cameraStateSaved) {
            setCameraState(_cameraPosition, _cameraOrientation);
            updateGL();
        }else{
            _visOptions.showRadial =! _visOptions.showRadial;
            updateGL();
        }
        break;
    case Qt::Key_PageUp:
        if(e->modifiers() == Qt::ControlModifier)
            _blendFunc = (_blendFunc + 1) % 2;
        else {
            _currentEnvMap++;
            if(_currentEnvMap > _maxId)
                _currentEnvMap = 1;
        }
        updateGL();
        break;
    case Qt::Key_PageDown:
        if(e->modifiers() == Qt::ControlModifier)
            _blendFunc = (_blendFunc + 1) % 2;
        else {
            _currentEnvMap--;
            if(_currentEnvMap < 1)
                _currentEnvMap = _maxId;
        }
        updateGL();
        break;

        // All of the names of the rendering styles are now obsolete
    case Qt::Key_1: _Draw2DScene = false; _Draw3DScene = true; _SurfaceRenderStyle = INVISIBLE_SURFACE_STYLE;  _visOptions.showMesh = false; updateGL(); break;
    case Qt::Key_2: _Draw2DScene = false; _Draw3DScene = true; _SurfaceRenderStyle = SELECTION_SURFACE_STYLE; _visOptions.lighting = true;  _visOptions.showMesh = true; _visOptions.nDotVshading = false; updateGL(); break;
    case Qt::Key_3: _Draw2DScene = false; _Draw3DScene = true; _SurfaceRenderStyle = SELECTION_SURFACE_STYLE; _visOptions.lighting = false;  _visOptions.showMesh = true; _visOptions.nDotVshading = false; updateGL(); break;
    case Qt::Key_4: _Draw2DScene = false; _Draw3DScene = true; _SurfaceRenderStyle = SMOOTH_SURFACE_STYLE; _visOptions.showMesh = false; _visOptions.lighting = false; _visOptions.nDotVshading = false; updateGL(); break;
    case Qt::Key_5: _Draw2DScene = false; _Draw3DScene = true; _SurfaceRenderStyle = N_DOT_V_STYLE; _visOptions.lighting = false;  _visOptions.showMesh = true; _visOptions.nDotVshading = true; updateGL(); break;
    case Qt::Key_0:
    {
        _frontCameraMode = !_frontCameraMode;
        if(_frontCameraMode)
            setCamera(&_frontCamera);
        else
            setCamera(_standardCamera);
        updateGL(); break;
    }
    case Qt::Key_B:
    {
        _drawBBox == true ? _drawBBox = false : _drawBBox = true; updateGL(); break;
    }
    case Qt::Key_S:
    {
        _visOptions.showSpecificPoints = !_visOptions.showSpecificPoints;
        updateGL();
    }

    case Qt::Key_Equal:
    {
        if (_visOptions.pointsToShow == DebugPoint::SI_SIL_CONNECTION)
            _visOptions.pointsToShow = DebugPoint::ERROR;
        else
            _visOptions.pointsToShow = DebugPoint::PointType(_visOptions.pointsToShow + 1);
        updateGL();
    }
        break;

    case Qt::Key_Minus:
    {
        if (_visOptions.pointsToShow == DebugPoint::ERROR)
            _visOptions.pointsToShow = DebugPoint::SI_SIL_CONNECTION;
        else
            _visOptions.pointsToShow = DebugPoint::PointType(_visOptions.pointsToShow - 1);
        updateGL();
    }
        break;

    case Qt::Key_L:
    {
        _visOptions.limitRegion = !_visOptions.limitRegion; updateGL();
        break;
    }
        break;

    case Qt::Key_T:
    {
        _visOptions.showPOCuspTesselation = !_visOptions.showPOCuspTesselation; updateGL();
        break;
    }

    case Qt::Key_E:
    {
        _visOptions.showViewEdges = !_visOptions.showViewEdges;
        updateGL();
        break;
    }

    case Qt::Key_W:
        _visOptions.showWireframe = !_visOptions.showWireframe;
        updateGL();
        break;

    case Qt::Key_I:
    {
        _visOptions.edgeColors = DebugVisOptions::EC_ID_COLOR;
        updateGL();
        break;
    }

    case Qt::Key_M:
    {
        _visOptions.showPairs = !_visOptions.showPairs;
        updateGL();
        break;
    }

    case Qt::Key_C:
    {
        _visOptions.edgeColors = DebugVisOptions::EC_TYPE;
        updateGL();
        break;
    }


    case Qt::Key_D:
    {
        _visOptions.showPoints = !_visOptions.showPoints; updateGL();
        break;
    }

    case Qt::Key_G:
    {
        _visOptions.showViewVertices = !_visOptions.showViewVertices; updateGL();
        break;
    }

    case Qt::Key_F:
    {
        _visOptions.showRIFPoints = !_visOptions.showRIFPoints; updateGL();
        break;
    }

    case Qt::Key_P:
        _visOptions.radialCurvaturePoints = !_visOptions.radialCurvaturePoints; updateGL();
        break;
    case Qt::Key_N:
        _visOptions.normals = !_visOptions.normals; updateGL();
        break;
    case Qt::Key_QuoteLeft:  // back-tick, next to 1 key
    case Qt::Key_F2:
        _Draw2DScene = true;
        _Draw3DScene = false;
        updateGL();
        break;

    case Qt::Key_F3:
        _Draw2DScene = false;
        _Draw3DScene = true;
        updateGL();
        break;

    case Qt::Key_Q:
        _visOptions.showInconsistentFaces = !_visOptions.showInconsistentFaces;
        updateGL();
        break;

    default:
        QGLViewer::keyPressEvent(e);
        break;
    }
}

void AppGLWidget::LoadEnvMap(const char *filename)
{
    GLuint textureId;
    GLubyte *data;
    QImage img(filename, "PNG");
    QImage glImage = QGLWidget::convertToGLFormat(img);
    int d = glImage.depth();
    // allocate a texture name
    glGenTextures( 1, &textureId );
    if(textureId > _maxId)
        _maxId = textureId;

    // select our current texture
    glBindTexture( GL_TEXTURE_2D, textureId );

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                    GL_NEAREST);

    glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA, glImage.width(), glImage.height(), 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, glImage.bits() );
}

void AppGLWidget::help(){
    emit helpRequired();

    bool resize = false;
    int width=600;
    int height=400;

    static QString label[] = {" &Keyboard ", " &Mouse "};

    QTabWidget * hWidget = helpWidget();
    if (!hWidget){
        hWidget = new QTabWidget(NULL);
        hWidget->setWindowTitle("Control Bindings");
        resize = true;
        for (int i=0; i<2; ++i){
            QTextEdit* tab = new QTextEdit(hWidget);
#if QT_VERSION >= 300
            tab->setReadOnly(true);
#endif
            hWidget->insertTab(i, tab, label[i]);
        }
    }

#if QT_VERSION < 300
    const int currentPageIndex = hWidget->currentPageIndex();
#endif

    for (int i=0; i<2; ++i)
    {
        QString text;
        switch (i)
        {
        case 0 : text = keyboardString(); break;
        case 1 : text = mouseString();	  break;
        default : break;
        }

#if QT_VERSION < 300
        hWidget->setCurrentPage(i);
        QTextEdit* textEdit = (QTextEdit*)(hWidget->currentPage());
#else
        hWidget->setCurrentIndex(i);
        QTextEdit* textEdit = (QTextEdit*)(hWidget->currentWidget());
#endif
        textEdit->setHtml(text);

        if (resize && (textEdit->heightForWidth(width) > height))
            height = textEdit->heightForWidth(width);
    }

#if QT_VERSION < 300
    hWidget->setCurrentPage(currentPageIndex);
#endif

    if (resize)
        hWidget->resize(width, height+40); // 40 is tabs' height
    hWidget->show();
    hWidget->raise();
}

QString AppGLWidget::helpString() const{
    QString pdir(Config::Path::getInstance()->getProjectDir());
    QString text = "<a href=\"" + pdir + "/doc/html/index.html\">help content</a>";
    return text;
}

QString AppGLWidget::mouseString() const{
    QString text("<table border=\"1\" cellspacing=\"0\">\n");
    text += "<tr bgcolor=\"#eebf00\"><th align=\"center\">Button</th><th align=\"center\">Description</th></tr>\n";
    text += "<tr><td><b>Shift+Left</b></td><td>If view map exists, selects a view edge.<br> If in selection mode, selects a shape</td></tr>";
    text += "</table>";
    text += QGLViewer::mouseString();
    return text;
}

QString AppGLWidget::keyboardString() const {

    QString text("<table border=\"1\" cellspacing=\"0\">\n");
    text += "<tr bgcolor=\"#eebf00\"><th align=\"center\">Key</th><th align=\"center\">Description</th></tr>\n";
    text += "<tr><td><b>F2</b></td><td>Toggles 2D Scene display</td></tr>";
    text += "<tr><td><b>F3</b></td><td>Toggles 3D Scene display</td></tr>";

    text += "<tr><td><b>1</b></td><td>Filled display mode</td></tr>";
    text += "<tr><td><b>2</b></td><td>Lines display mode</td></tr>";
    text += "<tr><td><b>3</b></td><td>Invisible display mode</td></tr>";

    text += "<tr><td><b>E</b></td><td>Toggles ViewMap display</td></tr>";
    text += "<tr><td><b>B</b></td><td>Toggles bounding boxes display</td></tr>";
    text += "<tr><td><b>S</b></td><td>Toggles GL silhouettes display</td></tr>";
    text += "<tr><td><b>D</b></td><td>Toggles debug information display</td></tr>";
    text += "<tr><td><b>P</b></td><td>Toggles punch-out debug display</td></tr>";
    text += "<tr><td><b>L</b></td><td>Toggles shape selection mode</td></tr>";
    //  text += "<tr><td><b>P</b></td><td>Toggles paper texture display</td></tr>";
    text += "<tr><td><b>M</b></td><td>Toggles toon shading</td></tr>";
    text += "<tr><td><b>V</b></td><td>Toggles visibility algorithm</td></tr>";

    text += "<tr><td><b>R</b></td><td>Reset camera to the latest ViewMap computation settings</td></tr>";
    text += "<tr><td><b>Shift+R</b></td><td>Toggles snapshots mode</td></tr>";

    text += "<tr><td><b>U</b></td><td>Recomputes the ViewMap when the view changes</td></tr>";

    text += "<tr><td><b>+/-</b></td><td>Change paper texture</td></tr>";
    text += "<tr><td><b>PgUp/PgDn</b></td><td>Changes EnvMap</td></tr>";
    text += "<tr><td><b>Ctrl+PgUp/PgDn</b></td><td>Changes blending function</td></tr>";
    text += "</table>";
    text += QGLViewer::keyboardString();
    return text;
}

void AppGLWidget::init()
{
#ifndef __MACH__
    GLenum err = glewInit();
    if (GLEW_OK != err)
    {
        fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
    }
#endif

    setShortcut(QGLViewer::EXIT_VIEWER, 0);
    setShortcut(QGLViewer::STEREO, 0);
    setShortcut(QGLViewer::ANIMATION, 0);
    setShortcut(QGLViewer::EDIT_CAMERA, 0);

    restoreStateFromFile();

    glClearColor(1,1,1,0);
    glShadeModel(GL_SMOOTH);

    glCullFace(GL_BACK);
    glDisable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);

    // open and read texture data
    Config::Path * cpath = Config::Path::getInstance();
    QString envmapDir = cpath->getEnvMapDir();
    LoadEnvMap((envmapDir + QString("gray00.png")).toAscii().data());
    //LoadEnvMap(Config::ENV_MAP_DIR + "gray01.bmp");
    LoadEnvMap((envmapDir + QString("gray02.png")).toAscii().data());
    LoadEnvMap((envmapDir + QString("gray03.png")).toAscii().data());
    LoadEnvMap((envmapDir + QString("brown00.png")).toAscii().data());
    glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_SPHERE_MAP) ;
    glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_SPHERE_MAP) ;

    // gl settings for Environmental Texturing:
    glColor3f(1, 1, 1);

    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    NodeLight *light = new NodeLight;
    _Light.AddChild(light);

    // Change QGLViewer's default shortcut for snapshots
    setShortcut(QGLViewer::SAVE_SCREENSHOT, Qt::CTRL + Qt::Key_W);

    cout << "Renderer (GL)    : " << glGetString(GL_RENDERER) << endl
         << "Vendor (GL)      : " << glGetString(GL_VENDOR) << endl << endl;
}

void AppGLWidget::draw()
{
    CHECK_FOR_ERROR;

    int swidth = camera()->screenWidth();
    int sheight = camera()->screenHeight();

    if(_Draw3DScene && _frontCameraMode){
        float sfactor = _sframe.scaleFactor*fmin((float)swidth/(float)(window_viewport_width),
                                                 (float)sheight/(float)window_viewport_height);
        glViewport((window_viewport_x + _sframe.trans.x) - (_sframe.scaleFactor-1.f)*swidth/4.f,
                   (window_viewport_y + _sframe.trans.y) - (_sframe.scaleFactor-1.f)*sheight/4.f,
                   sfactor*window_viewport_width+window_viewport_x,
                   sfactor*window_viewport_height+window_viewport_y);

        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixd(RIB_cameraModelview);

        glMatrixMode(GL_PROJECTION);
        glLoadMatrixd(RIB_cameraProjection);
    }else{
        glViewport(0,0,swidth,sheight);
    }

    if (_Draw3DScene)
    {
        // draw the 3D scene using the currently-selected rendering style
        switch(_SurfaceRenderStyle)
        {
        case INVISIBLE_SURFACE_STYLE:
            if (_visOptions.showWireframe)
            {
                _ModelRootNode->SetStyle(DrawingStyle::LINES);
                _ModelRootNode->SetLineWidth(1.0);
                _pGLRenderer->SetOverrideColors(true);
                glColor3f(.3,.3,.3);
                DrawScene(_pGLRenderer);
            }

            break; // don't draw anything

        case SMOOTH_SURFACE_STYLE:
            if (_visOptions.showWireframe)
            {
                _ModelRootNode->SetStyle(DrawingStyle::LINES);
                _ModelRootNode->SetLineWidth(1.0);
                _pGLRenderer->SetOverrideColors(true);
                glColor3f(.3,.3,.3);
                DrawScene(_pGLRenderer);
            }

            glPolygonOffset(1,1);
            glEnable(GL_POLYGON_OFFSET_FILL);
            _ModelRootNode->SetStyle(DrawingStyle::FILLED);
            _pGLRenderer->SetOverrideColors(false);
            DrawScene(_pGLRenderer);
            glDisable(GL_POLYGON_OFFSET_FILL);
            break;

        case LINE_SURFACE_STYLE:
            _ModelRootNode->SetStyle(DrawingStyle::LINES);
            _ModelRootNode->SetLineWidth(1.0);
            _pGLRenderer->SetOverrideColors(true);
            glColor3f(.3,.3,.3);
            DrawScene(_pGLRenderer);
            break;

        case SMOOTH_LINES_SURFACE_STYLE:
            _ModelRootNode->SetStyle(DrawingStyle::LINES);
            _ModelRootNode->SetLineWidth(1.0);
            _pGLRenderer->SetOverrideColors(true);
            glColor3f(.3,.3,.3);
            DrawScene(_pGLRenderer);

            glPolygonOffset(1,1);
            glEnable(GL_POLYGON_OFFSET_FILL);
            _ModelRootNode->SetStyle(DrawingStyle::FILLED);
            _pGLRenderer->SetOverrideColors(false);
            DrawScene(_pGLRenderer);
            glDisable(GL_POLYGON_OFFSET_FILL);
            break;

        case FRONTBACK_SURFACE_STYLE:
            if (_visOptions.showWireframe)
            {
                _ModelRootNode->SetStyle(DrawingStyle::LINES);
                _ModelRootNode->SetLineWidth(1.0);
                _pGLRenderer->SetOverrideColors(true);
                glColor3f(0,0,0);
                DrawScene(_pGLRenderer);
            }

            glPolygonOffset(1,1);
            glEnable(GL_POLYGON_OFFSET_FILL);
            _ModelRootNode->SetStyle(DrawingStyle::FILLED);
            _pGLRenderer->SetFrontBackStyle(true);
            _pGLRenderer->SetFBViewpoint(Vec3r(RIB_camera_center[0],RIB_camera_center[1],RIB_camera_center[2]));
            DrawScene(_pGLRenderer);
            _pGLRenderer->SetFrontBackStyle(false);
            glDisable(GL_POLYGON_OFFSET_FILL);

            break;

        case N_DOT_V_STYLE:
        case SELECTION_SURFACE_STYLE:

            break;
        default:
            assert(0);
        }

        g_pController->render(false,_visOptions);
        if (true == _silhouette)
            DrawSilhouette();

        if (true == _drawBBox) {
            glPushAttrib(GL_ALL_ATTRIB_BITS);
            _ModelRootNode->accept(*_pBBoxRenderer);
            glPopAttrib();
        }

        if (true == _debug) {
            glPushAttrib(GL_ALL_ATTRIB_BITS);
            _DebugRootNode->accept(*_pDebugRenderer);
            glPopAttrib();
        }

        if (_punchOutDebug)
        {
            glPushAttrib(GL_ALL_ATTRIB_BITS);
            _punchOutRootNode->accept(*_pDebugRenderer);
            glPopAttrib();
        }

        glDisable(GL_POLYGON_OFFSET_FILL);

        CHECK_FOR_ERROR;
    }

    if (true == _Draw2DScene) {
        Draw2DScene(_pGLRenderer);
        Set3DContext();
    }
    if(_record){
        saveSnapshot(true);
    }
    if(_captureMovie)
    {
        if(!camera()->keyFrameInterpolator(0)->interpolationIsStarted())
        {
            _captureMovie = false;
            return;
        }
        saveSnapshot(true);
    }
}

void AppGLWidget::DrawScene(SceneVisitor *iRenderer)
{
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    if(_drawEnvMap)
    {
        _ModelRootNode->SetLightingEnabled(false);
        glEnable(GL_COLOR_MATERIAL);

        glEnable(GL_TEXTURE_2D);
        // Bind the texture to use
        glBindTexture(GL_TEXTURE_2D,_currentEnvMap);
        switch(_blendFunc)
        {
        case 0:
            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE) ;
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glEnable(GL_BLEND);
            break;
        case 1:
            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE) ;
            glDisable(GL_BLEND);
            break;
        default:
            break;
        }

        glEnable(GL_TEXTURE_GEN_S);
        glEnable(GL_TEXTURE_GEN_T);
    }

    GLfloat pos1[4] = { 0, 0, -1, 0}; // light 0 is on by default, at pos (0,0,1,0)
    GLfloat dif1[4] = { 1, 1, 1, 1};
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glLightfv(GL_LIGHT1,GL_POSITION,pos1);
    glLightfv(GL_LIGHT1,GL_DIFFUSE,dif1);
    glPopMatrix();
    glEnable(GL_LIGHT1);

    _ModelRootNode->accept(*iRenderer);

    glDisable(GL_TEXTURE_GEN_S);
    glDisable(GL_TEXTURE_GEN_T);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_COLOR_MATERIAL);
    _ModelRootNode->SetLightingEnabled(true);

    glPopAttrib();
}

void AppGLWidget::prepareCanvas()
{
    // preparation for 2D rendering

    CHECK_FOR_ERROR;

    makeCurrent();
    CHECK_FOR_ERROR;

    glPushAttrib(GL_ALL_ATTRIB_BITS);
    CHECK_FOR_ERROR;

    if(_frontBufferFlag){
        if(_backBufferFlag)
            glDrawBuffer(GL_FRONT_AND_BACK);
        else
            glDrawBuffer(GL_FRONT);
        CHECK_FOR_ERROR;
    }
    else
        if(_backBufferFlag)
        {
            glDrawBuffer(GL_BACK);
            CHECK_FOR_ERROR;
        }

    // Projection Matrix
    //==================
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    if (offscreenRendering)
        glOrtho(0,outputWidth, 0, outputHeight, -1.0, 1.0);
    else
        glOrtho(0,width(), 0, height(), -1.0, 1.0);
    CHECK_FOR_ERROR;

    //Modelview Matrix
    //================
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    CHECK_FOR_ERROR;

    glViewport(0, 0, outputWidth, outputHeight);
    CHECK_FOR_ERROR;
}

void AppGLWidget::releaseCanvas()
{
    CHECK_FOR_ERROR;
    makeCurrent();
    CHECK_FOR_ERROR;
    glDrawBuffer(GL_BACK);
    CHECK_FOR_ERROR;
    glPopAttrib();
    CHECK_FOR_ERROR;
}

void AppGLWidget::Draw2DScene(SceneVisitor *iRenderer)
{
    CHECK_FOR_ERROR;

    static bool first = 1;
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    // Projection Matrix
    //==================
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if (offscreenRendering)
        glOrtho(0,outputWidth, 0, outputHeight, -1, 1);
    else
        glOrtho(0,width(), 0, height(), -1.0, 1.0);

    if (offscreenRendering)
        glViewport(0,0,outputWidth, outputHeight);

    CHECK_FOR_ERROR;

    //Modelview Matrix
    //================
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    Canvas * canvas = Canvas::getInstance();
    if((canvas) && (!canvas->isEmpty()))
    {
        CHECK_FOR_ERROR;
        if (first)
        {
            canvas->init();
            first = false;
        }

        CHECK_FOR_ERROR;

        canvas->Render(canvas->renderer());

        CHECK_FOR_ERROR;
    }

    glLoadIdentity();
    glPushAttrib(GL_DEPTH_BUFFER_BIT);
    glDisable(GL_DEPTH_TEST);
    _p2DSelectionNode->accept(*iRenderer);
    glPopAttrib();

    glPopAttrib();

    CHECK_FOR_ERROR;
}

void AppGLWidget::DrawSilhouette()
{
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glDepthFunc(GL_LESS);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    DrawScene(_pMonoColorRenderer);

    glCullFace(GL_FRONT);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glLineWidth(3.0);
    glPolygonOffset(0.5f, 0.5f);

    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    _pMonoColorRenderer->setColor(0.f, 0.f, 0.f);
    DrawScene(_pMonoColorRenderer);

    //Restore old context
    glPopAttrib();

}

void AppGLWidget::ReInitRenderers()
{
    // Debug Renderer
    if(NULL != _pDebugRenderer)
        _pDebugRenderer->ReInit(rabs(_ModelRootNode->bbox().getMax()[1] -
                                _ModelRootNode->bbox().getMin()[1]));
}

void AppGLWidget::setFrontBufferFlag(bool iBool){
    _frontBufferFlag = iBool;
}
bool AppGLWidget::getFrontBufferFlag() {
    return _frontBufferFlag;
}
void AppGLWidget::setBackBufferFlag(bool iBool){
    _backBufferFlag = iBool;
}
bool AppGLWidget::getBackBufferFlag() {
    return _backBufferFlag;
}
