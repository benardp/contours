//
//  Filename         : Controller.h
//  Author           : Stephane Grabli
//  Purpose          : The spinal tap of the system
//  Date of creation : 01/07/2002
//
///////////////////////////////////////////////////////////////////////////////


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

#ifndef  CONTROLLER_H
# define CONTROLLER_H

# include <string>
# include "ConfigIO.h"
# include <QGLViewer/qglviewer.h>   // for opengl
# include "../geometry/FastGrid.h"
# include "../geometry/HashGrid.h"
# include "../view_map/ViewMapBuilder.h"
# include "../system/TimeUtils.h"
# include "../system/Precision.h"
# include "../system/Interpreter.h"
# include "../view_map/FEdgeXDetector.h"

class AppProgressBar;
class AppGLWidget;
class AppMainWindow;
class NodeGroup;
class WShape;
class SShape;
class ViewMap;
class ViewEdge;
class AppCanvas;
class InteractiveShader;
class Shader;
class AppInteractiveShaderWindow;
class AppStyleWindow;
class AppOptionsWindow;
class AppDensityCurvesWindow;

class Controller
{
public:
  Controller() ;
  ~Controller() ;
  
  void SetView(AppGLWidget *iView);
  void SetMainWindow(AppMainWindow *iMainWindow); 
  int  Load3DSFile(const char *iFileName, double jiggleFactor = 0);
  void CloseFile();
  void LoadViewMapFile(const char *iFileName, bool only_camera = false);
  void SaveViewMapFile(const char *iFileName);
  void ComputeViewMap();
  void ComputeSteerableViewMap();
  void saveSteerableViewMapImages();
  void toggleEdgeTesselationNature(Nature::EdgeNature iNature);
  void DrawStrokes();
  void SaveSVG(const char * svgFilename, bool polyline, int polylineWidth);
  void ExposeStyleWindow();
  void ExposeOptionsWindow();
  void ExposeHelpWindow();
  void ExposeAboutWindow();
  void SwapStyleModules(unsigned i1, unsigned i2);
  void InsertStyleModule(unsigned index, const char *iFileName);
  void AddStyleModule(const char *iFileName);
  void RemoveStyleModule(unsigned index);
  void ReloadStyleModule(unsigned index, const char * iFileName);
  void Clear();
  void toggleLayer(unsigned index, bool iDisplay);
  void setModified(unsigned index, bool iMod);
  void resetModified(bool iMod=false);
  void updateCausalStyleModules(unsigned index);
  void saveSnapshot(bool b = false);
  void savePSSnapshot(const QString& iFileName, bool polyline, int polylineWidth);
  void savePSLayers(const QString &baseName, bool polyline, int polylineWidth);
  void saveTextSnapshot(const QString& iFileName);
  void captureMovie();
  void displayMessage(const char *msg, bool persistent = false);
  void displayDensityCurves(int x, int y);
 
  
  ViewEdge * SelectViewEdge(real x, real y);
  FEdge * SelectFEdge(real x, real y);
  NodeGroup* BuildRep(vector<ViewEdge*>::iterator vedges_begin, 
		      vector<ViewEdge*>::iterator vedges_end) ;
  
  NodeGroup* debugNode() {return _DebugNode;}
  AppGLWidget * view() {return _pView;}
  NodeGroup* debugScene() {return _DebugNode;}
  Grid& grid() {return _Grid;}
  
  void toggleVisibilityAlgo();
  void setVisibilityAlgo(ViewMapBuilder::visibility_algo alg, bool useConsistency);

  void SetCuspTrimThreshold(real threshold) { _cuspTrimThreshold = threshold; }
    void SetGraftThreshold(real threshold) { _graftThreshold = threshold; }

  void setQuantitativeInvisibility(bool iBool); // if true, we compute quantitativeInvisibility
  bool getQuantitativeInvisibility() const;

  void setFrontBufferFlag(bool b);
  bool getFrontBufferFlag() const;
  void setBackBufferFlag(bool b);
  bool getBackBufferFlag() const;

  void setComputeRidgesAndValleysFlag(bool b);
  bool getComputeRidgesAndValleysFlag() const ;
  void setComputeSuggestiveContoursFlag(bool b);
  bool getComputeSuggestiveContoursFlag() const ;

  void setComputeSteerableViewMapFlag(bool iBool);
  bool getComputeSteerableViewMapFlag() const;
  void setSphereRadius(real s){_sphereRadius=s;}
  real getSphereRadius() const {return _sphereRadius;}
  void setSuggestiveContourKrDerivativeEpsilon(real dkr){_suggestiveContourKrDerivativeEpsilon=dkr;}
  real getSuggestiveContourKrDerivativeEpsilon() const {return _suggestiveContourKrDerivativeEpsilon;}

  AppProgressBar* getProgressBar(void) { return _ProgressBar; }

  void		setModelsDir(const QString& dir);
  QString	getModelsDir() const;
  void		setModulesDir(const QString& dir);
  QString	getModulesDir() const;
  void		setPapersDir(const QString& dir);
  QString	getPapersDir() const;
  void		setHelpIndex(const QString& dir);
  QString	getHelpIndex() const;
  void		setBrowserCmd(const QString& cmd);
  QString	getBrowserCmd() const;

  void resetInterpreter();
  Interpreter * interpreter() { return _inter; }
  void printRowCount() const; 

  // stuff to do with picking and debugging visualizations
  //  void setupNames()  { renderWEdge(false,false,true); }
  void renderWEdge(bool selectionMode, bool useColors, DebugVisOptions options);
  void render(bool selectionMode, DebugVisOptions options);
  void printSelected(GLuint selection);


  void addRIFDebugPoint(DebugPoint::PointType ptType, Vec3r pos, char * debugString, real radialCurvature)
  //			bool rootFindingFailed, bool degenerate) 
  { _ViewMap->addRIFDebugPoint(ptType,pos, debugString, radialCurvature); }

  //  void addRIFFacings(const map<int,int> & facings);

  //  GLuint getName(WFace*f) { return (_faceNames2.find(f) != _faceNames2.end() ? _faceNames2[f] : (GLuint)-1);}

private:

  // Main Window:
  AppMainWindow *_pMainWindow;

  // List of models currently loaded
  vector<string> _ListOfModels;

  // Current directories
  ConfigIO* _current_dirs;

  //View
  // 3D
  AppGLWidget *_pView;
  
  // 2D
  //Viewer2DWindow *_pView2DWindow;
  //Viewer2D *_pView2D;
  
  //Model
  // Drawing Structure
  NodeGroup *_RootNode;

  // Winged-Edge structure
  WingedEdge* _winged_edge;
  
  ViewMap * _ViewMap;

  // Silhouette structure:
  //std::vector<SShape*> _SShapes;
  //NodeGroup *_SRoot;
  
  // Silhouette
  NodeGroup *_SilhouetteNode;
  //  NodeGroup *_ProjectedSilhouette;
  //  NodeGroup *_VisibleProjectedSilhouette;
  NodeGroup *_ViewMapVisNode;
  NodeGroup *_ViewMapColorNode;

  // more Debug info
  NodeGroup *_DebugNode;
  NodeGroup *_PODebugNode;

  // debug
  //  NodeUser<ViewMap> *_ViewMapNode; // FIXME

  // Chronometer:
  Chronometer _Chrono;

  // Progress Bar
  AppProgressBar *_ProgressBar;

  // edges tesselation nature
  int _edgeTesselationNature;

  FastGrid _Grid;
  //HashGrid _Grid;
  
  unsigned int _SceneNumFaces;
  real _minEdgeSize;
  real _EPSILON;
  real _bboxDiag;

  AppCanvas *_Canvas;

  AppStyleWindow *_pStyleWindow;
  AppOptionsWindow *_pOptionsWindow;
  AppDensityCurvesWindow *_pDensityCurvesWindow;

  ViewMapBuilder::visibility_algo	_VisibilityAlgo;
  bool _useConsistency;

  // Script Interpreter
  Interpreter* _inter;

  QString	_help_index;
  QString	_browser_cmd;

  bool _EnableQI;
  bool _ComputeRidges;
  bool _ComputeSuggestive;
  real _sphereRadius;
  real _suggestiveContourKrDerivativeEpsilon;

  bool _ComputeSteerableViewMap;

  FEdgeXDetector edgeDetector;

  real _cuspTrimThreshold;
    real _graftThreshold;

  // stuff for visualization/picking
  //  GLuint _selection;
  //  map<GLuint,WFace*> _faceNames;
  //  map<WFace*,GLuint> _faceNames2;
};

extern Controller	*g_pController;

#endif // CONTROLLER_H
