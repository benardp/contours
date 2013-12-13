#include <iostream>
#include <vector>
#include <regex.h>

#include <ri.h>
#ifdef RENDERMAN
#include <RifPlugin.h>
#else
#include <rif.h>
#endif

#include "refineContour.h"

struct Attribute
{
public:
    bool orientationOutside;
    Attribute(bool orientation) { orientationOutside = orientation; }
};

class rib2mesh : public RifPlugin
{ 
public:
    RifFilter _filter;
    int _subdivisionLevel;  // how much to subdivide
    const char * _outputFilename; // output mesh file
    int _numVerts;
    int _numFaces;
    double _meshSmoothing;
    RefinementType _refinement;
    RefineRadialStep _lastStep;
    bool _allowShifts;
    bool _cullBackFaces;
    bool _meshSilhouettes;
    int _maxInconsistentSplits;
    bool _useOrientation;
    bool _invertNormals;
    bool _useConsistency;

    // meshes to save to the output file
    std::vector<HbrMesh<VertexDataCatmark>*> _outputMeshesCatmark;

    // for running Freestyle from the RIF
    bool _runFreestyle;
    bool _runFreestyleInteractive;
    const char * _outputImage; // output TIFF file
    const char * _outputEPSPolyline;
    const char * _outputEPSThick;
    const char * _freestyleLibPath;
    std::vector<const char*> _styleModules;
    double _cuspTrimThreshold;
    double _graftThreshold;
    double _wiggleFactor;

    // Regex describing which objects to output
    regex_t _geom_regexp;

    bool _geomExclusion;
    regex_t _geom_exclusion_regexp;

    // state for capturing matches
    bool _getNextSubd;
    bool _getNextXform_geom;
    int _motionState;  // -1 = not in MotionBegin group, 0 = in MotionBegin, haven't seen any transformation matrix, 1 = in MotionBegin, have seen a matrix
    bool _firstConcatTransform;
    char _currentName[1000];
    mat4 _cameraMatrix;
    float _near, _far; // clipping planes
    float _left, _right, _top, _bottom; // viewport
    vec3 _cameraCenter;
    std::vector<mat4> _matrixStack;
    std::vector<Attribute> _attributeStack;

    // processing statistics
    int _totalInputFaces;
    int _totalOutputFaces;
    int _totalInconsistentFaces;
    int _totalContourInconsistentFaces;
    int _totalRadialInconsistentFaces;
    int _totalStrongInconsistentFaces;
    int _totalNonRadialFaces;

    // only needed when calling Freestyle
    int _xres, _yres;
    int _maxDisplayWidth, _maxDisplayHeight;
    float _pixelaspect, _aspect;
    float _focalLength;

    vec3 matrixTransform(const vec3 &);
    vec3 matrixTransform(double x, double y, double z) { return matrixTransform(vec3(x,y,z)); }

    static void handleCatmark(RtInt nf, RtInt nverts[],
                              RtInt verts[], RtInt nt, RtToken tags[],
                              RtInt nargs[], RtInt intargs[], RtFloat floatargs[],
                              RtToken stringargs[],
                              RtInt, RtToken[], RtPointer[]);

    // RIF hooks
    static RtVoid subdivisionMeshV(RtToken mask, RtInt nf, RtInt nverts[],
                                   RtInt verts[], RtInt nt, RtToken tags[],
                                   RtInt nargs[], RtInt intargs[], RtFloat floatargs[],
                                   RtInt, RtToken[], RtPointer[]);
    static RtVoid hierarchicalSubdivisionMeshV(RtToken mask, RtInt nf, RtInt nverts[],
                                               RtInt verts[], RtInt nt, RtToken tags[],
                                               RtInt nargs[], RtInt intargs[], RtFloat floatargs[],
                                               RtToken stringargs[],
                                               RtInt, RtToken[], RtPointer[]);
    static RtVoid attributeV(RtToken nm, RtInt n, RtToken tokens[], RtPointer parms[]);

    static RtVoid transform(RtMatrix xform);
    static RtVoid concatTransform(RtMatrix xform);
    static RtVoid clipping(RtFloat near, RtFloat far);
    static RtVoid screenWindow(RtFloat left, RtFloat right, RtFloat bottom, RtFloat top);
    static RtVoid orientation(RtToken orientation);
    static RtVoid reverseOrientation();
    static RtVoid identity();

    static RtVoid transformBegin();
    static RtVoid transformEnd();

    static RtVoid attributeBegin();
    static RtVoid attributeEnd();

    static RtVoid motionBeginV(RtInt n, RtFloat times[]);
    static RtVoid motionEnd();
    static RtVoid worldBegin();

    static RtVoid format(RtInt xres, RtInt yres, RtFloat pixelaspect);
    static RtVoid frameAspectRatio(RtFloat aspect);

    static RtVoid projection(RtToken name, RtInt nt, RtToken args[], RtPointer argVals[]);

    CameraModel cameraModel() {  return CameraModel(_cameraMatrix, _near, _far, _left, _right, _top, _bottom,
                                                    _xres, _yres, _focalLength, _cameraCenter); }
    void extractCameraCenter();

#ifdef LINK_FREESTYLE
    void runFreestyle();
#endif

    int SavePLYFile();

    rib2mesh(const char* targetPattern, const char *outputFilename, const char * exclusionPattern,
          int subdivisionMeshV, double meshSmoothing, RefinementType refinement,
          int maxInconsistentSplits, bool allowShifts, int maxDisplayWidth, int maxDisplayHeight, bool useOrientation,
          bool invertNormals, bool cullBackFaces, bool meshSilhouettes, bool useConsistency,
          bool runFreestyle, bool runFreestyleInteractive, double cuspTrimThreshold, double graftThreshhold,  double wiggleFactor,
          const char * outputTIFF, const char * outputEPSpolyline, const char * outputEPSthick,
          const char * freestyleLibPath, RefineRadialStep lastStep);
    void addStyle(char * filename) { _styleModules.push_back(filename); }
    ~rib2mesh();
    RifFilter& GetFilter() { return _filter; }
};
