#include <map>
#include <set>
#include <strings.h>  // for strcasecmp on Mac
#include <algorithm>

#include "rib2mesh.h"
#include "refineContour.h"

real OPT_LAMBDA = 1;//1e-16;
real OPT_EPSILON = 0.000001;//1e-10;

using namespace std;


OpenSubdiv::OsdCpuComputeContext * g_osdComputeContext = 0;
OpenSubdiv::OsdCpuComputeController * g_osdComputeController = 0;

//
//
// ************************************ RIF SETUP ****************************************
//
//


RifPlugin* RifPluginManufacture(int argc, char ** argv)
{ 
    g_osdComputeController = new OpenSubdiv::OsdCpuComputeController();

    const char * targetSurfacePattern = ".";
    const char * outputFilename = "output.ply";
    const char * exclusionPattern = NULL;
    int subdivisionLevel = 1;
    double meshSmoothing = 0;
    RefinementType refinement = RF_NONE;
    RefineRadialStep lastStep = EVERYTHING;
    bool allowShifts = false;
    bool cullBackFaces = false;
    bool meshSilhouettes = true;
    bool runFreestyle = false;
    const char * outputImage = "default.tiff";
    const char * outputEPSPolyline = "default.svg";
    const char * outputEPSThick = "default.ps";
    const char * freestyleLibPath = "";
    bool runFreestyleInteractive = false;
    int maxInconsistentSplits = 0;
    double cuspTrimThreshold = 0;
    double graftThreshold = 0;
    double wiggleFactor = 0;
    bool useOrientation = false;
    int maxDisplayWidth = -1;
    int maxDisplayHeight = -1;
    std::vector<char*> styleModules;
    bool invertNormals = false;
    bool useConsistency = true;
    int numRefinements = 0;

    if (argc > 1)
        outputFilename = argv[0];

    int i = 1;
    while (i<argc)
    {
        //      printf("PROCESSING %s\n", argv[i]);

        if (strcmp(argv[i],"-pattern") == 0)
        {
            targetSurfacePattern = argv[i+1];
            i+=2;
        }
        else
            if (strcmp(argv[i],"-subdivLevel") == 0)
            {
                subdivisionLevel = atoi(argv[i+1]);
                i+=2;
            }
            else
                if (strcmp(argv[i],"-meshSmoothing") == 0)
                {
                    meshSmoothing = atof(argv[i+1]);
                    i+=2;
                }
                else
                    if (strcmp(argv[i],"-cuspTrimThreshold") == 0)
                    {
                        cuspTrimThreshold = atof(argv[i+1]);
                        i+=2;
                    }
                    else
                        if (strcmp(argv[i],"-graftThreshold") == 0)
                        {
                            graftThreshold = atof(argv[i+1]);
                            i+=2;
                        }
                        else
                            if (strcmp(argv[i],"-wiggleFactor") == 0)
                            {
                                wiggleFactor = atof(argv[i+1]);
                                i+=2;
                            }
                            else
                                if (strcmp(argv[i],"-maxDisplayWidth") == 0)
                                {
                                    maxDisplayWidth= atoi(argv[i+1]);
                                    i+=2;
                                }
                                else
                                    if (strcmp(argv[i],"-maxDisplayHeight") == 0)
                                    {
                                        maxDisplayHeight = atof(argv[i+1]);
                                        i+=2;
                                    }
                                    else
                                        if (strcmp(argv[i],"-exclude") == 0)
                                        {
                                            exclusionPattern = argv[i+1];
                                            i+=2;
                                        }
                                        else if (strcmp(argv[i],"-refinement") == 0)
                                        {
                                            char * refinementStr = argv[i+1];
                                            i+=2;

                                            if (strcmp(refinementStr,"None") == 0)
                                                refinement = RF_NONE;
                                            else if (strcmp(refinementStr,"ContourOnly") == 0)
                                                refinement = RF_CONTOUR_ONLY;
                                            else if (strcmp(refinementStr,"ContourInconsistent") == 0)
                                                refinement = RF_CONTOUR_INCONSISTENT;
                                            else if (strcmp(refinementStr,"Full") == 0)
                                                refinement = RF_FULL;
                                            else if (strcmp(refinementStr, "Optimize") == 0)
                                                refinement = RF_OPTIMIZE;
                                            else if (strcmp(refinementStr, "Radial") == 0)
                                                refinement = RF_RADIAL;
                                            else
                                            {
                                                printf("Invalid refinement type %s\n", refinementStr);
                                                exit(1);
                                            }
                                        }
                                        else if (strcmp(argv[i],"-lastStep") == 0)
                                        {
                                            char * lastStepStr = argv[i+1];
                                            i+=2;

                                            if (strcmp(lastStepStr,"PreProcess") == 0)
                                                lastStep = PREPROCESS;
                                            else if (strcmp(lastStepStr,"DetectCusp") == 0)
                                                lastStep = DETECT_CUSP;
                                            else if (strcmp(lastStepStr,"InsertContour") == 0)
                                                lastStep = INSERT_CONTOUR;
                                            else if (strcmp(lastStepStr,"InsertRadial") == 0)
                                                lastStep = INSERT_RADIAL;
                                            else if (strcmp(lastStepStr,"FlipRadial") == 0)
                                                lastStep = FLIP_RADIAL;
                                            else if (strcmp(lastStepStr,"ExtendRadial") == 0)
                                                lastStep = EXTEND_RADIAL;
                                            else if (strcmp(lastStepStr,"FlipEdge") == 0)
                                                lastStep = FLIP_RADIAL;
                                            else if (strcmp(lastStepStr,"WigglingParam") == 0)
                                                lastStep = WIGGLING_PARAM;
                                            else if (strcmp(lastStepStr,"SplitEdge") == 0)
                                                lastStep = SPLIT_EDGE;
                                            else if (strcmp(lastStepStr,"Everything") == 0)
                                                lastStep = EVERYTHING;
                                            else{
                                                printf("Invalid last step type %s\n", lastStepStr);
                                                exit(1);
                                            }
                                        }
                                        else if (strcmp(argv[i],"-numRefinements") == 0)
                                        {
                                            numRefinements = atoi(argv[i+1]);
                                            i +=2;
                                        }
                                        else if (strcmp(argv[i],"-maxInconsistentSplits") == 0)
                                        {
                                            maxInconsistentSplits = atoi(argv[i+1]);
                                            i+=2;
                                        }
                                        else if (strcmp(argv[i],"-allowShifts") == 0)
                                        {
                                            allowShifts = (strcmp(argv[i+1],"False") != 0);
                                            i+=2;
                                        }
                                        else if (strcmp(argv[i],"-useOrientation") == 0)
                                        {
                                            useOrientation = (strcmp(argv[i+1],"False") != 0);
                                            i+=2;
                                        }
                                        else if (strcmp(argv[i],"-invertNormals") == 0)
                                        {
                                            invertNormals = (strcmp(argv[i+1],"False") != 0);
                                            i+=2;
                                        }
                                        else if (strcmp(argv[i],"-cullBackFaces") == 0)
                                        {
                                            cullBackFaces = (strcmp(argv[i+1],"False") != 0);
                                            i+=2;
                                        }
                                        else if (strcmp(argv[i],"-exclude") == 0)
                                        {
                                            exclusionPattern = argv[i+1];
                                            i+=2;
                                        }
                                        else if (strcmp(argv[i],"-meshSilhouettes") == 0)
                                        {
                                            meshSilhouettes = (strcmp(argv[i+1],"False") != 0);
                                            i+=2;
                                        }
                                        else if (strcmp(argv[i],"-runFreestyle") == 0)
                                        {
                                            runFreestyle = (strcmp(argv[i+1],"False") != 0);
                                            i+=2;
                                        }
                                        else if (strcmp(argv[i],"-useConsistency") == 0)
                                        {
                                            useConsistency = (strcmp(argv[i+1],"False") != 0);
                                            i+=2;
                                        }
                                        else if (strcmp(argv[i],"-outputImage") == 0)
                                        {
                                            outputImage = argv[i+1];
                                            i+=2;
                                        }
                                        else if (strcmp(argv[i],"-outputEPSPolyline") == 0)
                                        {
                                            outputEPSPolyline = argv[i+1];
                                            i+=2;
                                        }
                                        else if (strcmp(argv[i],"-outputEPSThick") == 0)
                                        {
                                            outputEPSThick = argv[i+1];
                                            i+=2;
                                        }
                                        else if (strcmp(argv[i],"-freestyleLibPath") == 0)
                                        {
                                            freestyleLibPath = argv[i+1];
                                            i+=2;
                                        }
                                        else if (strcmp(argv[i],"-runFreestyleInteractive") == 0)
                                        {
                                            runFreestyleInteractive = (strcmp(argv[i+1],"False") != 0);
                                            i += 2;
                                        }
                                        else if (strcmp(argv[i],"-beginStyleModules") == 0)
                                        {
                                            i++;
                                            while (i < argc && strcmp(argv[i],"-endStyleModules") != 0)
                                            {
                                                styleModules.push_back(argv[i]);
                                                i++;
                                            }
                                            i++;
                                        }
                                        else
                                        {
                                            printf("RIBTO3DS: INVALID COMMAND-LINE (i=%d, argv[i] = %s)\n",i,argv[i]);
                                            exit(1);
                                        }
    }


    rib2mesh * obj = new rib2mesh(targetSurfacePattern,outputFilename,exclusionPattern,subdivisionLevel,meshSmoothing,
                            refinement, numRefinements, maxInconsistentSplits, allowShifts, maxDisplayWidth, maxDisplayHeight, useOrientation, invertNormals,
                            cullBackFaces, meshSilhouettes, useConsistency, runFreestyle,
                            runFreestyleInteractive, cuspTrimThreshold, graftThreshold, wiggleFactor, outputImage, outputEPSPolyline, outputEPSThick, freestyleLibPath, lastStep);

    for(std::vector<char*>::iterator it = styleModules.begin(); it != styleModules.end(); ++it)
        obj->addStyle(*it);

    return obj;

}



rib2mesh::rib2mesh(const char * targetSurfacePattern, const char *outputFilename, const char * exclusionPattern,
             int subdivisionLevel,double meshSmoothing,
             RefinementType refinement, int numRefinements, int maxInconsistentSplits,
             bool allowShifts, int maxDisplayWidth, int maxDisplayHeight,
             bool useOrientation, bool invertNormals,
             bool cullBackFaces,
             bool meshSilhouettes, bool useConsistency, bool runFreestyle, bool runFreestyleInteractive,
             double cuspTrimThreshold, double graftThreshold, double wiggleFactor,
             const char * outputImage, const char * outputEPSPolyline, const char * outputEPSThick, const char * freestyleLibPath, RefineRadialStep lastStep)
{ 
    printf("Using pattern: %s\n", targetSurfacePattern);
    printf("Output geom filename: %s\n", outputFilename);
    if (exclusionPattern == NULL)
        printf("No exclusion pattern\n");
    else
        printf("Exclusion pattern: %s\n", exclusionPattern);

    //  printf("Mesh smo: %f\n", wiggleFactor);

    //  printf("Output camera filename: %s\n", outputCameraFilename);

    _filter.ClientData = this;
    _filter.SubdivisionMeshV = subdivisionMeshV;
    _filter.HierarchicalSubdivisionMeshV = hierarchicalSubdivisionMeshV;
    _filter.AttributeV = attributeV;

    _filter.Transform = transform;
    _filter.ConcatTransform = concatTransform;
    _filter.Clipping = clipping;
    _filter.ScreenWindow = screenWindow;
    _filter.Orientation = orientation;
    _filter.ReverseOrientation = reverseOrientation;
    _filter.Format = format;
    _filter.FrameAspectRatio = frameAspectRatio;
    _filter.TransformBegin = transformBegin;
    _filter.TransformEnd = transformEnd;
    _filter.Identity = identity;
    _filter.MotionBeginV = motionBeginV;
    _filter.MotionEnd = motionEnd;
    _filter.WorldBegin = worldBegin;
    _filter.ProjectionV = projection;

    // AttributeBegin and AttributeEnd push and pop the matrix stack too.  The PRman spec is misleading on this point, but this page
    // seems to indicate this: "http://wiki.pixar.com/display/FAQ/Scopes+and+stacks"
    _filter.AttributeBegin = attributeBegin;
    _filter.AttributeEnd = attributeEnd;
    // i am otherwise ignoring the entire attribute stack

    _outputFilename = outputFilename;
    //  _outputCameraFilename = outputCameraFilename;
    _getNextSubd = false;
    _getNextXform_geom = false;
    _firstConcatTransform = true;
    //  _transformDepth = 0;
    //  _getNextXform_camera = false;
    _subdivisionLevel = subdivisionLevel;
    _meshSmoothing = meshSmoothing;
    //  _orientationOutside = true;
    _attributeStack.push_back(Attribute(true));
    _refinement = refinement;
    _lastStep = lastStep;
    _numRefinements = numRefinements;
    _allowShifts = allowShifts;
    _cullBackFaces = cullBackFaces;
    _meshSilhouettes = meshSilhouettes;
    _runFreestyle = runFreestyle;
    _outputImage = outputImage;
    _outputEPSPolyline = outputEPSPolyline;
    _outputEPSThick = outputEPSThick;
    _freestyleLibPath = freestyleLibPath;
    _runFreestyleInteractive = runFreestyleInteractive;
    _maxInconsistentSplits = maxInconsistentSplits;
    _cuspTrimThreshold = cuspTrimThreshold;
    _graftThreshold = graftThreshold;
    _useOrientation = useOrientation;
    _motionState = -1;
    _maxDisplayWidth = maxDisplayWidth;
    _maxDisplayHeight = maxDisplayHeight;
    _invertNormals = invertNormals;
    _useConsistency = useConsistency;
    _focalLength = 1;

    // set the initial transformation to be an identity.  a hack (?) to try to get the Bunny to work.
    //  _currentMatrix.SetIdentity();
    mat4 firstMatrix;
    firstMatrix.SetIdentity();
    _matrixStack.push_back(firstMatrix);

    _totalInputFaces = 0;
    _totalOutputFaces = 0;
    _totalInconsistentFaces = 0;
    _totalStrongInconsistentFaces = 0;

    //  _numMeshes = 0;

    // ---------------------------- COMPILE THE REGEXPs ----------------------------------

    int err = regcomp(&_geom_regexp, targetSurfacePattern, REG_EXTENDED|REG_NOSUB);

    if (err != 0)
    {
        printf("Bad regexp: %s", targetSurfacePattern);
        exit(1);
    }

    if (exclusionPattern != NULL)
    {
        _geomExclusion = true;
        err = regcomp(&_geom_exclusion_regexp, exclusionPattern, REG_EXTENDED|REG_NOSUB);

        if (err != 0)
        {
            printf("Bad regexp (%s)",exclusionPattern);
            exit(1);
        }
    }
    else
    {
        _geomExclusion = false;
    }

    // ----------------------------- INITIALIZE THE OUTPUT FILE -------------------------

    //  _output3DSFile = lib3ds_file_new();
    //  _output3DSFile->frames = 1;
}

template <class T>
static inline T sgn(const T &x)
{
    return (x < T(0)) ? T(-1) : T(1);
}

#ifndef M_TWOPIf
# define M_TWOPIf 6.2831855
#endif

void hsv2srgb(double H, double S, double V, double color[3])
{
    // From FvD
    if (S <= 0.0){
        color[0] = V;
        color[1] = V;
        color[2] = V;
        return;
    }
    H = std::fmod(H, M_TWOPIf);
    if (H < 0.0)
        H += M_TWOPIf;
    H *= 6.0 / M_TWOPIf;
    int i = int(std::floor(H));
    double f = H - i;
    double p = V * (1.0 - S);
    double q = V * (1.0 - (S*f));
    double t = V * (1.0 - (S*(1.0-f)));
    switch(i) {
    case 0:
        color[0] = V;
        color[1] = t;
        color[2] = p;
        return;
    case 1:
        color[0] = q;
        color[1] = V;
        color[2] = p;
        return;
    case 2:
        color[0] = p;
        color[1] = V;
        color[2] = t;
        return;
    case 3:
        color[0] = p;
        color[1] = q;
        color[2] = V;
        return;
    case 4:
        color[0] = t;
        color[1] = p;
        color[2] = V;
        return;
    default:
        color[0] = V;
        color[1] = p;
        color[2] = q;
        return;
    }
}

// curvature to color
void compute_curv_colors(double k1, double k2, double feature_size, double color[3])
{
    double cscale = 20.0; //(8.0 * feature_size) * (8.0 * feature_size);

    double H = 0.5 * (k1 + k2);
    double K = k1 * k2;
    double h = 4.0 / 3.0 * fabs(atan2(H*H-K,H*H*sgn(H)));
    double s = M_2_PI * atan((2.0*H*H-K)*cscale);
    hsv2srgb(h,s,1.0,color);
}

// Similar, but grayscale mapping of mean curvature H
double compute_gcurv_colors(double k1, double k2, double feature_size)
{
    double cscale = 10.0 * feature_size;
    double H = 0.5 * (k1 + k2);
    double c = (atan(H*cscale) + M_PI_2) / M_PI;
    c = sqrt(c);
    return fmin(fmax(c, 0.0), 1.0);
}

double compute_feature_size(HbrMesh<VertexDataCatmark>* mesh, bool radial=false)
{
    int nv = mesh->GetNumVertices();
    int nsamp = std::min(nv, 500);

    vector<real> samples;
    samples.reserve(nsamp * 2);

    for (int i = 0; i < nsamp; i++) {
        // Quick 'n dirty portable random number generator
        static unsigned randq = 0;
        randq = unsigned(1664525) * randq + unsigned(1013904223);

        int ind = randq % nv;
        if(!mesh->GetVertex(ind)){
            i--;
            continue;
        }
        if(radial){
               samples.push_back(fabsl(mesh->GetVertex(ind)->GetData().radialCurvature));
        }else{
            samples.push_back(fabsl(mesh->GetVertex(ind)->GetData().k1));
            samples.push_back(fabsl(mesh->GetVertex(ind)->GetData().k2));
        }
    }

    const real frac = 0.1;
    const real mult = 0.01;

    int which = int(frac * samples.size());
    nth_element(samples.begin(), samples.begin() + which, samples.end());

    return (mult / samples[which]);
}

int rib2mesh::SavePLYFile()
{

    // ---- count the number of vertices and faces ----
    int numVertices = 0;
    int numFaces = 0;
    for(std::vector<HbrMesh<VertexDataCatmark>*>::iterator it = _outputMeshesCatmark.begin(); it != _outputMeshesCatmark.end(); ++it)
    {
        numVertices += (*it)->GetNumVertices();
        numFaces += (*it)->GetNumFaces();
    }

    // ---- output the PLY header ----

    FILE * fp = fopen(_outputFilename, "wt");

    if (fp == NULL)
    {
        printf("ERROR: CANNOT OPEN OUTPUT PLY FILE\n");
        exit(1);
    }

    fprintf(fp,"ply\n");
    fprintf(fp,"format ascii 1.0\n");
    fprintf(fp,"comment %s\n", _meshSilhouettes ? "mesh silhouettes" : "smooth silhouettes");
    fprintf(fp,"element vertex %d\n", numVertices);
    fprintf(fp,"property float x\n");   // maybe should save as doubles?  what would this require?
    fprintf(fp,"property float y\n");   // maybe should save as doubles?
    fprintf(fp,"property float z\n");   // maybe should save as doubles?
    fprintf(fp,"property float nx\n");
    fprintf(fp,"property float ny\n");
    fprintf(fp,"property float nz\n");
    fprintf(fp,"property float red\n");
    fprintf(fp,"property float green\n");
    fprintf(fp,"property float blue\n");
    fprintf(fp,"property float ndotv\n");

    fprintf(fp,"element face %d\n", numFaces);
    //  fprintf(fp,"property uchar vbf\n");
    fprintf(fp,"property list uchar int vertex_index\n");
    fprintf(fp,"property uchar int\n");  // vbf
    fprintf(fp,"end_header\n");

    // ---- output all the vertices and save their IDs ----

    int nextVertID = 0;
    std::map<HbrVertex<VertexDataCatmark>*,int> vmapcc;

    for(std::vector<HbrMesh<VertexDataCatmark>*>::iterator it = _outputMeshesCatmark.begin(); it != _outputMeshesCatmark.end(); ++it)
    {
        double feature_size = compute_feature_size(*it);
        double feature_size_radial = compute_feature_size(*it,true);

        std::list<HbrVertex<VertexDataCatmark>*> verts;
        (*it)->GetVertices(std::back_inserter(verts));
        for(std::list<HbrVertex<VertexDataCatmark>*>::iterator vit = verts.begin(); vit!= verts.end(); ++vit)
        {
            vmapcc[*vit] = nextVertID;
            nextVertID ++;
            fprintf(fp, "%.16f %.16f %.16f", double((*vit)->GetData().pos[0]), double((*vit)->GetData().pos[1]), double((*vit)->GetData().pos[2]));

            if (!_meshSilhouettes)
            {
                vec3 normal = FaceAveragedVertexNormal(*vit);
                fprintf(fp, " %.16f %.16f %.16f", double(normal[0]),double(normal[1]),double(normal[2]));
            }
            else {
                vec3 normal = -1.*(*vit)->GetData().normal;
                fprintf(fp, " %.16f %.16f %.16f", double(normal[0]),double(normal[1]),double(normal[2]));
            }

            //double C = compute_gcurv_colors((*vit)->GetData().k1,(*vit)->GetData().k2,feature_size);
            double color[3];
            compute_curv_colors((*vit)->GetData().k1,(*vit)->GetData().k2,feature_size,color);

            fprintf(fp, " %f %f %f %.16f\n",
                    //C,C,C,
                    color[0], color[1], color[2],
                    //double((*vit)->GetData().ndotv));
                    double((*vit)->GetData().radialCurvature)/(0.5*feature_size_radial));
                    //double(0.5*((*vit)->GetData().k1+(*vit)->GetData().k2))); //double((*vit)->GetData().ndotv),

        }
    }

    assert(nextVertID == numVertices);

    // --- output all the faces ----------

    for(std::vector<HbrMesh<VertexDataCatmark>*>::iterator it = _outputMeshesCatmark.begin(); it != _outputMeshesCatmark.end(); ++it)
    {
        std::list<HbrFace<VertexDataCatmark>*> faces;
        (*it)->GetFaces(std::back_inserter(faces));
        for(std::list<HbrFace<VertexDataCatmark>*>::iterator fit = faces.begin(); fit != faces.end(); ++fit)
        {
            FacingType vf = VertexBasedFacing(*fit);
            int vfint = (vf == FRONT ? 1 : (vf == BACK ? 2 : 3));

            if(((*fit)->GetVertex(0)->GetData().facing == CONTOUR ||
                (*fit)->GetVertex(1)->GetData().facing == CONTOUR ||
                (*fit)->GetVertex(2)->GetData().facing == CONTOUR) &&
                    IsRadialFace(*fit))
                vfint+=4;

            assert((*fit)->GetNumVertices() == 3);
            fprintf(fp,"3 %d %d %d %d\n", vmapcc[(*fit)->GetVertex(0)], vmapcc[(*fit)->GetVertex(1)], vmapcc[(*fit)->GetVertex(2)], vfint);
        }
    }

    // close output file
    fclose(fp);

    return numFaces;
}


rib2mesh::~rib2mesh()
{
    // ----------------------------- SAVE AND CLOSE THE OUTPUT FILE ---------------------

    // generate a PLY file

    int numFaces = SavePLYFile();

    printf("Deleting meshes\n");

    // delete all the meshes
    for(std::vector<HbrMesh<VertexDataCatmark>*>::iterator it = _outputMeshesCatmark.begin(); it != _outputMeshesCatmark.end(); ++it)
        delete *it;

    if (false && NUM_INCONSISTENT_SAMPLES > 0)
        printf("STATS: Input faces: %d, Output faces: %d, Inconsistent faces: %d, Strong Inconsistent Faces: %d\n\n",
               _totalInputFaces, _totalOutputFaces, _totalInconsistentFaces, _totalStrongInconsistentFaces);
    else
        printf("STATS: Input faces: %d, Output faces: %d, Inconsistent faces: %d on contour: %d radial: %d, Non-Radial faces: %d\n\n",
               _totalInputFaces, _totalOutputFaces, _totalInconsistentFaces, _totalContourInconsistentFaces, _totalRadialInconsistentFaces, _totalNonRadialFaces);


    if (numFaces == 0)
        printf("Entire scene clipped\n");

    if (_runFreestyle)
    {
#ifdef LINK_FREESTYLE
        if (numFaces > 0)
            runFreestyle();
        else
            printf("Not running Freestyle\n");
#else
        printf("Error: can't run Freestyle (not linked)\n");
        exit(1);
#endif
    }
}


//
//
// ************* Hooks for capturing camera, transformation, and name parameters *************
//
//

RtVoid rib2mesh::attributeV(RtToken nm, RtInt n, RtToken tokens[], RtPointer parms[])
{
    if ((strcmp(nm, "identifier") == 0) && (strcmp(tokens[0],"name") == 0 || strcmp(tokens[0],"string name") == 0))
    {
        rib2mesh *obj = static_cast<rib2mesh *> ( RifGetCurrentPlugin() );


        // test the regexp against the geometry pattern
        int r = regexec(& obj->_geom_regexp, ((RtString *) parms[0])[0], 1, 0, 0);
        int r2 = -1;

        if (obj->_geomExclusion)
            r2 = regexec(& obj->_geom_exclusion_regexp, ((RtString *) parms[0])[0], 1, 0, 0);


        obj->_getNextSubd = (r == 0) && (r2 != 0);
        obj->_getNextXform_geom = (r == 0) && (r2 != 0);

        if (obj->_getNextSubd)
        {
            if (strlen(((RtString *) parms[0])[0]) > 1000)
            {
                printf("Attribute name longer than 1000 characters (%s).\n", ((RtString *) parms[0])[0]);
            }

            strcpy(obj->_currentName, ((RtString *) parms[0])[0]);
        }
    }

    RiAttributeV(nm, n, tokens, parms); // pass the call down the chain
}

RtVoid rib2mesh::transform(RtMatrix xform)
{
    rib2mesh *obj = static_cast<rib2mesh *> ( RifGetCurrentPlugin() );

    obj->_matrixStack.back().Set(xform);

    RiTransform(xform);
}

RtVoid rib2mesh::identity()
{
    rib2mesh *obj = static_cast<rib2mesh *> ( RifGetCurrentPlugin() );

    obj->_matrixStack.back().SetIdentity();

    RiIdentity();
}

RtVoid rib2mesh::concatTransform(RtMatrix xform)
{
    rib2mesh *obj = static_cast<rib2mesh *> ( RifGetCurrentPlugin() );

    // ignore all motion but the first
    if (obj->_motionState != 1)
    {
        mat4 m;
        m.Set(xform);
        obj->_matrixStack.back() = m * obj->_matrixStack.back();

        if (obj->_motionState == 0)
            obj->_motionState = 1;
    }

    RiConcatTransform(xform);
}

RtVoid rib2mesh::worldBegin()
{
    // from the Prman spec: the camera matrix is defined as the current matrix at the time when WorldBegin is called. The current transform is reset to identity by this operation.

    rib2mesh *obj = static_cast<rib2mesh *> ( RifGetCurrentPlugin() );
    obj->_cameraMatrix = obj->_matrixStack.back();

    obj->extractCameraCenter();

    obj->_matrixStack.back().SetIdentity();

    RiWorldBegin();
}



RtVoid rib2mesh::transformBegin()
{
    rib2mesh *obj = static_cast<rib2mesh *> ( RifGetCurrentPlugin() );
    obj->_matrixStack.push_back(obj->_matrixStack.back());

    RiTransformBegin();
}

RtVoid rib2mesh::transformEnd()
{
    rib2mesh *obj = static_cast<rib2mesh *> ( RifGetCurrentPlugin() );
    obj->_matrixStack.pop_back();

    RiTransformEnd();
}


RtVoid rib2mesh::attributeBegin()
{
    rib2mesh *obj = static_cast<rib2mesh *> ( RifGetCurrentPlugin() );
    obj->_matrixStack.push_back(obj->_matrixStack.back());
    obj->_attributeStack.push_back(obj->_attributeStack.back());

    RiAttributeBegin();
}

RtVoid rib2mesh::attributeEnd()
{
    rib2mesh *obj = static_cast<rib2mesh *> ( RifGetCurrentPlugin() );
    obj->_matrixStack.pop_back();
    obj->_attributeStack.pop_back();

    RiAttributeEnd();
}

RtVoid rib2mesh::motionBeginV(RtInt n, RtFloat times[])
{
    rib2mesh *obj = static_cast<rib2mesh *> ( RifGetCurrentPlugin() );

    obj->_motionState = 0;

    RiMotionBeginV(n, times);
}

RtVoid rib2mesh::motionEnd()
{
    rib2mesh *obj = static_cast<rib2mesh *> ( RifGetCurrentPlugin() );

    obj->_motionState = -1;

    RiMotionEnd();
}



RtVoid rib2mesh::extractCameraCenter()
{
    // determine the coordinates of the camera center from the camera matrix.
    // assumes the upper diagonal of the matrix is orthonormal (i.e., reflection and/or rotation): [R t; 0 1]
    // compute -R^T * t

//      printf("wt = [%Lf %Lf %Lf %Lf; %Lf %Lf %Lf %Lf; %Lf %Lf %Lf %Lf; %Lf %Lf %Lf %Lf]'\n",
//        _cameraMatrix[0][0], _cameraMatrix[1][0], _cameraMatrix[2][0], _cameraMatrix[3][0],
//        _cameraMatrix[0][1], _cameraMatrix[1][1], _cameraMatrix[2][1], _cameraMatrix[3][1],
//        _cameraMatrix[0][2], _cameraMatrix[1][2], _cameraMatrix[2][2], _cameraMatrix[3][2],
//        _cameraMatrix[0][3], _cameraMatrix[1][3], _cameraMatrix[2][3], _cameraMatrix[3][3]
//         );


    // compute -R^T * t   (assumes the upper-left component is orthonormal (i.e., rotation or reflection)
    //probably a prettier way to write this using matrix operators

    for(int i=0;i<3;i++)
        _cameraCenter[i] =
                -_cameraMatrix[i][0] * _cameraMatrix[3][0] +
                -_cameraMatrix[i][1] * _cameraMatrix[3][1] +
                -_cameraMatrix[i][2] * _cameraMatrix[3][2];

    printf("camera center = %f %f %f\n", (float)_cameraCenter[0], (float)_cameraCenter[1], (float)_cameraCenter[2]);

}

RtVoid rib2mesh::clipping(RtFloat near, RtFloat far)
{
    rib2mesh *obj = static_cast<rib2mesh *> ( RifGetCurrentPlugin() );

    obj->_near = near;
    obj->_far = far;

    RiClipping(near,far);
}

RtVoid rib2mesh::screenWindow(RtFloat left, RtFloat right, RtFloat bottom, RtFloat top)
{
    rib2mesh *obj = static_cast<rib2mesh *> ( RifGetCurrentPlugin() );

    obj->_left = left;
    obj->_right = right;
    obj->_top = top;
    obj->_bottom = bottom;

    RiScreenWindow(left, right, bottom, top);
}

RtVoid rib2mesh::format(RtInt xres, RtInt yres, RtFloat pixelaspect)
{
    rib2mesh *obj = static_cast<rib2mesh *> ( RifGetCurrentPlugin() );

    obj->_xres = xres;
    obj->_yres = yres;
    obj->_pixelaspect = pixelaspect;
}

RtVoid rib2mesh::frameAspectRatio(RtFloat aspect)
{
    rib2mesh *obj = static_cast<rib2mesh *> ( RifGetCurrentPlugin() );

    obj->_aspect = aspect;
}  

RtVoid rib2mesh::orientation(RtToken orientation)
{
    rib2mesh *obj = static_cast<rib2mesh *> ( RifGetCurrentPlugin() );

    if (obj->_useOrientation)
        obj->_attributeStack.back().orientationOutside = (strcmp(orientation,"outside") == 0);
    
    RiOrientation(orientation);
}

RtVoid rib2mesh::reverseOrientation()
{
    rib2mesh *obj = static_cast<rib2mesh *> ( RifGetCurrentPlugin() );

    if (obj->_useOrientation)
        obj->_attributeStack.back().orientationOutside = false;
    
    RiReverseOrientation();
}

// focal length seems to be about depth of field, not image scaling

RtVoid rib2mesh::projection(RtToken name, RtInt nt, RtToken args[], RtPointer argVals[])
{
    rib2mesh *obj = static_cast<rib2mesh *> ( RifGetCurrentPlugin() );

    if (strcmp(name,"perspective") != 0)
    {
        printf("Unhandled projection type: %s, nt = %d\n", name, nt);
        exit(1);
    }

    float fov;

    if (nt > 0)
    {
        fov = *(RtFloat*)(argVals[0]);
        obj->_focalLength = 1/ tan((fov/2)*(M_PI/180));
    }
    else
    {
        fov = 90;
        obj->_focalLength = 1;
    }

    printf("Projection: %s, nt: %d, %f\n", name, nt, fov);
    fflush(stdout);



    RiProjectionV(name, nt, args, argVals);
}

//
//
// ********************************** Camera/viewing Helper functions ****************
//
//


vec3 rib2mesh::matrixTransform(const vec3 & pos)
{
    // transform the vertex position according to the current transformation
    vec4 pos_h(pos[0],pos[1],pos[2],1);

    vec4 posout_h = pos_h * _matrixStack.back();

    vec3 posout =  vec3(posout_h[0], posout_h[1], posout_h[2])/posout_h[3];

    static bool printed = false;

    if (!printed)
    {
        printf("an output point: %Lf %Lf %Lf\n", posout[0], posout[1], posout[2]);
        printed = true;
    }

    assert(posout_h[3] != 0); // check for degenerate point/matrix

    return vec3(posout[0],posout[1],posout[2]);
}


//
//
// *********************************** Hooks for capturing surfaces ***********************
//
//


RtVoid rib2mesh::subdivisionMeshV(RtToken mask, RtInt nf, RtInt nverts[],
                               RtInt verts[], RtInt nt, RtToken tags[],
                               RtInt nargs[], RtInt intargs[], RtFloat floatargs[],
                               RtInt ntokens, RtToken tokens[], RtPointer parms[])
{ 
    rib2mesh *obj = static_cast<rib2mesh *> ( RifGetCurrentPlugin() );

    // ignore everything except the subd we're looking for
    if (!obj->_getNextSubd)
        return;

    obj->_getNextSubd = false;

    /*
  if (obj->_firstConcatTransform)
    {
      printf("Didn't find camera matrix for subd\n");
      exit(1);
    }
  */
    int * nargs3 = new int[nt*3];   // do i need to delete this memory after building the mesh?

    // set the number of stringargs to zero, since the calling function assumes no stringargs
    for(int i=0;i<nt;i++)
    {
        nargs3[3*i] = nargs[2*i];
        nargs3[3*i+1] = nargs[2*i+1];
        nargs3[3*i+2] = 0;
    }

    bool isTriangleMesh = true;

    for(int i=0;i<nf && isTriangleMesh;i++)
        if (nverts[i] != 3)
            isTriangleMesh = false;

    if (strcmp(mask,"catmull-clark") == 0 && !isTriangleMesh)
        handleCatmark(nf,nverts,verts,nt,tags,nargs3,intargs,floatargs,NULL,ntokens,tokens,parms);
    else
    {
        printf("WARNING: UNHANLDED MASK (%s)\n",mask);
    }

    RiSubdivisionMeshV( mask,  nf,  nverts, verts,  nt,  tags, nargs, intargs, floatargs, ntokens, tokens, parms);
}

RtVoid rib2mesh::hierarchicalSubdivisionMeshV(RtToken mask, RtInt nf, RtInt nverts[],
                                           RtInt verts[], RtInt nt, RtToken tags[],
                                           RtInt nargs[], RtInt intargs[], RtFloat floatargs[],
                                           RtToken stringargs[],
                                           RtInt ntokens, RtToken tokens[], RtPointer parms[])
{ 
    rib2mesh *obj = static_cast<rib2mesh *> ( RifGetCurrentPlugin() );

    // ignore everything except the subd we're looking for
    if (!obj->_getNextSubd)
        return;

    obj->_getNextSubd = false;

    /*
  if (obj->_firstConcatTransform)
    {
      printf("Didn't find camera matrix when trying to clip\n");
      exit(1);
    }
  */

    bool isTriangleMesh = true;

    for(int i=0;i<nf && isTriangleMesh;i++)
        if (nverts[i] != 3)
            isTriangleMesh = false;

    if (strcmp(mask,"catmull-clark") == 0 && !isTriangleMesh)
        handleCatmark(nf,nverts,verts,nt,tags,nargs,intargs,floatargs,stringargs,ntokens,tokens,parms);
    else
    {
        printf(" ******* YES ******. mask = %s \n", mask);
    }

    RiHierarchicalSubdivisionMeshV( mask,  nf,  nverts, verts,  nt,  tags, nargs, intargs, floatargs, stringargs, ntokens, tokens, parms);

}


#ifdef LINK_FREESTYLE
template<class T>
void CreatePointDebuggingData(HbrMesh<T> * mesh)
{
    std::list<HbrVertex<T>*> vertices;
    mesh->GetVertices(std::back_inserter(vertices));
    for(typename std::list<HbrVertex<T>*>::iterator it = vertices.begin(); it != vertices.end(); ++it)
    {
        //      printf("adding vertex, location %08X\n", *it);

        vec3 posx = (*it)->GetData().pos;
        char debugString[2000];
        int type = (*it)->GetData().DebugString(debugString);
        double rk = (*it)->GetData().radialCurvature;
        addRIFDebugPoint(type, posx[0], posx[1], posx[2], debugString, rk);
    }
}
#endif

struct NsdSubdivParams {
    RtInt numFaces;
    RtInt numVertices;
    RtInt* faceSizes;
    RtInt* vertIndices;
    RtInt numTags;
    RtToken* tags;
    RtFloat* floatargs;
    RtToken* stringargs;
    RtInt* nargs;
    RtInt* intargs;
};

void rib2mesh::handleCatmark(RtInt nf, RtInt nverts[],
                          RtInt verts[], RtInt nt, RtToken tags[],
                          RtInt nargs[], RtInt intargs[], RtFloat floatargs[],
                          RtToken stringargs[],
                          RtInt ntokens, RtToken tokens[], RtPointer parms[])
{ 
    rib2mesh *obj = static_cast<rib2mesh *> ( RifGetCurrentPlugin() );

    // --------------------------------- SETUP THE SUBD DATA VARIABLES -------------------------

    printf("CREATING Catmark NSD: %s\n", obj->_currentName);
    fflush(stdout);

    NsdSubdivParams params;

    params.numFaces = nf;

    // find out how long verts is, then use verts to find out the number of vertices
    params.numVertices = 0;
    int i, j, k;

    int nverts_len=0;
    for(i=0;i<nf;i++)
        nverts_len += nverts[i];
    for(i=0;i<nverts_len;i++)
        if (verts[i]+1 > params.numVertices)
            params.numVertices = verts[i]+1;

    params.faceSizes = nverts;   // do we need to allocate more memory?
    params.vertIndices = verts;
    params.numTags = nt;
    params.tags = tags;

    params.floatargs = floatargs;
    params.stringargs = stringargs;

    params.nargs = nargs;

    // check if the first tag is interpolateBoundary with zero args (seems like it always is)
    // if so, we need to add an argument due to mismatch of versions

    if ((nt > 0) && (strcasecmp(tags[0],"interpolateboundary") == 0) && (nargs[0] == 0))
    {
        // count the number of intargs
        int nintargs = 0;
        for(i=0;i<nt;i++)
            nintargs += nargs[3*i];

        // add the argument
        params.nargs[0] = 1;

        params.intargs = new int[nintargs+1];   // do I need to worry about deallocating this?
        params.intargs[0] = CatmarkMesh::k_InterpolateBoundaryEdgeAndCorner;
        for(i=0;i<nintargs;i++)
            params.intargs[i+1] = intargs[i];
    }
    else
    {
        // directly copy the arguments; we don't need to fix up the interpolateBoundary argument
        params.intargs = intargs;
    }

    // extract the geometry from the "P" tag.
    // for now, we ignore all tokens other than the geometry.

    int Pindex = -1;
    for(i=0;i<ntokens;i++)
        if ( (strcmp(tokens[i],"P") == 0) || (strcmp(tokens[i],"vertex point P") == 0))
        {
            Pindex = i;
            break;
        }

    if (Pindex == -1)
    {
        printf("Catmark Subd is missing geometry!! (Pindex not defined)\n");
        exit(1);
    }

    // --- copy the vertex positions ---

    vec3* controlGeom = new vec3[params.numVertices];
    RtFloat * geom = (RtFloat*) parms[Pindex];

    for (i=0;i<params.numVertices;i++)
    {
        vec3 pt = obj->matrixTransform(geom[3*i], geom[3*i+1], geom[3*i+2]);
        controlGeom[i] = vec3(pt[0], pt[1], pt[2]);
    }

    // flip all the faces if necessary
    // orientationOutside == False requires flip
    // invertNormals == True requires flip
    // but they cancel out
    bool orientationOutside = obj->_attributeStack.back().orientationOutside;

    if ((!orientationOutside || obj->_invertNormals) && (!orientationOutside != obj->_invertNormals))
    {
        int vind = 0;
        int vs[256];
        for(int i=0;i<nf;i++)
        {
            int numVerts = nverts[i];
            assert(numVerts <= 256);
            for(int j=0;j<numVerts;j++)
                vs[j] = verts[j+vind];
            for(int j=0;j<numVerts;j++)
                verts[j+vind] = vs[numVerts - j -1];
            vind += numVerts;
        }
    }

    // ----------------- CREATE THE SUBD DATA STRUCTURE AND TESSELATE ------------------

    static HbrCatmarkSubdivision<Vertex> catmark;
    CatmarkMesh * surface = new CatmarkMesh(&catmark);

    Vertex vtx;
    for(int i=0;i<params.numVertices; i++ ) {
        CatmarkVertex* v = surface->NewVertex(i, vtx);
        v->GetData().SetPosition(controlGeom[i].x(),controlGeom[i].y(),controlGeom[i].z());
    }

    int maxFaceSize = 0;
    for(i = 0; i < params.numFaces; ++i) {
        maxFaceSize = std::max(params.faceSizes[i], maxFaceSize);
    }

    int* fv = new int[maxFaceSize];
    k = 0;
    for (i=0; i<params.numFaces; ++i) {
        int faceSize = params.faceSizes[i];
        for(j = 0; j < faceSize; ++j) {
            fv[j] = params.vertIndices[k++];
        }

        // now check the half-edges connectivity
        for(j=0; j<faceSize; j++) {
            CatmarkVertex * origin      = surface->GetVertex( fv[j] );
            CatmarkVertex * destination = surface->GetVertex( fv[(j+1)%faceSize] );
            CatmarkHalfedge * opposite  = destination->GetEdge(origin);

            if(origin==NULL || destination==NULL) {
                printf(" An edge was specified that connected a nonexistent vertex\n");
                continue;
            }

            if(origin == destination) {
                printf(" An edge was specified that connected a vertex to itself\n");
                continue;
            }

            if(opposite && opposite->GetOpposite() ) {
                printf(" A non-manifold edge incident to more than 2 faces was found\n");
                continue;
            }

            if(origin->GetEdge(destination)) {
                printf(" An edge connecting two vertices was specified more than once."
                       " It's likely that an incident face was flipped\n");
                continue;
            }
        }

        surface->NewFace(faceSize, fv, i);
    }

    surface->SetInterpolateBoundaryMethod( CatmarkMesh::k_InterpolateBoundaryEdgeOnly );

    surface->Finish();

    if(surface->GetNumDisconnectedVertices())
    {
        printf("The specified subdivmesh contains disconnected surface components.\n");

        // abort or iterate over the mesh to remove the offending vertices
    }

    delete[] controlGeom;

    // ------ RESAMPLE THE SUBD INTO A MESH ------------------------------------------


    printf("Converting to mesh\n");

    HbrMesh<VertexDataCatmark> * outputMesh = SurfaceToMesh(surface, obj->_subdivisionLevel, obj->cameraModel(), true);//, obj->_refinement != RF_FLOWTESS );

    if (outputMesh == NULL) // entire object culled
    {
        printf(" *** ENTIRE OBJECT CLIPPED; IGNORING *** \n");
        delete surface;
        return;
    }

    obj->_totalInputFaces += outputMesh->GetNumFaces();

    // -------- REFINE CONTOUR, RESOLVE INCONSISTENCIES, ETC -------------------------

    if (obj->_refinement == RF_CONTOUR_ONLY || obj->_refinement == RF_FULL || obj->_refinement == RF_CONTOUR_INCONSISTENT)
    {
        printf("Refining contour\n");

        RefineContour(outputMesh, obj->cameraModel().CameraCenter(), obj->_refinement, obj->_allowShifts, obj->_maxInconsistentSplits);
    }
    else if (obj->_refinement == RF_OPTIMIZE)
    {
        RefineContour(outputMesh, obj->cameraModel().CameraCenter(), RF_CONTOUR_ONLY, obj->_allowShifts, obj->_maxInconsistentSplits);

        OptimizeConsistency<VertexDataCatmark>(outputMesh, obj->cameraModel().CameraCenter(), OPT_LAMBDA, OPT_EPSILON);

        WiggleAllVertices<VertexDataCatmark>(outputMesh, obj->cameraModel().CameraCenter());
    }
    else if (obj->_refinement == RF_RADIAL)
    {
        RefineContourRadial(outputMesh, obj->cameraModel().CameraCenter(), obj->_allowShifts, obj->_lastStep);

//        WiggleInParamSpace(outputMesh,obj->cameraModel().CameraCenter());
//        OptimizeConsistency<VertexDataCatmark>(outputMesh, obj->cameraModel().CameraCenter(), OPT_LAMBDA, OPT_EPSILON);
//        WiggleAllFaces<VertexDataCatmark>(outputMesh, obj->cameraModel().CameraCenter());
//        WiggleAllVertices<VertexDataCatmark>(outputMesh, obj->cameraModel().CameraCenter());
    }

    if (obj->_cullBackFaces)
    {
        printf("Culling backfaces\n");
        CullBackFaces<VertexDataCatmark>(outputMesh);
    }

    obj->_totalOutputFaces += outputMesh->GetNumFaces();
    ComputeConsistencyStats(outputMesh, obj->cameraModel().CameraCenter(), obj->_totalInconsistentFaces, obj->_totalStrongInconsistentFaces,
                            obj->_totalNonRadialFaces, obj->_totalContourInconsistentFaces, obj->_totalRadialInconsistentFaces);

#ifdef LINK_FREESTYLE
    CreatePointDebuggingData<VertexDataCatmark>(outputMesh);
#endif

    delete surface;

    obj->_outputMeshesCatmark.push_back(outputMesh);

    printf("DONE: %s\n\n",obj->_currentName);
}


#ifdef LINK_FREESTYLE
void run2(const char * meshFilename, const char * snapshotFilename,
          const char * outputEPSPolyline, const char * outputEPSThick,
          float worldTransform[16],
          float left, float right, float bottom, float top,
          float pixelaspect, float aspectratio,
          float near, float far, float focalLength,
          int outputWidthArg, int outputHeightArg,
          int windowWidthArg, int windowHeightArg,
          int visAlgorithm, bool useConsistency, bool runInteractive,
          double cuspTrimThreshold, double graftThreshold, double wiggleFactor,
          const char * pythonLibPath, bool saveLayers);

void addStyleFS(const char * styleFilename);

void rib2mesh::runFreestyle()
{
    // create a pointer to a 4x4 Matrix
    float camera[16];  // get from _cameraMatrix

    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            camera[4*i+j] = _cameraMatrix[i][j];

    for(std::vector<const char*>::iterator it = _styleModules.begin(); it != _styleModules.end(); ++ it)
        addStyleFS(*it);

    int displayWidth;
    int displayHeight;

    //  _xres *= 2;
    //  _yres *= 2;

    //  printf("Doubling output resolution!\n");

    if (_runFreestyleInteractive && (_xres > _maxDisplayWidth || _xres > _maxDisplayHeight))
    {
        double scaleW = float(_maxDisplayWidth) / _xres;
        double scaleH = float(_maxDisplayHeight) / _yres;

        double scale = scaleW < scaleH ? scaleW : scaleH;

        displayWidth = floor(scale * _xres);
        displayHeight = floor(scale * _yres);
    }
    else
    {
        displayWidth = _xres;
        displayHeight = _yres;
    }

    run2(_outputFilename, _outputImage, _outputEPSPolyline, _outputEPSThick, camera, _left, _right, _bottom, _top,
         _pixelaspect, _aspect, _near, _far, _focalLength, _xres, _yres, displayWidth, displayHeight, 0,
         _useConsistency && _refinement != RF_NONE, _runFreestyleInteractive, _cuspTrimThreshold, _graftThreshold, _wiggleFactor,
         _freestyleLibPath,_styleModules.size() > 1);
}
#endif
