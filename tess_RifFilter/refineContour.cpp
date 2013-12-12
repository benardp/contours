#include <map>
#include <deque>
#include <set>
#include <numeric>
#include <iostream>
#include <sstream>
#include <math.h>
#include <cfloat>

#include "refineContour.h"

void SavePLYFile(HbrMesh<VertexDataCatmark> * outputMesh, const char *prefix, int index, bool meshSilhouettes)
{
    // Remove disconnected vertices
    std::list<MeshVertex*> vertices;
    outputMesh->GetVertices(std::back_inserter(vertices));
    for(std::list<MeshVertex*>::iterator vit = vertices.begin(); vit != vertices.end(); ++vit){
        if(!(*vit)->IsConnected()) {
            outputMesh->DeleteVertex(*vit);
        }
    }

    std::stringstream strs;
    strs << prefix;
    strs << ".";
    strs << index;
    strs << ".ply";
    std::string temp_str = strs.str();
    char* outputFilename = (char*) temp_str.c_str();

    // ---- count the number of vertices and faces ----
    int numVertices = outputMesh->GetNumVertices();
    int numFaces = outputMesh->GetNumFaces();

    // ---- output the PLY header ----

    FILE * fp = fopen(outputFilename, "wt");

    if (fp == NULL)
    {
        printf("ERROR: CANNOT OPEN OUTPUT PLY FILE\n");
        exit(1);
    }

    fprintf(fp,"ply\n");
    fprintf(fp,"format ascii 1.0\n");
    fprintf(fp,"comment %s\n", meshSilhouettes ? "mesh silhouettes" : "smooth silhouettes");
    fprintf(fp,"element vertex %d\n", numVertices);
    fprintf(fp,"property float x\n");   // maybe should save as doubles?  what would this require?
    fprintf(fp,"property float y\n");   // maybe should save as doubles?
    fprintf(fp,"property float z\n");   // maybe should save as doubles?
    fprintf(fp,"property float nx\n");
    fprintf(fp,"property float ny\n");
    fprintf(fp,"property float nz\n");

    fprintf(fp,"element face %d\n", numFaces);
    fprintf(fp,"property list uchar int vertex_index\n");
    fprintf(fp,"property uchar int\n");  // vbf
    fprintf(fp,"end_header\n");

    // ---- output all the vertices and save their IDs ----

    int nextVertID = 0;
    std::map<HbrVertex<VertexDataCatmark>*,int> vmapcc;

    std::list<HbrVertex<VertexDataCatmark>*> verts;
    outputMesh->GetVertices(std::back_inserter(verts));
    for(std::list<HbrVertex<VertexDataCatmark>*>::iterator vit = verts.begin(); vit!= verts.end(); ++vit)
    {
        vmapcc[*vit] = nextVertID;
        nextVertID ++;
        fprintf(fp, "%.16f %.16f %.16f", double((*vit)->GetData().pos[0]), double((*vit)->GetData().pos[1]), double((*vit)->GetData().pos[2]));

        if (!meshSilhouettes)
        {
            vec3 normal = FaceAveragedVertexNormal(*vit);
            fprintf(fp, " %.16f %.16f %.16f\n", double(normal[0]),double(normal[1]),double(normal[2]));
        }
        else
        {
            vec3 normal = -1.*(*vit)->GetData().normal;
            fprintf(fp, " %.16f %.16f %.16f\n", double(normal[0]),double(normal[1]),double(normal[2]));
        }
    }

    assert(nextVertID == numVertices);

    // --- output all the faces ----------

    std::list<HbrFace<VertexDataCatmark>*> faces;
    outputMesh->GetFaces(std::back_inserter(faces));
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

    // close output file
    fclose(fp);
}


int numVerts = 0;

bool IsRadialFace(MeshVertex* v0, MeshVertex* v1, MeshVertex* v2)
{
    MeshVertex* vertices[3] = {v0,v1,v2};

    for(int i=0; i<3; i++){
        if(vertices[i]->GetData().Radial())
            if(vertices[i]->GetData().HasRadialOrg(vertices[(i+1)%3]) || vertices[i]->GetData().HasRadialOrg(vertices[(i+2)%3]))
                return true;
    }
    return false;
}

bool IsRadialFace(MeshFace* face)
{
    return IsRadialFace(face->GetVertex(0),face->GetVertex(1),face->GetVertex(2));
}

bool IsStandardRadialFace(MeshVertex* v0, MeshVertex* v1, MeshVertex* v2)
{
    if((v0->GetData().cusp && (v1->GetData().HasRadialOrg(v0) || v2->GetData().HasRadialOrg(v0))) ||
            (v1->GetData().cusp && (v0->GetData().HasRadialOrg(v1) || v2->GetData().HasRadialOrg(v1))) ||
            (v2->GetData().cusp && (v0->GetData().HasRadialOrg(v2) || v1->GetData().HasRadialOrg(v2))))
        return true;

    MeshVertex* vertices[3] = {v0,v1,v2};
    int numRadial = 0;
    int numContour = 0;

    for(int i=0; i<3; i++){
        if(vertices[i]->GetData().Radial())
            numRadial++;
        if(vertices[i]->GetData().facing == CONTOUR)
            numContour++;
    }
    if((numRadial==1 && numContour==2) || (numRadial==2 && numContour==1) || numContour == 0){
        return true;
    }
    return false;
}

bool IsStandardRadialFace(MeshFace* face)
{
    return IsStandardRadialFace(face->GetVertex(0),face->GetVertex(1),face->GetVertex(2));
}

void SetupVertex(VertexDataCatmark & vd, const ParamPointCC & p, const vec3 & cameraCenter)
{
    vec3 limitPosition, limitNormal;
    real k1, k2;
    vec3 pdir1, pdir2;

    p.Evaluate(limitPosition, limitNormal, &k1, &k2, &pdir1, &pdir2);

    real ndotv;

    vec3 viewVec = limitPosition - cameraCenter;

    FacingType ft = Facing(viewVec, limitNormal, CONTOUR_THRESHOLD, &ndotv);

    vd.pos = limitPosition;
    vd.normal = limitNormal;
    vd.sourceLoc = p;
    vd.facing = ft;
    vd.ndotv = ndotv;
    vd.extraordinary = p.IsExtraordinaryVertex();

    real k_r = 0;
    if(p.SourceVertex() != NULL || (p.SourceEdge() !=NULL && (p.EdgeT() == 0 || p.EdgeT() == 1)) ||
            (p.SourceFace() !=NULL && (p.FaceU()==0 || p.FaceU()==1) && (p.FaceV()==0 || p.FaceV()==1) )){
        HbrVertex<Vertex> * vertex = NULL;
        if(p.SourceVertex() != NULL){
            vertex = p.SourceVertex();
        }else if(p.SourceEdge()!=NULL){
            vertex = p.EdgeT()==0 ? p.SourceEdge()->GetOrgVertex() : p.SourceEdge()->GetDestVertex();
        }else{
            if(p.FaceU()==0)
                if(p.FaceV()==0)
                    vertex = p.SourceFace()->GetVertex(0);
                else
                    vertex = p.SourceFace()->GetVertex(3);
            else
                if(p.FaceV()==0)
                    vertex = p.SourceFace()->GetVertex(1);
                else
                    vertex = p.SourceFace()->GetVertex(2);
        }
        assert(vertex);
        const real t_threshold = pow(.5,REF_LEVEL) + EXTRAORDINARY_REGION_OFFSET+1e-10;
        if(vertex->GetValence() != 4 || vertex->OnBoundary()){
            k1 = 0.0;
            k2 = 0.0;
            std::set<HbrFace<Vertex>*> oneRingFaces;
            GetOneRing(vertex,oneRingFaces);
            for(std::set<HbrFace<Vertex>*>::iterator it=oneRingFaces.begin(); it!=oneRingFaces.end(); it++){
                HbrFace<Vertex>* face = (*it);
                real u,v;
                GetVertexUV(face,vertex,u,v);
                if(u<0.5)
                    u += t_threshold;
                else
                    u -= t_threshold;
                if(v<0.5)
                    v += t_threshold;
                else
                    v -= t_threshold;
                ParamPointCC paramP = ParamPointCC(face,u,v);
                assert(paramP.IsEvaluable());
                vec3 p,n;
                real radCurv_c, k1_c, k2_c;
                paramP.Evaluate(p,n,&k1_c,&k2_c);
                k1 += k1_c;
                k2 += k2_c;
                paramP.RadialCurvature(cameraCenter,radCurv_c);
                k_r += radCurv_c;
            }
            assert(oneRingFaces.size()!=0);
            k1 /= real(oneRingFaces.size());
            k2 /= real(oneRingFaces.size());
            k_r /= real(oneRingFaces.size());
        }else{
            p.RadialCurvature(cameraCenter,k_r);
        }
    }else{
        p.RadialCurvature(cameraCenter,k_r);
    }

    vd.k1 = k1;
    vd.k2 = k2;
    vd.radialCurvature = k_r;
}

real FindZeroCrossingBySampling(const ParamPointCC & p0, const ParamPointCC & p1, FacingType facing, vec3 cameraCenter)
// given an edge where the vertices/face are all consistent (e.g., all front- or back-facing),
// sample the edge to see if there are any points on the edge with opposite facing.
// if none found, return a Null ParamPoint
{
    real maxNdotVmag = 0;
    real result = -1;

    for(int j=0;j<NUM_INCONSISTENT_SAMPLES;j++)
    {
        real t = (j+1.0)/(NUM_INCONSISTENT_SAMPLES + 1);
        ParamPointCC pt = ParamPointCC::Interpolate(p0, p1, t);

        if (!pt.IsEvaluable()){
            printf("NOT EVALUABLE\n");
            continue;
        }

        pt.SetNormalOffset(0);

        real ndotv;
        FacingType ft = Facing(pt, cameraCenter, CONTOUR_THRESHOLD, &ndotv);

        if (ft != CONTOUR && ft != facing && fabsl(ndotv) > maxNdotVmag)
        {
            maxNdotVmag = fabsl(ndotv);
            result = t;
        }
    }

    return result;
}



//////////////////////////////// SURFACE TO MESH: INITIAL SAMPLING ROUTINES //////////////////////

MeshVertex * ConvertVertex(Mesh * outputMesh, CatmarkVertex * sourceVertex,
                           const vec3 & cameraCenter, std::map<CatmarkVertex*, MeshVertex *> & vertexMap)
{
    if (vertexMap.find(sourceVertex) != vertexMap.end())
        return vertexMap[sourceVertex];

    MeshVertex * newVertex = outputMesh->NewVertex();

    SetupVertex(newVertex->GetData(), ParamPointCC(sourceVertex), cameraCenter);

    newVertex->GetData().age = numVerts ++;

    vertexMap[sourceVertex] = newVertex;

    return newVertex;
}

// generate a face in the output mesh from a face in the input mesh, using the specified vertices of the source face
void ConvertFaceToTriangle(Mesh * outputMesh, CatmarkFace * face, int v0, int v1, int v2,
                           std::map<CatmarkVertex*,MeshVertex *> & vertexMap,
                           const  CameraModel & cameraModel)
{
    CatmarkVertex * vertex[3];

    vertex[0] = face->GetVertex(v0);
    vertex[1] = face->GetVertex(v1);
    vertex[2] = face->GetVertex(v2);

    if (!cameraModel.TriangleInside(vertex[0]->GetData().GetPos(), vertex[1]->GetData().GetPos(),
                                    vertex[2]->GetData().GetPos())){
        return;
    }

    ConvertVertex(outputMesh, vertex[0], cameraModel.CameraCenter(), vertexMap);
    ConvertVertex(outputMesh, vertex[1], cameraModel.CameraCenter(), vertexMap);
    ConvertVertex(outputMesh, vertex[2], cameraModel.CameraCenter(), vertexMap);

    NewFace(outputMesh, vertexMap[vertex[0]],vertexMap[vertex[1]],vertexMap[vertex[2]]);
}

// insert a point at the center of a source quad and generate four triangles in the output mesh
void ConvertFaceToTriangles(Mesh * outputMesh, CatmarkFace * face,
                            std::map<CatmarkVertex*, MeshVertex*> & vertexMap,
                            const CameraModel & cameraModel)
{
    // create a point at the center of the face
    ParamPointCC centerLoc(face, 0.5, 0.5);

    vec3 centerPos, centerNormal;
    centerLoc.Evaluate(centerPos, centerNormal);
    //  EvaluatePoint(centerLoc, centerPos, centerNormal);

    MeshVertex * centerVertex = NULL;

    // potentially one triangle for edge (assuming they're all within the view cameraModel)

    for(int e=0;e<4;e++)
    {
        CatmarkVertex * v0 = face->GetVertex(e);
        CatmarkVertex * v1 = face->GetVertex((e+1)%4);

        if(  !cameraModel.TriangleInside(v0->GetData().GetPos(),v1->GetData().GetPos(),centerPos))
            continue;

        ConvertVertex(outputMesh, v0, cameraModel.CameraCenter(), vertexMap);
        ConvertVertex(outputMesh, v1, cameraModel.CameraCenter(), vertexMap);

        if (centerVertex == NULL)
        {
            centerVertex = outputMesh->NewVertex();
            SetupVertex( centerVertex->GetData(), centerLoc, cameraModel.CameraCenter() );
            centerVertex->GetData().age = numVerts ++;
        }

        NewFace(outputMesh, vertexMap[v0], vertexMap[v1], centerVertex);
    }
}

void ConvertQuadFace(Mesh * outputMesh, CatmarkFace * face,
                     std::map<CatmarkVertex*, MeshVertex*> & vertexMap,
                     const CameraModel & cameraModel)
{
    if (!cameraModel.TriangleInside(face->GetVertex(0)->GetData().GetPos(),
                                    face->GetVertex(1)->GetData().GetPos(),
                                    face->GetVertex(2)->GetData().GetPos()) &&
            !cameraModel.TriangleInside(face->GetVertex(0)->GetData().GetPos(),
                                        face->GetVertex(2)->GetData().GetPos(),
                                        face->GetVertex(3)->GetData().GetPos()))
        return;

    for(int i=0;i<4;i++)
        ConvertVertex(outputMesh, face->GetVertex(i), cameraModel.CameraCenter(), vertexMap);

    NewFace(outputMesh, vertexMap[face->GetVertex(0)], vertexMap[face->GetVertex(1)],
            vertexMap[face->GetVertex(2)], vertexMap[face->GetVertex(3)]);
}

int IsNearBoundary(CatmarkFace * face, int distance)
{
    // is this face within "distance" of the boundary?
    // distance 1: on the boundary
    // distance 2: next to a face that's on the boundary, etc.
    //
    // a quick-n-dirty inefficient implementation

    if (distance <= 0)
        return false;

    CatmarkHalfedge * edge = face->GetFirstEdge();
    do
    {
        if (edge->IsBoundary())
            return true;

        CatmarkFace *face2 = edge->GetLeftFace() != face ? edge->GetLeftFace() : edge->GetRightFace();

        if (distance >= 2 && IsNearBoundary(face2, distance-1))
            return true;

        edge = edge->GetNext();
    } while (edge != face->GetFirstEdge());

    return false;
}

// sample an initial triangle mesh from a surface, clipping to the view frustum
Mesh * SurfaceToMesh(CatmarkMesh * sourceMesh, int subdivisionLevel,
                     const CameraModel & cameraModel, bool triangles)
{
    // subdivide the mesh up to _subdivisionLevel
    // subdividing at least once is necessary since later steps assume all faces are quads.
    int numFaces = sourceMesh->GetNumCoarseFaces();

    printf("nb faces: %d\n",numFaces);

    for(int l=0; l<subdivisionLevel; ++l ) {
        int nfaces = sourceMesh->GetNumFaces();
        for(int i=0; i<nfaces; ++i) {
            CatmarkFace * f = sourceMesh->GetFace(i);
            if(f->GetDepth()==l)
                f->Refine();
        }
    }

    Subdiv::getInstance().initialize(sourceMesh);

    Mesh * outputMesh = new Mesh;

    if (outputMesh == NULL)
    {
        printf("Mesh allocation failed");
        return NULL;
    }

    // ------------- transfer all of the faces ----------------------------

    std::map<CatmarkVertex*,MeshVertex *> vertexMap;              // Mapping from input to output vertices

    int initialNumFaces = sourceMesh->GetNumFaces();
    printf("nb faces after refine: %d\n",initialNumFaces);

    // copy all faces, creating new vertices as necessary
    for(int i=0;i<initialNumFaces; i++)
    {
        CatmarkFace * face = sourceMesh->GetFace(i);

        // skip faces that are near the boundary
        if(face->GetDepth() != subdivisionLevel || IsNearBoundary(face, pow(2, subdivisionLevel))){
            continue;
        }

        printf("Processing face %d / %d                      \r", i, initialNumFaces);

        bool extraord[4];
        int numExtraord = 0;

        // ------ create two triangles from this quad -------

        assert(face->GetNumVertices() == 4);

        // want to make sure that there's a diagonal emanating from each extraordinary vertex
        for(int j=0;j<4;j++)
        {
            extraord[j] = (face->GetVertex(j)->GetValence() != 4 || face->GetVertex(j)->OnBoundary());
            if (extraord[j])
                numExtraord ++;
        }

        if (triangles)
        {
            // check if there are adjacent extraordinary points and we need to split into four triangles
            if ((extraord[1] || extraord[3]) && (extraord[0] || extraord[2]))
                ConvertFaceToTriangles(outputMesh, face, vertexMap, cameraModel);
            else
                if (extraord[1] || extraord[3])
                {
                    ConvertFaceToTriangle(outputMesh, face, 0,1,3, vertexMap, cameraModel);
                    ConvertFaceToTriangle(outputMesh, face, 1,2,3, vertexMap, cameraModel);
                }
                else
                {
                    ConvertFaceToTriangle(outputMesh, face, 0,1,2, vertexMap, cameraModel);
                    ConvertFaceToTriangle(outputMesh, face, 0,2,3, vertexMap, cameraModel);
                }
        }
        else
            ConvertQuadFace(outputMesh, face, vertexMap, cameraModel);
    }

    printf("\n");

    printf("nb output faces: %d\n",outputMesh->GetNumFaces());

    // was the entire thing culled?
    if (outputMesh->GetNumFaces() == 0)
    {
        delete outputMesh;
        return NULL;
    }

    // generate charts.  this has to be done after the geometry is complete

    return outputMesh;
}

/////////////////////////////////////  MAIN REFINEMENT ROUTINES ////////////////////////////


bool FindContour(const ParamPointCC & p0, const ParamPointCC & p1, vec3 cameraCenter, ParamPointCC & resultPoint)
// use root-finding to find a contour point between p0 and p1, assuming that p0 and p1 share some face
//
// this is potentially very inefficient because each call to interpolate params redoes the interpolation logic
{
    //  printf("\n_________________________________________\n");

    ParamPointCC lower = p0;
    ParamPointCC upper = p1;

    real t_lower = 0;
    real t_upper = 1;
    std::map<real,real> values;

    real ndotvL, ndotvU;

    FacingType lowerFacing = Facing(lower, cameraCenter, CONTOUR_THRESHOLD, &ndotvL);
    values[t_lower] = ndotvL;

    FacingType upperFacing = Facing(upper, cameraCenter, CONTOUR_THRESHOLD, &ndotvU);
    values[t_upper] = ndotvU;

    bool lowerIsEndpoint = true, upperIsEndpoint = true;

    assert(lowerFacing != CONTOUR && upperFacing != CONTOUR && lowerFacing != upperFacing);

    for(int i=0;i<MAX_ROOT_ITERATIONS;i++)
    {
        // bisection method
        ParamPointCC nextPoint = ParamPointCC::Interpolate(lower,upper,0.5);

        if (nextPoint.IsNull() || !nextPoint.IsEvaluable())   // rare round-off error seems to happen once in awhile. seems to relate to root-finding between an extSrc and another point.
        {
            printf("UNEXPECTED NON-EVALUABLE POINT IN FIND CONTOUR at ");
            nextPoint.Print();
            return false;
        }

        real ndotv;
        FacingType newFacing = Facing(nextPoint, cameraCenter, CONTOUR_THRESHOLD, &ndotv);

        if (newFacing == CONTOUR)
        {
            resultPoint = nextPoint;
            return true;
        }

        if (newFacing == lowerFacing)
        {
            lower = nextPoint;
            ndotvL = ndotv;
            lowerIsEndpoint = false;
            t_lower = (t_lower + t_upper)/2;
            values[t_lower] = ndotv;
        }
        else
        {
            upper = nextPoint;
            ndotvU = ndotv;
            upperIsEndpoint = false;
            t_upper = (t_lower + t_upper)/2;
            values[t_upper] = ndotv;
        }
    }

    printf("WARNING: EXCEEDED MAX ITERATIONS. ndotv bounds: (%lf, %lf)\n", double(ndotvL), double(ndotvU));

    static int n = 0;

    if (n < 10)
    {
        char filename[20];
        sprintf(filename, "rootfinding-%d.txt", n++);
        FILE * fp = fopen(filename, "wt");

        for(std::map<real,real>::iterator it = values.begin(); it!=values.end(); ++it)
            fprintf(fp, "%1.64f %1.16f\n", double((*it).first), double((*it).second));
        fclose(fp);
    }


    //  assert(0);

    assert(!(upperIsEndpoint && lowerIsEndpoint));

    // if one of the bounds is an endpoint, return the other point
    if (upperIsEndpoint)
        resultPoint = lower;
    else
        if (lowerIsEndpoint)
            resultPoint = upper;
        else
            resultPoint = (fabsl(ndotvU) < fabsl(ndotvL) ? upper : lower);

    return false;
}



bool FindContour(MeshVertex * vA, MeshVertex * vB, vec3 cameraCenter, ParamPointCC & resultPoint, MeshVertex  * &  extSrc)
// find a zero-crossing between two mesh vertices, with proper handling for extraordinary vertices.
{
    extSrc = NULL;

    ParamPointCC locA = vA->GetData().sourceLoc;
    ParamPointCC locB = vB->GetData().sourceLoc;

    ParamPointCC endpointA = locA;
    ParamPointCC endpointB = locB;
    FacingType ftA = vA->GetData().facing;
    FacingType ftB = vB->GetData().facing;

    ParamPointCC sepLocA, sepLocB;

    if (vA->GetData().extraordinary && REQUIRE_SEPARATORS)
    {
        sepLocA = ParamPointCC::MakeExtraordinarySeparator(locA, locB);
        endpointA = sepLocA;
        ftA = Facing(sepLocA,cameraCenter);
        assert(sepLocA.IsEvaluable());
    }

    if (vB->GetData().extraordinary && REQUIRE_SEPARATORS)
    {
        sepLocB = ParamPointCC::MakeExtraordinarySeparator(locB, locA);
        endpointB = sepLocB;
        ftB = Facing(sepLocB, cameraCenter);
        assert(sepLocB.IsEvaluable());
    }

    // ---------------------- determine the location of the new vertex ------------------

    if (ftA != ftB && ftA != CONTOUR && ftB != CONTOUR)
        return FindContour(endpointA, endpointB, cameraCenter, resultPoint);

    assert(REQUIRE_SEPARATORS);

    if (vA->GetData().extraordinary && ftA != vA->GetData().facing)
    {
        resultPoint = sepLocA;
        extSrc = vA;
        return true;
    }

    assert(vB->GetData().extraordinary && ftB != vB->GetData().facing);
    resultPoint = sepLocB;
    extSrc = vB;
    return true;
}


bool EnsureShiftable(MeshVertex * shiftVertex, const ParamPointCC & targetLoc,
                     const vec3 & cameraCenter, Mesh * mesh,
                     PriorityQueueCatmark * wiggleQueue, PriorityQueueCatmark * splitQueue, bool testMode)
// make sure that the topology allows us to shift the shiftVertex to a new location targetLoc.
// For each edge out of the shift vertex, we want to ensure that the new location has a common AB
// parameterization with the vertex on the other end of the edge.  If not, then split the edge.
{
    // get the neighboring edges
    std::list<MeshEdge*> edges;
    shiftVertex->GetSurroundingEdges(std::back_inserter(edges));

    std::map<MeshVertex*,std::pair<ParamPointCC,vec3> > splits;  // store edges by their vertex, since edges might change during splitting

    vec3 targetPos, targetNormal;
    targetLoc.Evaluate(targetPos, targetNormal);

    // visit all adjacent vertices. check if edge is splittable and determine if splitting is required
    int i=0;
    for(std::list<MeshEdge*>::iterator it = edges.begin(); it != edges.end(); ++it, ++i)
    {
        MeshEdge * edge = *it;

        MeshVertex * v = (edge->GetOrgVertex() == shiftVertex ? edge->GetDestVertex() :
                                                                edge->GetOrgVertex());

        if (splits.find(v) != splits.end())
            continue;

        // if they have a common origin, then we're fine.
        if (ParamPointCC::HasCommonChart(v->GetData().sourceLoc, targetLoc))
            continue;

        // list of interpolants to try
        int nt = 7;
        real tcands[] = { 0.5, 0.25, 0.75, 0.125, 0.375, 0.625, 0.875 };

        bool valid = false;

        ParamPointCC newLoc;
        vec3 newPos, newNormal;
        for(int j=0;j<nt;j++)
        {
            // no common origin, so splitting is required
            newLoc = ParamPointCC::Interpolate(v->GetData().sourceLoc, shiftVertex->GetData().sourceLoc, tcands[j]);

            if (!newLoc.IsNull() && newLoc.IsEvaluable() && ParamPointCC::HasCommonChart(newLoc,targetLoc) &&
                    ParamPointCC::HasCommonChart(newLoc, v->GetData().sourceLoc))
            {
                newLoc.Evaluate(newPos, newNormal);

                // avoid vanishing edges
                if (length(newPos - targetPos) >0 && length(newPos - v->GetData().pos) > 0)
                {
                    valid = true;
                    break;
                }
            }
        }

        if (!valid && ENFORCE_SHIFTABLE)
        {
            printf("WARNING: UNABLE TO ENSURE SHIFTABLE\n");
            return false;
        }

        splits[v] = std::pair<ParamPointCC,vec3>(newLoc,newPos);
    }

    // final check for vanishing edges
    std::set<MeshFace*> faces;
    GetOneRing<VertexDataCatmark>(shiftVertex, faces);

    for(std::set<MeshFace*>::iterator fit = faces.begin(); fit != faces.end();++fit)
    {
        vec3 pos[3];

        for(int i=0;i<3;i++)
        {
            MeshVertex * v = (*fit)->GetVertex(i);
            if ( v == shiftVertex)
                pos[i] = targetPos;
            else
            {
                std::map<MeshVertex*,std::pair<ParamPointCC,vec3> >::iterator  it = splits.find(v);
                if (it == splits.end())
                    pos[i] = v->GetData().pos;
                else
                    pos[i] = (*it).second.second;
            }
        }

        for(int i=0;i<3;i++)
            if (length(pos[i] - pos[(i+1)%3]) == 0)
            {
                printf("WARNING: UNABLE TO ENSURE (vanishing edge)\n");
                return false;
            }
    }


    if (testMode)
        return true;

    // split each adjacent edge what needs splittin'

    for(std::map<MeshVertex*,std::pair<ParamPointCC,vec3> >::iterator it = splits.begin(); it != splits.end(); ++it)
    {
        MeshEdge * edge = shiftVertex->GetEdge((*it).first);

        if (edge == NULL)
            edge = (*it).first->GetEdge(shiftVertex);
        if(edge == NULL)
            continue;

        MeshVertex * vnew = mesh->NewVertex();
        VertexDataCatmark & data = vnew->GetData();
        SetupVertex(data, (*it).second.first, cameraCenter);

        data.age = numVerts ++;
        data.shiftSplit = true;

        MeshFace * oldFace = edge->GetLeftFace() == NULL ? edge->GetRightFace() : edge->GetLeftFace();

        int oppVertex = GetVertexIndex<VertexDataCatmark>(oldFace, GetOtherVertex(oldFace, shiftVertex, (*it).first));

        InsertVertex<VertexDataCatmark>(oldFace, oppVertex, vnew, mesh, *wiggleQueue, *splitQueue);
    }

    return true;
}


bool ShiftingCausesFold(MeshVertex * vsource, const ParamPointCC & newLoc)
// check if moving vsource to newLoc would cause an orientation flip or degeneracy in any adjacent face
{
    std::set<MeshFace*> faces;
    GetOneRing<VertexDataCatmark>(vsource, faces);

    for(std::set<MeshFace*>::iterator it = faces.begin(); it != faces.end(); ++it)
    {
        MeshFace * face = *it;

        int i = GetVertexIndex<VertexDataCatmark>(face, vsource);
        assert(i != -1);

        if (!ParamPointCC::SameSide(vsource->GetData().sourceLoc, newLoc,
                                    face->GetVertex((i+1)%3)->GetData().sourceLoc,
                                    face->GetVertex((i+2)%3)->GetData().sourceLoc))
            return true;
    }

    return false;
}

template<class T>
bool CanShift(HbrVertex<T> * vertex) // can probably make this generic
{
    std::set<HbrFace<T>*> oneRingFaces;
    GetOneRing<T>(vertex, oneRingFaces);
    std::set<HbrVertex<T>*> oneRingVerts;

    int numZCs = 0; // number of zero-crossings in the one-ring
    for(typename std::set<HbrFace<T>*>::iterator it = oneRingFaces.begin(); it!= oneRingFaces.end(); ++it)
    {
        HbrFace<T> * face = *it;
        int i = GetVertexIndex<T>(face, vertex);
        HbrVertex<T> * v1 = face->GetVertex((i+1)%3);
        HbrVertex<T> * v2 = face->GetVertex((i+2)%3);
        FacingType f1 = v1->GetData().facing;
        FacingType f2 = v2->GetData().facing;
        if (f1 == CONTOUR && f2 == CONTOUR)
            return false;
        if (f1 == CONTOUR && oneRingVerts.find(v1) == oneRingVerts.end())
        {
            numZCs ++;
            oneRingVerts.insert(v1);
        }
        if (f2 == CONTOUR && oneRingVerts.find(v2) == oneRingVerts.end())
        {
            numZCs ++;
            oneRingVerts.insert(v2);
        }
        if (f1 != CONTOUR && f2 != CONTOUR && f1 != f2)
            numZCs ++;

        if (numZCs > 2)
            return false;
    }
    return true;
}


// check if any of the given vertices can be moved to the location of a contour point "newLoc" that lies on the edge (vA,vB) or triangle (vA,vB,vC).
// vB and/or vC may be NULL
// If testMode is true: just check if shifting is possible without making any changes
MeshVertex * ShiftVertex(MeshVertex * vA, MeshVertex * vB, const ParamPointCC & newLoc, const vec3 & cameraCenter, 
                         Mesh * mesh, PriorityQueueCatmark * wiggleQueue, PriorityQueueCatmark * splitQueue,
                         bool testMode, bool enqueueNewFaces)
{
    MeshVertex * shiftVertex = NULL;

    vec3 limitPos, limitNormal;
    newLoc.Evaluate(limitPos, limitNormal);

    if (vB != NULL)
    {
        real vADist = length(limitPos - vA->GetData().pos);
        real vBDist = length(limitPos - vB->GetData().pos);

        real edgeDist = length(vA->GetData().pos - vB->GetData().pos);

        if (vADist / edgeDist < MAX_SHIFT_PERCENTAGE && (ALLOW_EXTRAORDINARY_INTERPOLATION || (!vA->GetData().extraordinary && vA->GetData().extSrc == NULL)) && CanShift(vA) &&
                !ShiftingCausesFold(vA, newLoc))
            shiftVertex = vA;
        else
            if (vBDist / edgeDist < MAX_SHIFT_PERCENTAGE && (ALLOW_EXTRAORDINARY_INTERPOLATION || (!vB->GetData().extraordinary && vB->GetData().extSrc == NULL)) && CanShift(vB) &&
                    !ShiftingCausesFold(vB, newLoc))
                shiftVertex = vB;
            else
                return NULL;
    }
    else
    {
        if (!(ALLOW_EXTRAORDINARY_INTERPOLATION || (!vA->GetData().extraordinary || vA->GetData().extSrc == NULL)) || !CanShift(vA) || ShiftingCausesFold(vA,newLoc))
        {
            return NULL;
        }

        shiftVertex = vA;
    }

    if (!EnsureShiftable(shiftVertex, newLoc, cameraCenter, mesh, wiggleQueue, splitQueue, testMode))
        return NULL;

    if (testMode)
        return shiftVertex;

    VertexDataCatmark & data = shiftVertex->GetData();

    if (data.origLoc.IsNull())
        data.origLoc = data.sourceLoc;
    data.isShifted = true;

    real ndotv;
    FacingType ft = Facing(limitPos - cameraCenter, limitNormal, CONTOUR_THRESHOLD, &ndotv);

    data.pos = limitPos;
    data.facing = ft;
    data.sourceLoc = newLoc;
    data.rootFindingFailed = false;
    data.ndotv = ndotv;

    // note: not touching extSrc...

    if (wiggleQueue != NULL && enqueueNewFaces)
    {
        // add the one-ring to the list of modified faces
        std::set<MeshFace*> faces;
        GetOneRing<VertexDataCatmark>(shiftVertex, faces);

        for(std::set<MeshFace*>::iterator it = faces.begin(); it != faces.end(); ++it)
        {
            MeshFace * face = *it;

            assert(GetArea<VertexDataCatmark>(face) > 0);
            
            if (!IsConsistent<VertexDataCatmark>(face, cameraCenter))
                wiggleQueue->Insert(face);
            splitQueue->Insert(face);
        }
    }
    return shiftVertex;
}

bool IsAdjacentToContourOrBoundary(MeshFace * face, PriorityQueueCatmark & wiggleQueue, PriorityQueueCatmark & splitQueue)
{
    // CCC face?
    FacingType vbf = VertexBasedFacing(face);
    if (vbf == CONTOUR)
        return true;

    for(int e=0;e<3;e++)
    {
        MeshFace * oppFace = GetOppFace(face, e);
        if (oppFace == NULL)
            return true;

        FacingType ft = VertexBasedFacing(oppFace);
        if (ft != CONTOUR && ft != vbf)
            return true;

        // if the other face is, say, CCC, and still being processed, then refine.
        // i.e., ignore the case where the opposite face cannot be refined further due to
        //       extraordinary points.  this is kind a hack that could probably be handled better elsewhere
        if (ft == CONTOUR && (splitQueue.HasFace(oppFace) || wiggleQueue.HasFace(oppFace)))
            return true;
    }

    return false;
}




real GetMinAngle(const MeshVertex * v1, const MeshVertex * v2, const MeshVertex * v3)
{
    vec3 p[3] = { v1->GetData().pos, v2->GetData().pos, v3->GetData().pos };

    real minAngle = -1;

    for(int i=0;i<3;i++)
    {
        vec3 e1 = (p[(i+1)%3] - p[i]).normalize();
        vec3 e2 = (p[(i+2)%3] - p[i]).normalize();

        real angle = acosl(e1 * e2);

#ifdef isnan
        if (isnan(angle) || isinf(angle))
            return -1;
#endif

        if (i == 0 || angle < minAngle)
            minAngle = angle;
    }

    return minAngle;
}


real EdgeLengthSquare(const MeshVertex * v1, const MeshVertex * v2)
{
    vec3 e = v1->GetData().pos - v2->GetData().pos;
    return e*e;
}

real TriangleQuality(const MeshVertex * v1, const MeshVertex * v2, const MeshVertex * v3)
{
    real area = GetArea<VertexDataCatmark>(v1,v2,v3);
    real sumLength = EdgeLengthSquare(v1,v2)+EdgeLengthSquare(v2,v3)+EdgeLengthSquare(v3,v1);
    if(sumLength > 0.0)
        return (4.0*sqrtl(3.0)*area) / sumLength;
    return 0;
}

real TriangleQuality(MeshVertex * const v[])
{
    return TriangleQuality(v[0],v[1],v[2]);
}

real TriangleQuality(const MeshFace * face)
{
    return TriangleQuality(face->GetVertex(0), face->GetVertex(1), face->GetVertex(2));
}

bool IsCuspEdge(const MeshFace * face, const std::set<std::pair<MeshVertex*,MeshVertex*> > & cuspEdges, int e)
{
    MeshVertex * v1 = face->GetVertex( (e+1)%3);
    MeshVertex * v2 = face->GetVertex( (e+2)%3);

    if (cuspEdges.find(std::pair<MeshVertex*,MeshVertex*>(v1,v2)) != cuspEdges.end() ||
            cuspEdges.find(std::pair<MeshVertex*,MeshVertex*>(v2,v1)) != cuspEdges.end()){
        return true;
    }
    return false;
}

bool IsBadEdge(const MeshFace * face,const std::set<std::pair<MeshVertex*,MeshVertex*> > & badEdges,int e)
{
    MeshVertex * v1 = face->GetVertex( (e+1)%3);
    MeshVertex * v2 = face->GetVertex( (e+2)%3);

    if (badEdges.find(std::pair<MeshVertex*,MeshVertex*>(v1,v2)) != badEdges.end() ||
            badEdges.find(std::pair<MeshVertex*,MeshVertex*>(v2,v1)) != badEdges.end())
        return true;

    if (!IsSplittable(v1,v2))
        return true;

    MeshFace * oppFace = GetOppFace(face,e);
    if (oppFace != NULL && SmallTriangle<VertexDataCatmark>(oppFace))
        return true;

    return false;
}


struct SplitCandidate
{
    int numConsistent;   // how many adjacent triangles are consistent?  OR total improvement, depends on use
    ParamPointCC splitLoc; // where is the split point
    real minQuality;  // smallest value of "quality" among the triangles
    MeshVertex * extSrc;     // if the split point is an extraordinary separator point, what is the extraordinary point
    bool isContour;     // was the split point found by root-finding?
    bool canShift;      // can this point be shifted?

    SplitCandidate() { numConsistent = -1; minQuality = -1; extSrc = NULL; }
};



struct Score   // score that a given candidate has.
{
    int numCCC;          // number of CCC triangles
    int numConsistent;   // how many triangles are consistent?  OR total improvement, depends on use
    real minQuality;  // smallest value of "quality" among the triangles
};


// for NON-zero-crossing edges, find the point on the edge with the best score.
SplitCandidate FindBestSplitPoint(MeshVertex * v1, MeshVertex * v2,
                                  const vec3 & cameraCenter,const bool allowShifts)
{
    ParamPointCC result;
    MeshVertex * extSrc = NULL;

    MeshEdge * edge = v1->GetEdge(v2);
    if (edge == NULL)
        edge = v2->GetEdge(v1);
    assert(edge != NULL); // might crash at boundaries in which case we must check v2->GetEdge(v1);

    // create a dummy vertex for the test point
    MeshVertex * vnew = v1->GetMesh()->NewVertex();
    VertexDataCatmark & data = vnew->GetData();
    data.extraordinary = false;
    data.extSrc = NULL;
    data.age = -1;

    MeshVertex * newFaces[4][3];

    MakeSplitFaceIndices(edge,vnew,newFaces);


    MeshVertex * v0 = NULL;
    MeshVertex * v3 = NULL;

    if (edge->GetLeftFace() != NULL)
        v0 = GetOtherVertex(edge->GetLeftFace(), v1,v2);
    if (edge->GetRightFace() != NULL)
        v3 = GetOtherVertex(edge->GetRightFace(), v1,v2);


    // try root-finding across the transverse edge if it has a zero-crossing
    if (edge->GetLeftFace() != NULL && edge->GetRightFace() != NULL && IsSplittable(v1,v2))
    {
        assert(edge->GetLeftFace() != edge->GetRightFace());

        // TODO: we could still do root-finding if it's not convex, by taking a point on the edge and interpolating
        // Now that the tests below do root-finding, this branch may be unnecessary

        if (v0->GetData().facing != CONTOUR && v3->GetData().facing != CONTOUR &&
                v0->GetData().facing != v3->GetData().facing &&
                ParamPointCC::ConvexInChart(v1->GetData().sourceLoc,v0->GetData().sourceLoc,v2->GetData().sourceLoc,v3->GetData().sourceLoc))
        {
            // do root-finding on the transverse edge
            FindContour(v0,v3,cameraCenter,result,extSrc);
            if(result.IsNull() || !result.IsEvaluable())
                return SplitCandidate();

            SetupVertex(data, result, cameraCenter);
            data.extSrc = extSrc;
            if (extSrc == NULL)
                data.facing = CONTOUR;


            // see if we can just shift an adjacent vertex to this contour point
            if (allowShifts && extSrc == NULL && ShiftVertex(v0,v3,result,cameraCenter,NULL,NULL,NULL,true))
            {
                SplitCandidate candidate;
                candidate.numConsistent = 0;
                candidate.splitLoc = result;
                candidate.minQuality = -1;
                candidate.extSrc = NULL;
                candidate.isContour = true;
                candidate.canShift = true;

                return candidate;
            }

            int numConsistent = 0;

            real minQuality = -1;

            for(int f=0;f<4;f++)
            {
                if (SmallTriangle<VertexDataCatmark>(newFaces[f][0],newFaces[f][1],newFaces[f][2]))
                {
                    v1->GetMesh()->DeleteVertex(vnew);
                    SplitCandidate candidate;
                    candidate.numConsistent = -1;
                    return candidate;
                }

                real quality = TriangleQuality(newFaces[f]);

                if (quality < 0)
                {
                    v1->GetMesh()->DeleteVertex(vnew);
                    SplitCandidate candidate;
                    candidate.numConsistent = -1;
                    return candidate;
                }

                real ndotv;
                if (IsConsistent<VertexDataCatmark>(newFaces[f],cameraCenter, &ndotv))
                    numConsistent ++;

                if (minQuality == -1 || quality < minQuality)
                    minQuality = quality;
            }

            v1->GetMesh()->DeleteVertex(vnew);


            SplitCandidate candidate;

            candidate.numConsistent = numConsistent;
            candidate.splitLoc = result;
            candidate.extSrc = extSrc;
            candidate.minQuality = minQuality;
            candidate.isContour = (data.facing == CONTOUR);
            candidate.canShift = false;

            return candidate;
        }
    }

    std::vector<std::pair<MeshVertex*,ParamPointCC> > shiftCandidates;
    std::vector<ParamPointCC> splitCandidates;

    // generate candidate locations

    if (!ParamPointCC::HasCommonChart(v1->GetData().sourceLoc,v2->GetData().sourceLoc))
    {
        printf("WARNING: NO COMMON ORIGIN IN FIND BEST SPLIT\n");
        v1->GetMesh()->DeleteVertex(vnew);
        return SplitCandidate();
    }

    for(int i=0;i<NUM_SPLIT_SAMPLES;i++)
    {
        real t = (i+1.0)/(NUM_SPLIT_SAMPLES + 1);
        ParamPointCC pt1 = ParamPointCC::Interpolate(v1->GetData().sourceLoc, v2->GetData().sourceLoc, t);

        if (!pt1.IsEvaluable())
            continue;

        splitCandidates.push_back(pt1);

        int halfN =  NUM_TRANSVERSE_SPLIT_SAMPLES;

        FacingType ft1 = Facing(pt1, cameraCenter);

        if (v0 != NULL && !v0->GetData().extraordinary)
        {
            if (ft1 != CONTOUR && v0->GetData().facing != CONTOUR && ft1 != v0->GetData().facing)
            {
                ParamPointCC ptnew;
                FindContour(pt1, v0->GetData().sourceLoc, cameraCenter, ptnew);
                if (!ptnew.IsNull() && ptnew.IsEvaluable())
                {
                    splitCandidates.push_back(ptnew);  // need to make note that this is a CONTOUR?
                    shiftCandidates.push_back(std::pair<MeshVertex*,ParamPointCC>(v0,ptnew));
                }
                // TODO: check if we can shift CONTOUR points
            }
            else
                for(int j=0;j<halfN;j++)
                {
                    real s = (j+1.0)/(halfN+1);
                    ParamPointCC pt2 = ParamPointCC::Interpolate(v0->GetData().sourceLoc, pt1, s);

                    if (pt2.IsEvaluable())
                        splitCandidates.push_back(pt2);
                }
        }

        if (v3 != NULL && !v3->GetData().extraordinary)
        {
            if (ft1 != CONTOUR && v3->GetData().facing != CONTOUR && ft1 != v3->GetData().facing)
            {
                ParamPointCC ptnew;
                FindContour(pt1, v3->GetData().sourceLoc, cameraCenter, ptnew);
                if (!ptnew.IsNull() && ptnew.IsEvaluable())
                {
                    shiftCandidates.push_back(std::pair<MeshVertex*,ParamPointCC>(v3,ptnew));
                    splitCandidates.push_back(ptnew);// need to make note that this is a CONTOUR?
                }
            }
            else
                for(int j=0;j<halfN;j++)
                {
                    real s = (j+1.0)/(halfN+1);
                    ParamPointCC pt3 = ParamPointCC::Interpolate(v3->GetData().sourceLoc, pt1, s);

                    if (pt3.IsEvaluable())
                        splitCandidates.push_back(pt3);
                }
        }
    }

    int bestNumConsistent = -1;
    real bestMinQuality = 0;

    for(std::vector<ParamPointCC>::iterator it = splitCandidates.begin(); it != splitCandidates.end(); ++it)
    {
        ParamPointCC pt = (*it);

        SetupVertex(data, pt, cameraCenter);

        // count number of inconsistent faces there'd be using this as the split point.

        real minQuality = -1;

        int numConsistent = 0;
        for(int f = 0;f<4;f++)
            if (newFaces[f][0] != NULL)
            {
                real ndotv;

                if (SmallTriangle<VertexDataCatmark>(newFaces[f][0],newFaces[f][1],newFaces[f][2]))
                {
                    numConsistent = -100;
                    continue;
                }

                real quality = TriangleQuality(newFaces[f]);

                if (quality < 0)
                {
                    numConsistent = -100;
                    continue;
                }

                if (IsConsistent<VertexDataCatmark>(newFaces[f],cameraCenter,&ndotv))
                {
                    numConsistent ++;
                }

                assert(ndotv <= 1 && ndotv >= -1);


                if (minQuality == -1 || quality < minQuality)
                    minQuality = quality;
            }

        if (numConsistent > bestNumConsistent)
        {
            bestNumConsistent = numConsistent;
            bestMinQuality = minQuality;
            result = pt;
        }
        else
            if (numConsistent == bestNumConsistent && minQuality > bestMinQuality)
            {
                bestMinQuality = minQuality;
                result = pt;
            }


    }

    // delete the dummy vertex
    v1->GetMesh()->DeleteVertex(vnew);

    if (bestNumConsistent < 0)
        return SplitCandidate();

    SplitCandidate candidate;

    candidate.numConsistent = bestNumConsistent;
    candidate.splitLoc = result;
    candidate.minQuality = bestMinQuality;
    candidate.extSrc = extSrc;
    candidate.isContour = false;
    candidate.canShift = false;

    return candidate;
}

bool FindBestSplitPointFace(MeshFace * face, bool allowShifts, const vec3 & cameraCenter, Mesh * mesh,
                            std::set<std::pair<MeshVertex*,MeshVertex*> > & badEdges,
                            PriorityQueueCatmark & wiggleQueue, PriorityQueueCatmark & splitQueue)
{
    bool bad[3];
    for(int i=0;i<3;i++){
        bad[i] = IsBadEdge(face, badEdges, i);
    }

#ifdef VERBOSE
    printf("\n-------------------------------------------------\n");

    FacingType faceFacing = Facing(face->GetVertex(0)->GetData().pos-cameraCenter,
                                   GetNormal(face),0);
    printf("Processing face %p SPLIT.  Verts: [%s %s %s].  Face: %s.   Bad: [%d %d %d]\n", face,
           FacingToString(face->GetVertex(0)->GetData().facing),
           FacingToString(face->GetVertex(1)->GetData().facing),
           FacingToString(face->GetVertex(2)->GetData().facing),
           FacingToString(faceFacing), bad[0], bad[1], bad[2]);
#endif      

    int maxNumConsistent = 0;

    // find the operation that leads to most consistency. break ties based on edge length.
    // if we find a useful contour shift, do that instead

    SplitCandidate candidates[3];

    for(int i=0;i<3;i++){
        if (!bad[i])
        {
            MeshVertex * v1 = face->GetVertex((i+1)%3);
            MeshVertex * v2 = face->GetVertex((i+2)%3);

            candidates[i] = FindBestSplitPoint(v1,v2,cameraCenter,allowShifts);

            if (candidates[i].numConsistent < 0)
            {
                badEdges.insert(std::pair<MeshVertex*,MeshVertex*>(v1,v2));
                continue;
            }

            if (candidates[i].numConsistent > maxNumConsistent)
                maxNumConsistent = candidates[i].numConsistent;

            if (candidates[i].canShift)
            {
                int eOpp;
                MeshFace * oppFace = GetOppFace(face,i,eOpp);
                MeshVertex * v0 = GetOtherVertex(face, v1,v2);
                MeshVertex * v3 = oppFace->GetVertex(eOpp);
                assert(v3 != v1 && v3 != v0 && v3 != v2);

                MeshVertex * shifted = ShiftVertex(v0,v3,candidates[i].splitLoc,cameraCenter,mesh,&wiggleQueue,&splitQueue,false,false);
                assert(shifted != NULL);
                shifted->GetData().facing = CONTOUR;

                // shifting might not have fixed the face or re-inserted the current one
                wiggleQueue.Insert(face);

#ifdef VERBOSE
                printf("Transverse shift\n");
#endif
                return false;  // didn't do a split, did a shift
            }
        }
    }

    int e = -1;

    real bestMinQuality = 0.0;
    for(int i=0;i<3;i++){
        if (!bad[i])
            if (candidates[i].numConsistent == maxNumConsistent && candidates[i].minQuality > bestMinQuality)
            {
                real lambda,mu;
                vec3 newPos, newNormal;
                candidates[i].splitLoc.Evaluate(newPos,newNormal);
                if(Outside<VertexDataCatmark>(face->GetVertex(0)->GetData().pos,face->GetVertex(1)->GetData().pos,face->GetVertex(2)->GetData().pos,newPos,lambda,mu)){
                    continue;
                }else{
                    bool validMove = true;
                    for(int j=1; j<3; j++){
                        MeshVertex * v0 = face->GetVertex((i+j)%3);
                        std::set<MeshFace*> oneRing;
                        GetOneRing(v0,oneRing);
                        std::set<MeshVertex*> oneRingVertices;
                        GetOneRingVertices(v0,oneRingVertices);
                        for(std::set<MeshFace*>::iterator fit=oneRing.begin(); fit!=oneRing.end() && validMove; fit++){
                            MeshFace* f = (*fit);
                            int idx = GetVertexIndex(f,v0);
                            for(std::set<MeshVertex*>::iterator it=oneRingVertices.begin(); it!=oneRingVertices.end() && validMove; it++){
                                MeshVertex* v = (*it);
                                if(v==f->GetVertex(0) || v==f->GetVertex(1) || v==f->GetVertex(2))
                                    continue;
                                if(!Outside<VertexDataCatmark>(f->GetVertex((idx+1)%3)->GetData().pos,f->GetVertex((idx+2)%3)->GetData().pos,newPos,v->GetData().pos,lambda,mu)){
                                    validMove = false;
                                    break;
                                }
                            }
                        }
                        if(!validMove)
                            break;
                    }
                    if(!validMove)
                        continue;
                }

                e = i;
                bestMinQuality = candidates[i].minQuality;
            }
    }

    if (e == -1)
        return false;

    // insert the new vertex
    MeshVertex * newVertex = mesh->NewVertex();
    SetupVertex(newVertex->GetData(),candidates[e].splitLoc,cameraCenter);
    newVertex->GetData().extSrc = candidates[e].extSrc;

    InsertVertex<VertexDataCatmark>(face, e, newVertex, mesh, wiggleQueue, splitQueue, false, false);
    if (candidates[e].isContour) // just in case
        newVertex->GetData().facing = CONTOUR;

#ifdef VERBOSE
    printf("Splitting edge of inconsistent face\n");
#endif

    return true;
}


bool FindRadialSplitPointFace(MeshFace * face, const vec3 & cameraCenter, Mesh * mesh,
                              std::set<std::pair<MeshVertex*,MeshVertex*> > & badEdges,
                              PriorityQueueCatmark & wiggleQueue, PriorityQueueCatmark & splitQueue)
{
    int bestEdge = -1;
    ParamPointCC newPoint;

    for(int i=0;i<3;i++)
    {
        ParamPointCC p0 = face->GetVertex(i)->GetData().sourceLoc;
        ParamPointCC p1 = face->GetVertex((i+1)%3)->GetData().sourceLoc;
        ParamPointCC p2 = face->GetVertex((i+2)%3)->GetData().sourceLoc;
        if (!p0.IsEvaluable() || !ParamPointCC::HasCommonChart(p1,p2))
            continue;

        vec3 viewVec = cameraCenter - face->GetVertex(i)->GetData().pos;
        ParamRayCC pr = p0.VectorToParamRay(viewVec);

        ParamPointCC pint = pr.IntersectSegment(p1,p2);
        if (!pint.IsNull())
        {
            bestEdge = i;
            newPoint = pint;
            break;
        }

        pr = p0.VectorToParamRay(-1.*viewVec);
        pint = pr.IntersectSegment(p1,p2);
        if (!pint.IsNull())
        {
            bestEdge = i;
            newPoint = pint;
            break;
        }
    }

    if (bestEdge == -1)
        return false;

    FacingType ft = Facing(newPoint, cameraCenter);

    // note: not checking the result value
    if (ft != face->GetVertex(bestEdge)->GetData().facing && face->GetVertex(bestEdge)->GetData().facing != CONTOUR)
        FindContour(newPoint, face->GetVertex(bestEdge)->GetData().sourceLoc, cameraCenter, newPoint);
    else
    {
        int eOpp;
        MeshFace * oppFace = GetOppFace(face, bestEdge,eOpp);
        if (ft != oppFace->GetVertex(eOpp)->GetData().facing && oppFace->GetVertex(eOpp)->GetData().facing != CONTOUR)
            FindContour(newPoint, oppFace->GetVertex(eOpp)->GetData().sourceLoc, cameraCenter, newPoint);
    }
    // need to try opp side too.

    // insert the new vertex
    MeshVertex * newVertex = mesh->NewVertex();
    SetupVertex(newVertex->GetData(),newPoint,cameraCenter);

    InsertVertex<VertexDataCatmark>(face, bestEdge, newVertex, mesh, wiggleQueue, splitQueue);

    return true;
}

struct ShiftCandidate
{
    int improvement;
    ParamPointCC newLoc;
    real minQuality;
    real moveDist;
};


ShiftCandidate FindBestWiggleVertex(MeshVertex * vertex, const vec3 cameraCenter, Mesh * mesh)
{
    if (vertex->GetData().extraordinary || vertex->GetData().extSrc != NULL || vertex->GetData().facing == CONTOUR)
    {
        ShiftCandidate result;
        result.improvement = -1;
        return result;
    }

    // gather the one ring and all adjacent vertices

    std::set<MeshFace*> oneRing;
    GetOneRing<VertexDataCatmark>(vertex,oneRing);

    std::set<MeshVertex*> oneRingVertices;
    GetOneRingVertices<VertexDataCatmark>(vertex, oneRingVertices);

    // count the current consistency
    int currentConsistency=0;

    for(std::set<MeshFace*>::iterator fit = oneRing.begin(); fit != oneRing.end(); ++fit)
        if (IsConsistent<VertexDataCatmark>( *fit, cameraCenter))
            currentConsistency ++;


    // save the old vertex information
    VertexDataCatmark oldData = vertex->GetData();


    int bestConsistency = -1;
    real bestMinQuality = -1;
    real bestMoveDist = 0;
    // test a bunch of sample points
    ParamPointCC bestPt;

    // try normal offsets on the current location, even if the point is extraordinary or a separator.
    if (NUM_WIGGLE_SAMPLES_SQRT > 1)
    {
        vec3 laplacian = Laplacian<VertexDataCatmark>( vertex, oneRingVertices );
        real laplacianMag = length(laplacian);

        // make sure we have an odd number, so that zero offset is one of them
        assert((NUM_NORMAL_WIGGLE_SAMPLES % 2) == 1);
        int h = (NUM_NORMAL_WIGGLE_SAMPLES - 1)/2;

        if (laplacianMag > 1e-5 && laplacianMag == laplacianMag)  // checking for NaN, don't have isnan()
        {
            for(int k=-h;k<=h;k++)
            {
                real t = (h == 0 ? 0 : real(k)/h);

                vertex->GetData().sourceLoc.SetNormalOffset(t * laplacianMag);

                vec3 normal;
                // doing much more work than necessary here.
                vertex->GetData().sourceLoc.Evaluate(vertex->GetData().pos, normal);


                // count the one-ring consistency using this sample point
                int consistency = 0;

                real minQuality = -1;
                bool badTri = false;

                for(std::set<MeshFace*>::iterator fit2 = oneRing.begin(); fit2 != oneRing.end(); ++fit2)
                {
                    if (IsConsistent<VertexDataCatmark>( *fit2, cameraCenter))
                        consistency ++;

                    real quality = 1; //TriangleQuality(*fit2);
                    if (quality <= 0 || SmallTriangle<VertexDataCatmark>(*fit2))
                    {
                        badTri = true;
                        break;
                    }

                    if (minQuality == -1 || quality < minQuality)
                        minQuality = quality;
                }

                real moveDist = length(vertex->GetData().pos - oldData.pos);

                if (!badTri)
                {
                    if (consistency > bestConsistency)
                    {
                        bestConsistency = consistency;
                        bestMinQuality = minQuality;
                        bestPt = vertex->GetData().sourceLoc;
                        bestMoveDist = moveDist;
                    }
                    else
                        if ( consistency == bestMoveDist && moveDist < bestMoveDist)
                            //		if (consistency == bestConsistency && minQuality > bestMinQuality)
                        {
                            bestMinQuality = minQuality;
                            bestPt = vertex->GetData().sourceLoc;
                            bestMoveDist = moveDist;
                        }
                }
                // restore the vertex data
                vertex->GetData() = oldData;

            }
        }
    }

    // try wiggle + normal offset
    if (!vertex->GetData().extraordinary && vertex->GetData().extSrc == NULL)
    {
        for(std::set<MeshFace*>::iterator fit = oneRing.begin(); fit != oneRing.end(); ++fit)
            for(int i=0;i<NUM_WIGGLE_SAMPLES_SQRT;i++)
            {
                MeshFace * face = *fit;
                int v = GetVertexIndex<VertexDataCatmark>(face, vertex);

                // seems weird that this would fail...
                if (!ParamPointCC::HasCommonChart(face->GetVertex( (v+1)%3)->GetData().sourceLoc,
                                                  face->GetVertex( (v+2)%3)->GetData().sourceLoc))
                    break;

                ParamPointCC oppPt = ParamPointCC::Interpolate( face->GetVertex( (v+1)%3)->GetData().sourceLoc,
                                                                face->GetVertex( (v+2)%3)->GetData().sourceLoc,
                                                                i/(NUM_WIGGLE_SAMPLES_SQRT-1.0));

                for(int j=0;j<NUM_WIGGLE_SAMPLES_SQRT;j++)
                {
                    // seems weird that this would fail...
                    if (!ParamPointCC::HasCommonChart(oldData.sourceLoc, oppPt))
                        break;

                    ParamPointCC testPt = ParamPointCC::Interpolate(oldData.sourceLoc, oppPt, (j+1.0)/(NUM_WIGGLE_SAMPLES_SQRT+1.0));

                    // check if this is a valid point to shift to, and, if so, shift
                    // note: there is a tremendous amount of rendundant computation hidden here
                    if (!testPt.IsEvaluable())
                        break;   // more extreme shifts are unlikely to work out

                    if (!ShiftVertex(vertex,NULL,testPt,cameraCenter,mesh,NULL,NULL,true))
                        break;   // more extreme shifts are unlikely to work out

                    real laplacianMag;

                    // shift the vertex but without "ensuring shiftability"
                    SetupVertex(vertex->GetData(), testPt, cameraCenter);

                    // make sure we haven't changed sign
                    if (vertex->GetData().facing != oldData.facing)
                    {
                        vertex->GetData() = oldData;
                        break;
                    }

                    // compute the laplacian vector
                    vec3 laplacian = Laplacian<VertexDataCatmark>( vertex, oneRingVertices );

                    //		printf("laplacian = %f %f %f\n", double(laplacian[0]), double(laplacian[1]), double(laplacian[2]));

                    laplacianMag = length(laplacian);

                    if (laplacianMag < 1e-5 || laplacianMag != laplacianMag)  // checking for NaN, don't have isnan()
                        laplacianMag = 1e-5;

                    // make sure we have an odd number, so that zero offset is one of them
                    assert((NUM_NORMAL_WIGGLE_SAMPLES % 2) == 1);

                    int h = (NUM_NORMAL_WIGGLE_SAMPLES - 1)/2;

                    for(int k=-h;k<=h;k++)
                    {
                        real t = (h == 0 ? 0 : real(k)/h);

                        //		  testPt.normalOffset = oldData.sourceLoc.normalOffset + t * laplacianMag;

                        testPt.SetNormalOffset(t * laplacianMag/2);

                        vec3 normal;
                        // doing much more work than necessary here.
                        testPt.Evaluate(vertex->GetData().pos, normal);

                        // count the one-ring consistency using this sample point
                        int consistency = 0;

                        real minQuality = -1;
                        bool badTri = false;

                        for(std::set<MeshFace*>::iterator fit2 = oneRing.begin(); fit2 != oneRing.end(); ++fit2)
                        {
                            if (IsConsistent<VertexDataCatmark>( *fit2, cameraCenter))
                                consistency ++;

                            real quality = 1; //TriangleQuality(*fit2);
                            if (quality <= 0 || SmallTriangle<VertexDataCatmark>(*fit2))
                            {
                                badTri = true;
                                break;
                            }

                            if (minQuality == -1 || quality < minQuality)
                                minQuality = quality;
                        }

                        real moveDist = length(vertex->GetData().pos - oldData.pos);

                        if (badTri)
                        {
                            vertex->GetData() = oldData;
                            break;
                        }

                        if (consistency > bestConsistency)
                        {
                            bestConsistency = consistency;
                            bestMinQuality = minQuality;
                            bestPt = testPt;
                            bestMoveDist = moveDist;
                        }
                        else
                            if ( consistency == bestMoveDist && moveDist < bestMoveDist)
                            {
                                bestMinQuality = minQuality;
                                bestPt = testPt;
                                bestMoveDist = moveDist;
                            }

                        // restore the vertex data
                        vertex->GetData() = oldData;
                    }
                }
            }
    }

#ifdef VERBOSE  
    printf("\nbestConsistency = %d, current = %d, oneRing size = %ld\n", bestConsistency, currentConsistency, oneRing.size());
#endif

    ShiftCandidate result;

    result.improvement = bestConsistency - currentConsistency;
    result.newLoc = bestPt;
    result.minQuality = bestMinQuality;
    result.moveDist = bestMoveDist;

    return result;
}

bool ValidMove(MeshVertex* currV, vec3 newPos, std::set<MeshFace*> & adjacentFaces)
{
    //check if moving currV would produce folds
    real lambda,mu;
    bool validMove = false;

    for(std::set<MeshFace*>::iterator fit=adjacentFaces.begin(); fit!=adjacentFaces.end() && !validMove; fit++){
        MeshFace* f = (*fit);
        if(!Outside<VertexDataCatmark>(f->GetVertex(0)->GetData().pos,f->GetVertex(1)->GetData().pos,f->GetVertex(2)->GetData().pos,newPos,lambda,mu)){
            validMove = true;
            break;
        }
    }
    std::set<MeshVertex*> oneRingVertices;
    GetOneRingVertices(currV,oneRingVertices);
    for(std::set<MeshFace*>::iterator fit=adjacentFaces.begin(); fit!=adjacentFaces.end() && validMove; fit++){
        MeshFace* f = (*fit);
        int idx = GetVertexIndex(f,currV);
        for(std::set<MeshVertex*>::iterator it=oneRingVertices.begin(); it!=oneRingVertices.end() && validMove; it++){
            MeshVertex* v = (*it);
            if(v==f->GetVertex(0) || v==f->GetVertex(1) || v==f->GetVertex(2))
                continue;
            if(!Outside<VertexDataCatmark>(f->GetVertex((idx+1)%3)->GetData().pos,f->GetVertex((idx+2)%3)->GetData().pos,newPos,v->GetData().pos,lambda,mu)){
                validMove = false;
                break;
            }
        }
    }
    return validMove;
}

int NumInconsistent(std::set<MeshFace*> & adjacentFaces, real & minQuality, const vec3 & cameraCenter)
{
    const int contourScore = 1;

    int numInconsistent = 0;
    std::vector<real> quality;
    quality.reserve(adjacentFaces.size());
    for(std::set<MeshFace*>::iterator it = adjacentFaces.begin(); it != adjacentFaces.end(); ++it){
        if (!IsConsistent(*it, cameraCenter)){
            if (IsContourFace(*it))
                numInconsistent += contourScore;
            else
                numInconsistent ++;
        }
        quality.push_back(TriangleQuality(*it));
    }
    for(std::vector<real>::iterator qIt=quality.begin(); qIt!=quality.end(); qIt++){
        if((*qIt)<minQuality)
            minQuality = (*qIt);
    }
    assert(minQuality>=0.0);
    return numInconsistent;
}

int WiggleVertexInParamSpace(MeshVertex* currV, const vec3 & cameraCenter)
{
    real u,v;
    const int NUM_SAMPLES = 21;
    const int h = (NUM_SAMPLES - 1)/2;

    std::set<MeshFace*> adjacentFaces;
    GetOneRing(currV, adjacentFaces);
    const int contourScore = 1;

    int initialNumInconsistent = 0;
    for(std::set<MeshFace*>::iterator it = adjacentFaces.begin(); it != adjacentFaces.end(); ++it)
        if (!IsConsistent(*it, cameraCenter)){
            if (IsContourFace(*it))
                initialNumInconsistent += contourScore;
            else
                initialNumInconsistent ++;
        }
    if(initialNumInconsistent==0)
        return 0;

    vec3 bestPos, bestNormal;
    int bestNumInconsistent = initialNumInconsistent;
    real bestMinQuality = 0.0;

    if(currV->GetData().facing == CONTOUR)
        return 0;

    std::set<HbrFace<Vertex>*> oneRingSourceFaces;
    if(currV->GetData().sourceLoc.SourceVertex()){
        HbrVertex<Vertex>* sourceV = currV->GetData().sourceLoc.SourceVertex();
        GetOneRing(sourceV,oneRingSourceFaces);
    }else if(currV->GetData().sourceLoc.SourceEdge()){
        HbrHalfedge<Vertex>* sourceE = currV->GetData().sourceLoc.SourceEdge();
        oneRingSourceFaces.insert(sourceE->GetLeftFace());
        oneRingSourceFaces.insert(sourceE->GetRightFace());
    }else{
        oneRingSourceFaces.insert(currV->GetData().sourceLoc.SourceFace());
    }

    vec3 oldPos = currV->GetData().pos;
    bestPos = oldPos;

    for(typename std::set<HbrFace<Vertex>*>::iterator it = oneRingSourceFaces.begin(); it != oneRingSourceFaces.end(); ++it){
        HbrFace<Vertex>* currF = (*it);
        if(!GetFaceUV(currF,currV->GetData().sourceLoc,u,v)){
            continue;
        }
        for(int du=-h; du<=h; du++){
            for(int dv=-h; dv<=h; dv++){
                if(du==0 && dv == 0)
                    continue;
                real newU = u + real(du)/real(h);
                if(newU > 1 || newU < 0)
                    continue;
                real newV = v + real(dv)/real(h);
                if(newV > 1 || newV < 0)
                    continue;
                ParamPointCC newPosParam = ParamPointCC(currF,newU,newV);
                if(newPosParam.IsNull() || !newPosParam.IsEvaluable())
                    continue;

                vec3 newPos, newNormal;
                newPosParam.Evaluate(newPos, newNormal);

                if(!ValidMove(currV,newPos,adjacentFaces))
                    continue;

                currV->GetData().pos = newPos;

                int numInconsistent = 0;
                std::vector<real> quality;
                quality.reserve(adjacentFaces.size());
                for(std::set<MeshFace*>::iterator it = adjacentFaces.begin(); it != adjacentFaces.end(); ++it)
                    if (!IsConsistent(*it, cameraCenter)){
                        if (IsContourFace(*it))
                            numInconsistent += contourScore;
                        else
                            numInconsistent ++;
                        quality.push_back(TriangleQuality(*it));
                    }
                real minQuality = FLT_MAX;
                for(std::vector<real>::iterator qIt=quality.begin(); qIt!=quality.end(); qIt++){
                    if((*qIt)<minQuality)
                        minQuality = (*qIt);
                }
                if(!(minQuality>=0.0))
                    printf("min quality = %f\n",(double) minQuality);
                assert(minQuality>=0.0);

                if (numInconsistent < bestNumInconsistent || (numInconsistent == bestNumInconsistent && minQuality > bestMinQuality)){
                    bestNumInconsistent = numInconsistent;
                    bestMinQuality = minQuality;
                    bestPos = newPos;
                    bestNormal = newNormal;
                }
                currV->GetData().pos = oldPos;
            }
        }
    }

    if(bestPos!=oldPos){
        currV->GetData().pos = bestPos;
        currV->GetData().normal = bestNormal;
    }

    return initialNumInconsistent - bestNumInconsistent;
}

int WiggleFaceVerticesInParamSpace(MeshFace* currF, MeshVertex* currV, const vec3 & cameraCenter, vec3 & bestPos, vec3 & bestNormal)
{
    std::set<MeshFace*> oneRing;
    GetOneRing(currV, oneRing);

    real bestMinQuality = FLT_MAX;
    int initialNumInconsistent = NumInconsistent(oneRing,bestMinQuality,cameraCenter);

    if(initialNumInconsistent==0)
        return 0;

    int bestNumInconsistent = initialNumInconsistent;
    vec3 oldPos = currV->GetData().pos;
    bestPos = oldPos;
    assert(!currV->GetData().sourceLoc.IsNull());

    if(currV->GetData().facing==CONTOUR){
        const int NUM_SAMPLES = 5;

        // special case to wiggle contour points: search on the smooth contour
        for(int i=0; i<3; i++){
            MeshVertex* cpt = currF->GetVertex(i);
            assert(!cpt->GetData().sourceLoc.IsNull());

            if(cpt->GetData().facing!=CONTOUR || cpt == currV)
                continue;
            if(!currV->GetEdge(cpt) || !cpt->GetEdge(currV))
                continue;

            MeshEdge* edge = currV->GetEdge(cpt);
            assert(edge);
            MeshVertex* adjVertices[2] = {GetOtherVertex(edge->GetLeftFace(),currV,cpt),GetOtherVertex(edge->GetRightFace(),currV,cpt)};
            if(adjVertices[0]->GetData().facing==adjVertices[1]->GetData().facing || adjVertices[0]->GetData().facing==CONTOUR || adjVertices[1]->GetData().facing == CONTOUR)
                continue;
            assert(!adjVertices[0]->GetData().sourceLoc.IsNull() && !adjVertices[1]->GetData().sourceLoc.IsNull());

            for(int s1=1; s1<NUM_SAMPLES; s1++){
                ParamPointCC onEdgeParam = ParamPointCC::Interpolate(currV->GetData().sourceLoc, cpt->GetData().sourceLoc,
                                                                     real(s1)/real(NUM_SAMPLES));
                if(onEdgeParam.IsNull() || !onEdgeParam.IsEvaluable())
                    continue;

                FacingType ft = Facing(onEdgeParam,cameraCenter);
                ParamPointCC newPosParam;
                if(ft==CONTOUR){
                    newPosParam = onEdgeParam;
                }else if(ft!=adjVertices[0]->GetData().facing){
                    FindContour(adjVertices[0]->GetData().sourceLoc,onEdgeParam,cameraCenter,newPosParam);
                }else{
                    assert(ft!=adjVertices[1]->GetData().facing);
                    FindContour(adjVertices[1]->GetData().sourceLoc,onEdgeParam,cameraCenter,newPosParam);
                }
                if(newPosParam.IsNull() || !newPosParam.IsEvaluable())
                    continue;

                vec3 newPos, newNormal;
                newPosParam.Evaluate(newPos, newNormal);

                if(!ValidMove(currV,newPos,oneRing))
                    continue;

                currV->GetData().pos = newPos;

                real minQuality = FLT_MAX;
                int numInconsistent = NumInconsistent(oneRing,minQuality,cameraCenter);

                if ((numInconsistent < bestNumInconsistent && minQuality > 0.25*bestMinQuality) ||
                        (numInconsistent == bestNumInconsistent && minQuality > bestMinQuality)){
                    bestNumInconsistent = numInconsistent;
                    bestMinQuality = minQuality;
                    bestPos = newPos;
                    bestNormal = newNormal;
                }
                currV->GetData().pos = oldPos;
            }
        }
    }else{
        const int NUM_SAMPLES = 11;
        // Regular case (not a contour point): search in one ring triangles
        int idx = GetVertexIndex(currF,currV);
        for(int s1=0; s1<=NUM_SAMPLES; s1++){
            assert(!currF->GetVertex((idx+1)%3)->GetData().sourceLoc.IsNull() && !currF->GetVertex((idx+2)%3)->GetData().sourceLoc.IsNull());
            ParamPointCC onEdgeParam = ParamPointCC::Interpolate(currF->GetVertex((idx+1)%3)->GetData().sourceLoc,
                                                                 currF->GetVertex((idx+2)%3)->GetData().sourceLoc,
                                                                 real(s1)/real(NUM_SAMPLES));
            if(onEdgeParam.IsNull() || !onEdgeParam.IsEvaluable()){
#if LINK_FREESTYLE
                char str[200];
                sprintf(str, "CAN'T INTERPOLATE");
                addRIFDebugPoint(-1, double(currF->GetVertex((idx+1)%3)->GetData().pos[0]), double(currF->GetVertex((idx+1)%3)->GetData().pos[1]), double(currF->GetVertex((idx+1)%3)->GetData().pos[2]), str, 0);
                addRIFDebugPoint(-1, double(currF->GetVertex((idx+2)%3)->GetData().pos[0]), double(currF->GetVertex((idx+2)%3)->GetData().pos[1]), double(currF->GetVertex((idx+2)%3)->GetData().pos[2]), str, 0);
#endif
                continue;
            }

            for(int s2=1; s2<NUM_SAMPLES; s2++){
                ParamPointCC newPosParam = ParamPointCC::Interpolate(currV->GetData().sourceLoc,onEdgeParam,
                                                                     real(s2)/real(NUM_SAMPLES));
                if(newPosParam.IsNull() || !newPosParam.IsEvaluable()){
#if LINK_FREESTYLE
                    char str[200];
                    sprintf(str, "CAN'T INTERPOLATE");
                    addRIFDebugPoint(-1, double(currV->GetData().pos[0]), double(currV->GetData().pos[1]), double(currV->GetData().pos[2]), str, 0);
#endif
                    continue;
                }

                vec3 newPos, newNormal;
                newPosParam.Evaluate(newPos, newNormal);

                if(!ValidMove(currV,newPos,oneRing))
                    continue;

                currV->GetData().pos = newPos;

                real minQuality = FLT_MAX;
                int numInconsistent = NumInconsistent(oneRing,minQuality,cameraCenter);

                if ((numInconsistent < bestNumInconsistent && minQuality > 0.25*bestMinQuality) ||
                        (numInconsistent == bestNumInconsistent && minQuality > bestMinQuality)){
                    bestNumInconsistent = numInconsistent;
                    bestMinQuality = minQuality;
                    bestPos = newPos;
                    bestNormal = newNormal;
                }
                currV->GetData().pos = oldPos;
            }
        }
    }

    return initialNumInconsistent - bestNumInconsistent;
}



int WiggleFaceVerticesInABSpace(MeshFace* face, const vec3 & cameraCenter)
{
    bool skip[3] = {true,true,true};

    std::set<MeshFace*> adjacentFaces;
    for(int i=0; i<3; i++){
        MeshVertex* currV = face->GetVertex(i);
        std::set<MeshFace*> oneRing;
        GetOneRing(currV, oneRing);
        for(std::set<MeshFace*>::iterator it = oneRing.begin(); it != oneRing.end(); ++it){
            adjacentFaces.insert(*it);
            if(!IsConsistent(*it, cameraCenter)){
                skip[i] = false;
            }
        }
    }

    real bestMinQuality = FLT_MAX;
    int initialNumInconsistent = NumInconsistent(adjacentFaces,bestMinQuality,cameraCenter);
    if(initialNumInconsistent==0)
        return 0;

    vec3 bestPos, bestNormal;
    int bestNumInconsistent = initialNumInconsistent;
    int bestIndex = -1;

    MeshFace* currentFace = face;

    std::vector<ParamPointCC> pts;
    for(int i=0; i<3; i++)
        pts.push_back(currentFace->GetVertex(i)->GetData().sourceLoc);
    std::vector<vec2> abs;
    ChartCC* chart = ParamPointCC::FindChart(pts,abs);

    if(!chart){
        for(int i=0; i<3; i++){
            MeshVertex* currV = currentFace->GetVertex(i);
            if(skip[i])
                continue;

            vec3 newPos, newNormal;
            vec3 oldPos = currV->GetData().pos;
            newPos = oldPos;
            WiggleFaceVerticesInParamSpace(currentFace,currV,cameraCenter,newPos,newNormal);
            if(oldPos == newPos)
                continue;
            currV->GetData().pos = newPos;

            real minQuality = FLT_MAX;
            int numInconsistent = NumInconsistent(adjacentFaces,minQuality,cameraCenter);

            if ((numInconsistent < bestNumInconsistent && minQuality > 0.25*bestMinQuality) ||
                    (numInconsistent == bestNumInconsistent && minQuality > bestMinQuality)){
                bestNumInconsistent = numInconsistent;
                bestMinQuality = minQuality;
                bestIndex = GetVertexIndex(face,currV);
                bestPos = newPos;
                bestNormal = newNormal;
            }
            currV->GetData().pos = oldPos;
        }
    }else{
        // Regular sampling
        const int NUM_SAMPLES = 11;
        for(int s1=1; s1<NUM_SAMPLES; s1++){
            real r1 = real(s1)/real(NUM_SAMPLES);
            for(int s2=1; s2<NUM_SAMPLES; s2++){
                real r2 = real(s2)/real(NUM_SAMPLES);
                real sr2 = sqrtl(1.0-r2);
                real beta = r1 * sr2;
                real gamma = 1.0 - sr2;
                vec2 newAB = (1.0-beta-gamma) * abs[0] + beta * abs[1] + gamma * abs[2];

                ParamPointCC newPosParam = chart->ABtoParam(newAB[0],newAB[1]);

                if(newPosParam.IsNull())// || !newPosParam.IsEvaluable())
                    continue;

                vec3 newPos, newNormal;
                newPosParam.Evaluate(newPos, newNormal);

                // Try moving one of the 3 vertices at a time and keep the best displacement
                for(int i=0; i<3; i++){
                    MeshVertex* currV = currentFace->GetVertex(i);
                    if(skip[i])
                        continue;

                    vec3 oldPos = currV->GetData().pos;

                    if(currV->GetData().facing == CONTOUR){
                        newPos = oldPos;
                        WiggleFaceVerticesInParamSpace(currentFace,currV,cameraCenter,newPos,newNormal);
                        if(oldPos == newPos)
                            continue;
                    }else{
                        //check if moving currV would produce fold
                        if(!ValidMove(currV,newPos,adjacentFaces))
                            continue;
                    }

                    currV->GetData().pos = newPos;

                    real minQuality = FLT_MAX;
                    int numInconsistent = NumInconsistent(adjacentFaces,minQuality,cameraCenter);

                    if ((numInconsistent < bestNumInconsistent && minQuality > 0.25*bestMinQuality) ||
                            (numInconsistent == bestNumInconsistent && minQuality > bestMinQuality)){
                        bestNumInconsistent = numInconsistent;
                        bestMinQuality = minQuality;
                        bestIndex = GetVertexIndex(face,currV);
                        bestPos = newPos;
                        bestNormal = newNormal;
                    }
                    currV->GetData().pos = oldPos;
                }
            }
        }
    }

    if(bestIndex>=0){
        face->GetVertex(bestIndex)->GetData().pos = bestPos;
        face->GetVertex(bestIndex)->GetData().normal = bestNormal;
    }

    return initialNumInconsistent - bestNumInconsistent;
}

void WiggleInParamSpace(Mesh * mesh, const vec3 & cameraCenter)
{
    std::list<MeshFace*> meshFaces;
    mesh->GetFaces(std::back_inserter(meshFaces));

    int reduction;
    int pass = 1;
    int prevNumInconsistent = 0;
    int idx = 0;
    do{
        int numInconsistent = 0;
        for(std::list<MeshFace*>::iterator it = meshFaces.begin(); it != meshFaces.end(); ++it)
            if (!IsConsistent(*it, cameraCenter))
                numInconsistent ++;
        printf("\n# inconsistent faces: %d\n", numInconsistent);
        if(numInconsistent == prevNumInconsistent)
            break;
        prevNumInconsistent = numInconsistent;

        reduction = 0;
        printf(" %d",pass++); fflush(stdout);
        for(std::list<MeshFace*>::iterator fit= meshFaces.begin(); fit != meshFaces.end(); ++fit){
            MeshFace* currentFace = (*fit);
            int res = WiggleFaceVerticesInABSpace(currentFace,cameraCenter);
            reduction += res;
            if(res>0){
                idx++;
                SavePLYFile(mesh,"optimization",idx);
            }
        }
        printf("(%d)",reduction);
    }while (reduction > 0);

    printf("\n");

    int numInconsistent = 0;
    for(std::list<MeshFace*>::iterator it = meshFaces.begin(); it != meshFaces.end(); ++it)
        if (!IsConsistent(*it, cameraCenter))
            numInconsistent ++;
    printf("Final # inconsistent faces: %d\n", numInconsistent);

}

bool WiggleFace(MeshFace * face, Mesh * mesh, const vec3 & cameraCenter, PriorityQueueCatmark & wiggleQueue, PriorityQueueCatmark & splitQueue)
{
    ShiftCandidate cand[3];
    int bestImprovement =0;
    real bestMoveDist = 1e10;

    int e = -1;
    for(int i=0;i<3;i++)
    {
        cand[i] = FindBestWiggleVertex(face->GetVertex(i),cameraCenter,mesh);//,pt[i]);

        if (cand[i].improvement > bestImprovement)
        {
            e = i;
            bestImprovement = cand[i].improvement;
            bestMoveDist = cand[i].moveDist;
        }
        else
            if (cand[i].improvement > 0 && cand[i].improvement == bestImprovement && cand[i].moveDist < bestMoveDist)
            {
                e = i;
                bestMoveDist = cand[i].moveDist;
            }
    }

    if (e == -1)
        return false;

#ifdef VERBOSE
    printf("\nWIGGLIN' %p.  e = %d, Improvement = %d. Quality = %f\n", face->GetVertex(e), e, cand[e].improvement,double(cand[e].minQuality));
    char debugString[200];
    face->GetVertex(e)->GetData().DebugString(debugString);
    printf("%s",debugString);
    fflush(stdout);
#endif

    assert(EnsureShiftable(face->GetVertex(e),cand[e].newLoc, cameraCenter, mesh, NULL,NULL, true));

    MeshVertex * sv = ShiftVertex(face->GetVertex(e),NULL,cand[e].newLoc,cameraCenter, mesh,&wiggleQueue, &splitQueue, false);
    assert(sv != NULL);

    return true;
}




int FlipEdge(MeshFace * face, int oppVertex, Mesh * mesh, const vec3 & cameraCenter, bool testMode,
             PriorityQueueCatmark * wiggleQueue = NULL, PriorityQueueCatmark * splitQueue = NULL, real * quality = NULL)
// testMode = true: only compute the improvement, don't actually do the flip
// testMode = false: assume the flip is valid, do the flip
{
    int eOpp;
    MeshFace * oppFace = GetOppFace(face, oppVertex, eOpp);

    if (oppFace == NULL || eOpp==-1)
        return 0;

    MeshVertex * v0 = face->GetVertex(oppVertex);
    MeshVertex * v1 = face->GetVertex( (oppVertex+1)%3 );
    MeshVertex * v2 = face->GetVertex( (oppVertex+2)%3 );
    MeshVertex * v3 = oppFace->GetVertex(eOpp);

    MeshVertex * vflip0[3] = { v0, v1, v3 };
    MeshVertex * vflip1[3] = { v0, v3, v2 };

    if (testMode)  // count how much improvement we get from the flip
    {
        // check if the flip is valid
        if  (SmallTriangle<VertexDataCatmark>(oppFace) || !IsSplittable(v1,v2) ||
             !ParamPointCC::ConvexInChart(v1->GetData().sourceLoc,v0->GetData().sourceLoc,
                                          v2->GetData().sourceLoc,v3->GetData().sourceLoc) ||
             SmallTriangle<VertexDataCatmark>(vflip0) || SmallTriangle<VertexDataCatmark>(vflip1)){
            if(quality)
                (*quality) = -1.0;
            return 0;
        }

        int currentNumInconsistent = 0;

        if (!IsConsistent<VertexDataCatmark>(face, cameraCenter))
            currentNumInconsistent ++;
        if (!IsConsistent<VertexDataCatmark>(oppFace, cameraCenter))
            currentNumInconsistent ++;

        assert(currentNumInconsistent > 0);

        int flipNumInconsistent = 0;

        real flipMinQuality = std::min<real>(TriangleQuality(vflip0), TriangleQuality(vflip1));
        if(quality)
            (*quality) = flipMinQuality;

        if (flipMinQuality <= 0)
            return -1;

        if (!IsConsistent<VertexDataCatmark>(vflip0, cameraCenter))
            flipNumInconsistent ++;
        if (!IsConsistent<VertexDataCatmark>(vflip1, cameraCenter))
            flipNumInconsistent ++;

        return currentNumInconsistent - flipNumInconsistent;
    }
    else
    {
        //  do the flip, assuming the flip is valid

        if(v0->GetEdge(v3) || v3->GetEdge(v0))
            return 0;

        wiggleQueue->Remove(face);
        wiggleQueue->Remove(oppFace);
        splitQueue->Remove(face);
        splitQueue->Remove(oppFace);

        mesh->DeleteFace(face);
        mesh->DeleteFace(oppFace);

        MeshFace * f1 = NewFaceDebug(mesh, v0, v1, v3);
        MeshFace * f2 = NewFaceDebug(mesh, v0, v3, v2);

        wiggleQueue->Insert(f1);
        wiggleQueue->Insert(f2);

        return 0;
    }
}


bool FlipFace(MeshFace * face, Mesh * mesh, const vec3 & cameraCenter, PriorityQueueCatmark & wiggleQueue, PriorityQueueCatmark & splitQueue)
{
    int bestFlipImprovement = 0;

    int e = -1;

    real bestQuality = TriangleQuality(face);

    for(int i=0;i<3;i++)
    {
        real quality = -1.0;
        int flipImprovement = FlipEdge(face, i, mesh, cameraCenter, true, NULL, NULL, &quality);
        if (flipImprovement > bestFlipImprovement || (flipImprovement == bestFlipImprovement && quality > bestQuality))
        {
            bestFlipImprovement = flipImprovement;
            bestQuality = quality;
            e = i;
        }
    }

    // do the flip
    if (e != -1)
    {
        FlipEdge(face, e, mesh, cameraCenter, false, &wiggleQueue, &splitQueue);

#ifdef VERBOSE
        printf("Edge flip.\n");
#endif
        return true;
    }

    return false;
}



void SplitZeroCrossingEdge(MeshFace * face, int oppVertex, Mesh * mesh, 
                           const vec3 & cameraCenter,  const bool allowShifts,
                           PriorityQueueCatmark & wiggleQueue,PriorityQueueCatmark & splitQueue,
                           std::pair<MeshVertex*,MeshVertex*> & badEdge,
                           const std::set<std::pair<MeshVertex*,MeshVertex*> > & cuspEdges, bool isCusp)
{
    MeshVertex * v0 = face->GetVertex(oppVertex);
    MeshVertex * v1 = face->GetVertex( (oppVertex+1)%3 );
    MeshVertex * v2 = face->GetVertex( (oppVertex+2)%3 );

    if(isCusp)
        assert(v1->GetData().facing!=CONTOUR && v2->GetData().facing!=CONTOUR);

    if (!isCusp && (SmallTriangle<VertexDataCatmark>(face) || !IsSplittable(v1,v2)))
    {
        badEdge = std::pair<MeshVertex*,MeshVertex*>(v1,v2);
        return;
    }

    int eOpp;
    MeshFace * oppFace = GetOppFace(face, oppVertex, eOpp);

    MeshVertex * v3 = NULL;

    if (oppFace != NULL)
    {
        v3 = oppFace->GetVertex(eOpp);
        assert(v3 != v1 && v3 != v2);

        if (!isCusp && SmallTriangle<VertexDataCatmark>(oppFace))
        {
            badEdge = std::pair<MeshVertex*,MeshVertex*>(v1,v2);
            return;
        }
    }

    // endpoints of the edge that we'll be splitting
    MeshVertex * vA = v1;
    MeshVertex * vB = v2;

    ParamPointCC newLoc;
    bool rootFindingFailed = false;
    MeshVertex * extSrc = NULL;


    assert (vA->GetData().facing != vB->GetData().facing &&
            vA->GetData().facing != CONTOUR && vB->GetData().facing != CONTOUR);

    // ---------------------- determine the location of the new vertex ------------------

    rootFindingFailed = !FindContour(vA, vB,cameraCenter,newLoc,extSrc);

    // --------- check if we can "shift" an old vertex instead of creating a new one ---------

    bool shift = allowShifts;
    for(std::set<std::pair<MeshVertex*,MeshVertex*> >::iterator it=cuspEdges.begin(); it!=cuspEdges.end(); it++){
        if((*it).first==vA || (*it).second==vA || (*it).first==vB || (*it).second==vB){
            shift = false;
            break;
        }
    }

    if (shift && extSrc == NULL)
    {
        MeshVertex * shifted = ShiftVertex(vA,vB,newLoc,cameraCenter,mesh,
                                           &wiggleQueue,&splitQueue,false);
        if (shifted != NULL) {
            //printf("SHIFT\n");

            shifted->GetData().facing = CONTOUR;

            // debugging
            std::set<MeshFace*> oneRing;
            GetOneRing<VertexDataCatmark>(shifted, oneRing);

            for(std::set<MeshFace*>::iterator it = oneRing.begin(); it != oneRing.end(); ++it)
                assert( (*it)->GetVertex(0)->GetData().facing != CONTOUR || (*it)->GetVertex(1)->GetData().facing != CONTOUR || (*it)->GetVertex(2)->GetData().facing != CONTOUR);

            return;
        }
    }

    // ----------- create the new vertex --------------------

    MeshVertex * newVertex = mesh->NewVertex();

    // set up data for the new vertex


    VertexDataCatmark & data = newVertex->GetData();

    SetupVertex(data, newLoc, cameraCenter);

    if (extSrc == NULL)
        data.facing = CONTOUR;
    else
        data.extSrc = extSrc;
    data.rootFindingFailed = rootFindingFailed;
    data.age = numVerts++;
    data.cusp = isCusp;
    
    // ------------------- create the new faces and delete the old one ------------

    // check if we'd be creating triangles that are too small
    if (!isCusp && (GetArea<VertexDataCatmark>(v0,v1,newVertex) < MIN_TRIANGLE_AREA || GetArea<VertexDataCatmark>(v0,newVertex,vB) < MIN_TRIANGLE_AREA ||
                    (oppFace != NULL && (GetArea<VertexDataCatmark>(newVertex, v1, v3) < MIN_TRIANGLE_AREA &&
                                         GetArea<VertexDataCatmark>(v2, newVertex, v3) < MIN_TRIANGLE_AREA))))
    {
        mesh->DeleteVertex(newVertex);
        badEdge = std::pair<MeshVertex*,MeshVertex*>(vA,vB);
        return;
    }

    InsertVertex<VertexDataCatmark>(face,oppVertex,newVertex,mesh,wiggleQueue,splitQueue);
}

bool SplitZeroCrossingFace(MeshFace * face, Mesh * mesh,const vec3 & cameraCenter, bool allowShifts,  
                           PriorityQueueCatmark & wiggleQueue, PriorityQueueCatmark & splitQueue,
                           std::set<std::pair<MeshVertex*,MeshVertex*> > & badEdges, const std::set<std::pair<MeshVertex*,MeshVertex*> > & cuspEdges,
                           bool isCusp)
{
    for(int e=0;e<3;e++)
    {
        FacingType f1 = face->GetVertex((e+1)%3)->GetData().facing;
        FacingType f2 = face->GetVertex((e+2)%3)->GetData().facing;

        if(isCusp && IsCuspEdge(face, cuspEdges, e))
            assert(f1 != CONTOUR && f2 != CONTOUR && f1 != f2);

        if (f1 != CONTOUR && f2 != CONTOUR && f1 != f2) {
            if((!IsBadEdge(face, badEdges, e) && !IsCuspEdge(face, cuspEdges, e)) || (isCusp && IsCuspEdge(face, cuspEdges, e)))
            {
                std::pair<MeshVertex*,MeshVertex*> badEdge;
                SplitZeroCrossingEdge(face, e, mesh, cameraCenter, allowShifts, wiggleQueue, splitQueue, badEdge, cuspEdges, isCusp);

                if (badEdge.first != NULL)
                    badEdges.insert(badEdge);
                else
                    return true;
            }
        }
    }
    return false;
}


void RefineContour(Mesh * mesh, const vec3 & cameraCenter, const RefinementType refinement, const bool allowShifts,
                   const int maxInconsistentSplits)
{
    PriorityQueueCatmark wiggleQueue;  // faces to be tested for shifting, wiggling, flipping improvements
    PriorityQueueCatmark splitQueue;   // faces to be split

    assert(refinement != RF_NONE);

    bool localAllowShifts = allowShifts; // && (refinement == RF_CONTOUR_INCONSISTENT || refinement == RF_FULL);

    // find all inconsistent faces and put them in the queue
    std::list<MeshFace*> faces;
    mesh->GetFaces(std::back_inserter(faces));

    for(std::list<MeshFace*>::iterator it = faces.begin(); it != faces.end(); ++it)
        if (!IsConsistent<VertexDataCatmark>(*it,cameraCenter))
            wiggleQueue.Insert(*it);

    printf("Num faces: %d\n", mesh->GetNumFaces());
    printf("Initial queue: %d\n", (int)wiggleQueue.Size());

    std::set<std::pair<MeshVertex*,MeshVertex*> > badEdges;

    int numZCs = 0;
    int numFlips = 0;
    int numSplits = 0;
    int numWiggles = 0;
    int minInc = mesh->GetNumFaces()*256;

    //---------------- MAIN LOOP: iterate over the faces until everything is consistent --------

    printf("Refining contour\n");

    assert(splitQueue.Size() == 0);
    std::set<std::pair<MeshVertex*,MeshVertex*> > cuspEdges;

    while (wiggleQueue.Size() != 0 || splitQueue.Size() != 0)
    {
        if (wiggleQueue.Size() + splitQueue.Size() < minInc && (numSplits < maxInconsistentSplits || maxInconsistentSplits < 0))
            minInc = wiggleQueue.Size()+splitQueue.Size();

        fflush(stdout);

        printf("queue size: %d+%d / %d. ZC: %d. flips: %d. splits: %d. wiggles: %d   \r",
               wiggleQueue.Size(), splitQueue.Size(), mesh->GetNumFaces(),numZCs, numFlips, numSplits, numWiggles);

        if (wiggleQueue.Size() != 0)
        {
            MeshFace * face = wiggleQueue.PopFront();

            if (IsConsistent<VertexDataCatmark>(face,cameraCenter))
                continue;

#ifdef VERBOSE
            printf("\n-------------------------------------------------\n");

            FacingType faceFacing = Facing(face->GetVertex(0)->GetData().pos-cameraCenter,
                                           GetNormal(face),0);
            printf("Processing face %p ZC.  Verts: [%s %s %s].  Face: %s. Bad: [%d %d %d]\n", face,
                   FacingToString(face->GetVertex(0)->GetData().facing),
                   FacingToString(face->GetVertex(1)->GetData().facing),
                   FacingToString(face->GetVertex(2)->GetData().facing),
                   FacingToString(faceFacing),
                   int(IsBadEdge(face,badEdges,0)),int(IsBadEdge(face,badEdges,1)),int(IsBadEdge(face,badEdges,2)));
#endif      


            // ----------------- check if any edge has a zero-crossing.  If so split that edge. -------------
            if (SplitZeroCrossingFace(face, mesh, cameraCenter, localAllowShifts,  wiggleQueue, splitQueue, badEdges, cuspEdges))
            {
                numZCs ++;
                continue;
            }

            // no zero-crossings that could be split. check if we're going to try other ways to refine
            if (refinement != RF_FULL && // !IsCCC(face,badEdges) &&
                    !(refinement == RF_CONTOUR_INCONSISTENT && IsAdjacentToContourOrBoundary(face, wiggleQueue, splitQueue)))
                continue;

            // ------------------------ try an edge flip -----------------------------------------
            if (FlipFace(face, mesh, cameraCenter, wiggleQueue, splitQueue))
            {
                numFlips ++;
                continue;
            }

            // --------------------- try wiggling each adjacent vertex -------------------------------

            if (WiggleFace(face, mesh, cameraCenter, wiggleQueue, splitQueue))
            {
                numWiggles ++;
                continue;
            }

            // we weren't able to do anything for this face, so move it to the splitqueue
            splitQueue.Insert(face);
            continue;
        }

        assert(wiggleQueue.Size() == 0);

        if (numSplits >= maxInconsistentSplits && maxInconsistentSplits >= 0)
            break;

        // -------- for an inconsistent face, try to find a way to fix it by splits ---------

        MeshFace * face = splitQueue.PopFront();

        if (IsConsistent<VertexDataCatmark>(face,cameraCenter))  // might have changed since it was added to the queue
            continue;

        bool didSplit = FindBestSplitPointFace(face, localAllowShifts, cameraCenter, mesh, badEdges, wiggleQueue, splitQueue);

        //bool didSplit = FindRadialSplitPointFace(face, cameraCenter, mesh, badEdges, wiggleQueue, splitQueue);

        if (didSplit)
            numSplits++;
    }

    if (wiggleQueue.Size() + splitQueue.Size() < minInc && (numSplits < maxInconsistentSplits || maxInconsistentSplits < 0))
        minInc = wiggleQueue.Size() + splitQueue.Size();

    printf("queue size: %d+%d / %d. ZCsplits = %d. flips = %d. splits = %d. wiggles = %d                    \n", (int)wiggleQueue.Size(),splitQueue.Size(),(int)mesh->GetNumFaces(),numZCs, numFlips, numSplits, numWiggles);

    printf("minInc = %d\n", minInc);
    if (minInc == 0)
        printf("NO INCONSISTENT TRIANGLES!\n");

}


Mesh * DuplicateMesh(Mesh * sourceMesh)
{
    Mesh * outputMesh = new Mesh;

    assert(outputMesh != NULL);

    std::map<MeshVertex*, MeshVertex*> vertexMap;

    std::list<MeshVertex*> verts;
    std::list<MeshFace*> faces;

    sourceMesh->GetVertices(std::back_inserter(verts));
    sourceMesh->GetFaces(std::back_inserter(faces));

    for(std::list<MeshVertex*>::iterator it = verts.begin(); it != verts.end(); ++it)
    {
        MeshVertex * vertex = *it;

        if (vertexMap.find(vertex) == vertexMap.end())
        {
            MeshVertex * newv = outputMesh->NewVertex();
            newv->GetData() = vertex->GetData();
            vertexMap[vertex] = newv;
        }
    }

    for(std::list<MeshFace*>::iterator it = faces.begin(); it != faces.end(); ++it)
    {
        MeshFace * face = *it;

        int vtx[3] = { vertexMap[face->GetVertex(0)]->GetID(),
                       vertexMap[face->GetVertex(1)]->GetID(),
                       vertexMap[face->GetVertex(2)]->GetID()};

        outputMesh->NewFace(3,vtx,0);
    }
    return outputMesh;
}


struct PointData
{
    MeshVertex * vertex;
    MeshVertex * frontOffset;
    MeshVertex * backOffset;
    vec3 vertexNormal;
    vec3 frontOffsetNormal;
    vec3 backOffsetNormal;

    PointData() { vertex = frontOffset = backOffset = NULL; }
};


int VertexDataCatmark::DebugString(char * str) const
{
    int type = -1;

    char typeString[20];

    if (degenerate)
        strcpy(typeString,"DEGENERATE");
    else
        if (rootFindingFailed)
            strcpy(typeString,"ROOT-FINDING-FAILED");
        else
        {
            switch(facing)
            {
            case FRONT: type = 0; strcpy(typeString,"FRONT"); break;
            case BACK: type = 1; strcpy(typeString,"BACK"); break;
            case CONTOUR: type = 2; strcpy(typeString,"CONTOUR"); break;
            }
        }

    real ndotvx = ndotv;

    char srcStr[1000];
    sourceLoc.PrintString(srcStr);

    sprintf(str, "%s (%p)\n\tndotv = %f\n\t%s\n\textraordinary: %s\n\textSrc: %p\n\tisShifted: %s\n\tshiftSplit: %s\n"
            "\trootFindingFailed: %s\n\tdegenerate: %s\n\tage: %d\n\tradialCurvature: %f\n\tk1: %f\n\tk2: %f\n\tcusp: %s\n\tradial: %s\n",
            typeString, this, double(ndotvx),
            srcStr,
            extraordinary ? "true" : "false",
            extSrc,
            isShifted ? "true" : "false",
            shiftSplit ? "true" : "false",
            rootFindingFailed ? "true":"false",
            degenerate ? "true":"false",
            age,
            double(radialCurvature),
            double(k1),
            double(k2),
            cusp ? "true":"false",
            Radial() ? "true":"false");

    return type;
}

void ComputeRadialCurvatures(Mesh * mesh, const CameraModel & camera, real isovalue, int maxIsophoteDistance)
{
    printf("Computing radial curvatures and isophote distance.\n");

    std::list<MeshVertex*> verts;
    mesh->GetVertices(std::back_inserter(verts));

    for(std::list<MeshVertex*>::iterator it = verts.begin(); it != verts.end(); ++it)
    {
        // radial curvature

        real k_r;
        VertexDataCatmark & data = (*it)->GetData();
        bool result = data.sourceLoc.RadialCurvature(camera.CameraCenter(), k_r);
        if (!result)
            k_r = -123456;
        (*it)->GetData().radialCurvature = k_r;

        // isophote distance
        data.isophoteDistance = data.sourceLoc.IsophoteDistance(camera, isovalue, maxIsophoteDistance);
    }
}

void ComputeConsistencyStats(HbrMesh<VertexDataCatmark> * outputMesh, const vec3 & cameraCenter, int & numInconsistent, int & numStrongInconsistent,
                             int &numNonRadial, int &numInconsistentContour, int &numInconsistentRadial)
{

    printf("Checking consistency\n");

    std::list<HbrFace<VertexDataCatmark>*> faces;
    outputMesh->GetFaces(std::back_inserter(faces));

    int i = 0;

    for(std::list<HbrFace<VertexDataCatmark>*>::iterator it = faces.begin(); it != faces.end(); ++it)
    {
        if(i%40==0)
            printf("Face: %d / %ld.   Count: %d, %d      \r", i, faces.size(), numInconsistent, numStrongInconsistent);
        i++;

        HbrFace<VertexDataCatmark> * face = *it;

        if(!IsStandardRadialFace(face))
        {
            numNonRadial++;
        }

        if (!IsConsistent(face, cameraCenter))
        {
            numInconsistent ++;
            for(int e=0;e<3;e++) {
                if(face->GetVertex(e)->GetData().facing == CONTOUR){
                    numInconsistentContour++;
                    if(IsStandardRadialFace(face)){
                        numInconsistentRadial++;
#if LINK_FREESTYLE
                        char str[200];
                        sprintf(str, "INCONSISTENT RADIAL");
                        addRIFDebugPoint(-1,double(face->GetVertex(0)->GetData().pos[0]),double(face->GetVertex(0)->GetData().pos[1]),double(face->GetVertex(0)->GetData().pos[2]),str,0);
                        addRIFDebugPoint(-1,double(face->GetVertex(1)->GetData().pos[0]),double(face->GetVertex(1)->GetData().pos[1]),double(face->GetVertex(1)->GetData().pos[2]),str,0);
                        addRIFDebugPoint(-1,double(face->GetVertex(2)->GetData().pos[0]),double(face->GetVertex(2)->GetData().pos[1]),double(face->GetVertex(2)->GetData().pos[2]),str,0);
#endif
                    }
                    break;
                }
            }
        }

        if (true ||NUM_INCONSISTENT_SAMPLES == 0)
            continue;

        bool strongInconsistent = false;

        for(int e=0;e<3;e++)
        {
            int eOpp;
            MeshFace * oppFace = GetOppFace(face,e,eOpp);

            // skip edges adjacent to inconsistent faces
            if (oppFace != NULL && !IsConsistent(oppFace, cameraCenter))
                continue;

            // do each edge only once
            if (oppFace != NULL && oppFace < face)
                continue;


            ParamPointCC p0 = face->GetVertex(e)->GetData().sourceLoc;
            ParamPointCC p1 = face->GetVertex((e+1)%3)->GetData().sourceLoc;
            FacingType ft0 = face->GetVertex(e)->GetData().facing;
            FacingType ft1 = face->GetVertex((e+1)%3)->GetData().facing;

            if (ft0 == CONTOUR && ft1 == CONTOUR)
                continue;

            assert(ft0 == CONTOUR || ft1 == CONTOUR || ft0 == ft1);

            FacingType ft = (ft0 != CONTOUR ? ft0: ft1);

            // check for zero-crossings by sampling

            real t = FindZeroCrossingBySampling( p0, p1, ft, cameraCenter);


            if (t != -1)
            {
                strongInconsistent = true;

#ifdef LINK_FREESTYLE
                ParamPointCC zeroCrossingPoint = ParamPointCC::Interpolate(p0, p1, t);
                zeroCrossingPoint.SetNormalOffset(0);

                vec3 pos = (1-t)*face->GetVertex(e)->GetData().pos + t*face->GetVertex((e+1)%3)->GetData().pos;
                //                vec3 pos2, normal;
                //                zeroCrossingPoint.Evaluate(pos2,normal);

                char * str = new char[200];
                zeroCrossingPoint.PrintString(str);

                real ndotv;
                FacingType myFacing = Facing(zeroCrossingPoint, cameraCenter, CONTOUR_THRESHOLD, &ndotv);

                char * str2 = new char[300];
                sprintf(str2, "STRONG INCONSISTENCY.\nft0 = %s, ft1 = %s, myFacing = %s, ndotv = %f, t= %f, %s",
                        FacingToString(ft0), FacingToString(ft1), FacingToString(myFacing), double(ndotv), double(t), str);

                addRIFDebugPoint(-1, pos[0], pos[1], pos[2], str2, 0);
#endif
            }
        }

        if (strongInconsistent)
            numStrongInconsistent ++;
    }
}

bool IsSplittable(MeshVertex * v0, MeshVertex * v1)
// is this an edge splittable?  
{
    VertexDataCatmark & d0 = v0->GetData();
    VertexDataCatmark & d1 = v1->GetData();

    if (ALLOW_EXTRAORDINARY_INTERPOLATION && !REQUIRE_SEPARATORS)
        return true;

    // not splittable if the edge connects an extraordinary point with its separator
    if (d0.extSrc == v1 || d1.extSrc == v0)
        return false;

    // not splittable if the edge connects two separators of the same extraordinary point
    if (d0.extSrc != NULL && d0.extSrc == d1.extSrc)
        return false;

    return true;
}
