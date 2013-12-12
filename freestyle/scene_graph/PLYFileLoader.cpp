#include <float.h>

#include "NodeShape.h"
#include "IndexedFaceSet.h"
#include "PLYFileLoader.h"
#include <locale.h>

PLYFileLoader::PLYFileLoader(const char *iFileName)
{
    _FileName = new char[strlen(iFileName)+1];
    strcpy(_FileName, iFileName);

    _Scene = NULL;
    _numFacesRead = 0;
    _minEdgeSize = DBL_MAX;
}

PLYFileLoader::~PLYFileLoader()
{
    if(NULL != _FileName)
    {
        delete [] _FileName;
        _FileName = NULL;
    }

    _Scene = NULL;
}

/*
void PLYFileLoader::SetFileName(const char *iFileName)
{
  if(NULL != _FileName)
    delete [] _FileName;

  _FileName = new char[strlen(iFileName)+1];
  strcpy(_FileName, iFileName);
  }*/

NodeGroup* PLYFileLoader::Load()
{
    printf("Loading PLY file %s\n", _FileName);

    FILE * fp = fopen(_FileName, "rt");
    if (fp == NULL)
    {
        printf("ERROR: CANNOT OPEN INPUT FILE %s\n", _FileName);
        exit(1);
    }

    // ---------- Read the headers ---------

    char buffer[200];

    unsigned numVertices = 0, numFaces = 0;
    fscanf(fp, "ply\nformat ascii 1.0\n");

    fgets(buffer, 200, fp);
    bool meshSilhouettes = (strcmp(buffer, "comment mesh silhouettes\n") == 0);
    if (!meshSilhouettes && (strcmp(buffer, "comment smooth silhouettes\n") != 0))
    {
        printf("missing comment indicating smooth vs. mesh silhouettes. line: %s\n", buffer);
        exit(1);
    }

    fscanf(fp,"element vertex %d\n", &numVertices);
    fscanf(fp, "property float x\nproperty float y\nproperty float z\n");

    fscanf(fp, "property float nx\n");
    fscanf(fp, "property float ny\n");
    fscanf(fp, "property float nz\n");

    fscanf(fp, "property float red\n");
    fscanf(fp, "property float green\n");
    fscanf(fp, "property float blue\n");
    fscanf(fp, "property float ndotv\n");

    fscanf(fp,"element face %d\n", &numFaces);

    if (numVertices <= 0 || numFaces <= 0)
    {
        printf("ERROR:  READING PLY (nv = %d, nf = %d)\n", numVertices, numFaces);
        exit(1);
    }

    fscanf(fp, "property list uchar int vertex_index\n");
    fscanf(fp, "property uchar int\n");  // vbf
    fscanf(fp, "end_header\n");

    // ------ Initialize data structures ------

    // create of the scene root node
    _Scene = new NodeGroup;
    NodeShape * shape = new NodeShape;
    _Scene->AddChild(shape);

    // allocate elements for the indexed face set
    real * vertices = new real[3*numVertices];
    unsigned * nvertPerFace = new unsigned[numFaces];
    IndexedFaceSet::TRIANGLES_STYLE * faceStyles = new IndexedFaceSet::TRIANGLES_STYLE[numFaces];
    unsigned * faces = new unsigned[3*numFaces];
    _numFacesRead = numFaces;

    unsigned numNormals = numVertices;
    real * normals = new real[numNormals * 3];
    unsigned * nindices = new unsigned[numFaces * 3];

    int * faceUserData = new int[numFaces];
    float * vertexUserData = new float[numVertices];

    real minBBox[3] = { 0,0,0};
    real maxBBox[3] = { 0,0,0};
    //  real minBBox[3] = { DBL_MAX, DBL_MAX, DBL_MAX };
    //  real maxBBox[3] = { DBL_MIN, DBL_MIN, DBL_MIN };

    printf("Num Vertices = %d, Num Faces = %d\n", numVertices, numFaces);

    // ------- Read the vertices and faces -----
    for(unsigned i=0;i<numVertices;i++)
    {
        if (feof(fp) != 0)
        {
            printf("UNEXPECTED EOF IN PLY\n");
            exit(1);
        }

        // note: should be using strtod

        char buffer[200];
        fgets(buffer, 200, fp);

        real x,y,z, ndotv;
        char * nextptr;

        setlocale(LC_NUMERIC,"C");
        x = strtod(buffer, &nextptr);
        y = strtod(nextptr, &nextptr);
        z = strtod(nextptr, &nextptr);

        vertices[3*i] = x;
        vertices[3*i+1] = y;
        vertices[3*i+2] = z;

        for(int j=0;j<3;j++)
        {
            if (vertices[3*i+j] < minBBox[j] || i == 0)
                minBBox[j] = vertices[3*i+j];
            if (vertices[3*i+j] > maxBBox[j] || i == 0)
                maxBBox[j] = vertices[3*i+j];
        }

        // per-vertex normals
        real nx, ny, nz;

        nx = strtod(nextptr, &nextptr);
        ny = strtod(nextptr, &nextptr);
        nz = strtod(nextptr, &nextptr);

        Vec3r normal(nx,ny,nz);
        normal.normalize();
        for(int j=0;j<3;j++)
            normals[3*i+j]=normal[j];

        // per-vertex color
        real red, green, blue;

        red = strtod(nextptr, &nextptr);
        green = strtod(nextptr, &nextptr);
        blue = strtod(nextptr, &nextptr);

        ndotv = strtod(nextptr, &nextptr);
        vertexUserData[i] = ndotv;

        //if (i < 3 || i+4 > numVertices )
        //printf("Vertex %d: %f %f %f (%g %g %g)\n", i, vertices[3*i], vertices[3*i+1], vertices[3*i+2],x,y,z);
    }


    for(unsigned i=0;i<numFaces;i++)
    {
        if (feof(fp) != 0)
        {
            printf("UNEXPECTED EOF IN PLY\n");
            exit(1);
        }

        //      char buffer[200];
        //      fgets(buffer, 200, fp);

        int N, v[3], vfint;
        int r = fscanf(fp, "%d %d %d %d %d\n", &N, &v[0], &v[1], &v[2], &vfint);
        faces[3*i] = 3*v[0];  // why multiply by 3?  no idea (there's a mysterious division by 3 is in WingedEdgeBuilder::buildTriangles)
        faces[3*i+1] = 3*v[1];
        faces[3*i+2] = 3*v[2];
        //      fscanf(fp, "%d %d %d %d\n", &N, &faces[3*i], &faces[3*i+1], &faces[3*i+2]);

        //      if (i <5  || i +3 > numFaces -1)
        //      	printf("Face %d: %d verts: %d %d %d (r = %d)\n", i, N, v[0], v[1], v[2], r);

        if (N != 3)
        {
            printf("UNEXPECTED NON-TRIANGULAR FACE IN PLY %d: %d vertices)\n", i, N);
            exit(1);
        }

        nvertPerFace[i] = N;
        faceStyles[i] = IndexedFaceSet::TRIANGLES;

        faceUserData[i] = vfint;  // vbf goes here

        Vec3r vert[3];
        for(int j=0;j<3;j++)
            for(int k=0;k<3;k++)
                vert[j][k] = vertices[3*v[j] + k];

        for(int j=0; j<3; j++)
        {
            real norm = sqrt((vert[j] - vert[(j+1)%3])*(vert[j] - vert[(j+1)%3]));
            if(_minEdgeSize > norm)
                _minEdgeSize = norm;
        }

//        if (meshSilhouettes)  // per-face normals
//        {
//            Vec3r normal = (vert[2] - vert[0]) ^ (vert[1] - vert[0]);
//            normal.normalize();
//            for(int j=0;j<3;j++) // 3 entries of the normal vector
//                normals[3*i+j] = normal[j];
//            for(int j=0;j<3;j++) // normal for each vertex
//                nindices[3*i+j] = 3*i;   // mysterious factor of 3 (see WingedEdgeBuilder::buildTriangles)
//        }
//        else
        for(int j=0;j<3;j++) // per-vertex normals
            nindices[3*i+j] = 3*v[j];
    }



    // -------- create the indexed face set and finish up

    IndexedFaceSet * rep = new IndexedFaceSet(vertices, 3*numVertices, normals, 3*numNormals, NULL, 0, 0, 0,
                                              numFaces, nvertPerFace, faceStyles, faces, 3*numFaces, nindices, 3*numFaces, NULL, 0, NULL, 0,
                                              faceUserData, vertexUserData, 0, meshSilhouettes); // set to zero means it will be deallocated elsewhere

    rep->SetId(Id(0,0));

    const BBox<Vec3r> bbox(Vec3r(minBBox[0], minBBox[1], minBBox[2]),
                           Vec3r(maxBBox[0], maxBBox[1], maxBBox[2]));
    rep->SetBBox(bbox);
    shape->AddRep(rep);

    //  printf("dbl_min = %f, dbl_max = %f\n", DBL_MIN, DBL_MAX);
    //  printf("bbox: %f, %f, %f ; %f %f %f\n", minBBox[0], minBBox[1], minBBox[2],
    //         maxBBox[0], maxBBox[1], maxBBox[2]);
    //  printf("bbox: %f, %f, %f ; %f %f %f\n", bbox.getMin()[0], bbox.getMin()[1], bbox.getMin()[2],
    //         bbox.getMax()[0], bbox.getMax()[1], bbox.getMax()[2]);

    //Returns the built scene.
    return _Scene;
}



