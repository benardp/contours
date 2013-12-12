#include <Python.h>

// Data structure for passing Matrix data between C++ and Python
struct Matrix4x4
{
public:
    float data[4][4];
    /*  float data_00, data_10, data_20, data_30,
    data_01, data_11, data_21, data_31,
    data_02, data_12, data_22, data_32,
    data_03, data_13, data_23, data_33;
  */

    Matrix4x4() { }
    Matrix4x4(float d_00, float d_10, float d_20, float d_30,
              float d_01, float d_11, float d_21, float d_31,
              float d_02, float d_12, float d_22, float d_32,
              float d_03, float d_13, float d_23, float d_33)
    {
        data[0][0] = d_00;
        data[1][0] = d_10;
        data[2][0] = d_20;
        data[3][0] = d_30;

        data[0][1] = d_01;
        data[1][1] = d_11;
        data[2][1] = d_21;
        data[3][1] = d_31;

        data[0][2] = d_02;
        data[1][2] = d_12;
        data[2][2] = d_22;
        data[3][2] = d_32;

        data[0][3] = d_03;
        data[1][3] = d_13;
        data[2][3] = d_23;
        data[3][3] = d_33;
    }

    Matrix4x4 operator*(const Matrix4x4 & m2) const
    {
        Matrix4x4 out;
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
            {
                out.data[i][j] = 0;
                for(int k=0;k<4;k++)
                    out.data[i][j] += data[i][k] * m2.data[k][j];
            }
        return out;
    }

    void print()
    {
        printf("[");
        for(int i=0;i<4;i++)
            printf("%f %f %f %f;",data[i][0],data[i][1],data[i][2],data[i][3]);
        printf("]\n");
    }
};


void addStyleFS(const char * styleFilename);
void clearStylesFS();

void run(const char * meshFilename, const char * snapshotFilename, const char * outputEPSPolyline, const char * outputEPSThick,
         Matrix4x4 worldTransform,
         float top, float bottom, float left, float right,
         float pixelaspect, float aspectratio,
         float near, float far, float focalLength,
         int outputWidthArg, int outputHeightArg,
         int windowWidthArg, int windowHeightArg,
         int visAlgorithm, bool useConsistency, bool runInteractive,
         double cuspTrimThreshold, double graftThreshold, double wiggleFactor,
         const char * pythonLibPath, bool saveLayers);

#ifndef CHECK_FOR_ERROR
#  ifdef NDEBUG
#     define CHECK_FOR_ERROR   ;
#  else
#     define CHECK_FOR_ERROR   checkForError(__FILE__, __LINE__, __func__);
#  endif
#endif



