#ifndef __CAMERA_MODEL_H__
#define __CAMERA_MODEL_H__

#include "VecMat.h"
using namespace VecMat;

class CameraModel
{
private:
    float _near, _far; // clipping planes
    float _left, _right, _top, _bottom; // viewport
    mat4 _cameraMatrix;
    float _xres, _yres;
    vec3 _cameraCenter;
    float _focalLength;
public:
    CameraModel(const mat4 & cameraMatrix, float near, float far, float left, float right, float top, float bottom, float xres, float yres, float focalLength, vec3 cameraCenter);
    bool PointInside(const vec3 & pt) const;
    bool TriangleInside(const vec3 & p0, const vec3 & p1, const vec3 & p2) const;
    vec3 Project(const vec3 & p) const;
    vec3 CameraCenter() const { return _cameraCenter; }
    real ImageSpaceArea(const vec3 & p0, const vec3 & p1, const vec3 & p2) const;
    real ImageSpaceDistance(const vec3 & p0, const vec3 & p1) const { return length(Project(p0) - Project(p1)); }
};



#endif
