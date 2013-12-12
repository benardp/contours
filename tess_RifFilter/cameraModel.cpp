#include "cameraModel.h"


CameraModel::CameraModel(const mat4 & cameraMatrix, 
                         float near, float far, float left, float right, float top, float bottom,
                         float xres, float yres, float focalLength,vec3 cameraCenter)
{
    _cameraMatrix = cameraMatrix;
    _near = near;
    _far = far;
    _left = left;
    _right = right;
    _top = top;
    _bottom = bottom;
    _xres = xres;
    _yres = yres;
    _cameraCenter = cameraCenter;
    _focalLength = focalLength;
}


bool CameraModel::PointInside(const vec3 & pt) const
{
    vec4 pt2 = vec4(pt[0],pt[1],pt[2],1) * _cameraMatrix;

    bool result =
            _focalLength * pt2[0] >= _left * pt2[2]   && _focalLength * pt2[0] <= _right * pt2[2] &&
            _focalLength * pt2[1] >= _bottom * pt2[2] && _focalLength * pt2[1] <= _top * pt2[2] &&
            pt2[2] >= _near && pt2[2] <= _far;

    return result;
}

bool CameraModel::TriangleInside(const vec3 & p0, const vec3 & p1, const vec3 & p2) const
{
    vec3 p[3] = {p0, p1, p2};

    vec3 proj[3];
    for(int i=0;i<3;i++)
    {
        vec3 pt = p[i];
        vec4 pt2 = vec4(pt[0],pt[1],pt[2],1) * _cameraMatrix ;
        proj[i][0] = _focalLength * pt2[0]/pt2[2];
        proj[i][1] = _focalLength * pt2[1]/pt2[2];
        proj[i][2] = pt2[2];
    }

    // it is possible that all three vertices are outside the view frustum, but the triangle itself
    // intersects the frustum anyway.  this is avoiding by checking if all three points are clipped
    // with respect to the same clipping plane as each other

    if (proj[0][0] < _left   && proj[1][0] < _left   && proj[2][0] < _left)
        return false;

    if (proj[0][0] > _right  && proj[1][0] > _right  && proj[2][0] > _right)
        return false;

    if (proj[0][1] < _bottom && proj[1][1] < _bottom && proj[2][1] < _bottom)
        return false;

    if (proj[0][1] > _top    && proj[1][1] > _top    && proj[2][1] > _top)
        return false;

    if (proj[0][2] < _near   && proj[1][2] < _near   && proj[2][2] < _near)
        return false;

    if (proj[0][2] > _far    && proj[1][2] > _far    && proj[2][2] > _far)
        return false;

    return true;
}



vec3 CameraModel::Project(const vec3 & pt) const
{
    vec4 pt2 = vec4(pt[0],pt[1],pt[2],1) * _cameraMatrix;
    real x = _focalLength * pt2[0]/pt2[2];
    real y = _focalLength * pt2[1]/pt2[2];

    x *= _xres / (_left - _right);
    y *= _yres / (_top - _bottom);

    return vec3(x,y,0);  // third value needs to be constant for the area calculation
}


real CameraModel::ImageSpaceArea(const vec3 & p0, const vec3 & p1, const vec3 & p2) const
{
    vec3 q[3] = { Project(p0), Project(p1), Project(p2) };

    vec3 v1 = q[1] - q[0];
    vec3 v2 = q[2] - q[0];

    vec3 n = v2 ^ v1;

    return 0.5*n*n;
}



