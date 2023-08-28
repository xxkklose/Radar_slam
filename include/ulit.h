#pragma once

#include <math.h>
#include <cmath>
#include <Eigen/Eigen>
using namespace Eigen;

namespace ulit
{
    double inline EuclideanNorm(double x, double y, double z)
    
    {
        return sqrt(x*x + y*y + z*z);
    }
    double inline EuclideanNorm(Vector3d point)
    {
        return sqrt(point.x()*point.x() + point.y()*point.y() + point.z()*point.z());
    }
};


