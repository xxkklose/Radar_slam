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

    Eigen::Matrix4f getInverse(const Eigen::Matrix4f& matrix)
    {
        Eigen::Matrix4f inv = Eigen::Matrix4f::Identity();
        inv.block<3,3>(0,0) = matrix.block<3,3>(0,0).transpose();
        inv.block<3,1>(0,3) = -inv.block<3,3>(0,0) * matrix.block<3,1>(0,3);
        return inv;
    }
};


