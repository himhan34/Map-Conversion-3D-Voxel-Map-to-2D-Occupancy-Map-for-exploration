#pragma once
#include <vector>
#include <Eigen/Dense>
#include "ros/ros.h"
#include "ros/console.h"

using namespace std;
using namespace Eigen;

struct point_int{
    int x,y;
};

point_int DIRECTIONS[]={{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1}};

struct point2D{
    double x,y;
};

struct point3D{
    double x,y,z;
};

struct voxel{
    point3D position;
    float halfSize;
    bool occupied;
};

struct heightRange{
    double top, bottom;
};




