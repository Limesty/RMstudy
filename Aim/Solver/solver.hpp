#ifndef ANGELSOLVER_H
#define ANGELSOLVER_H

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

class Solver
{
public:
    const double PI = 3.14159265358979;
    const double PTICH_HALF_PERSPECT = 0.23;
    const double YAW_HALF_PERSPECT = 0.36;
    Solver();
    ~Solver();
    Point2f solve(Point2f LeftUpPoint, Point2f RightDownPoint, Size size);
};

#endif